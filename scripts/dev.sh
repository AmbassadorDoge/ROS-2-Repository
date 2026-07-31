#!/usr/bin/env bash
# Run a command inside the ROS 2 Jazzy dev container, with this repo mounted.
#
#   scripts/dev.sh colcon build --packages-select drivebase_behaviour
#   scripts/dev.sh python3 -m pytest src/drivebase_behaviour/test -v
#   scripts/dev.sh ros2 launch drivebase_sim sim.launch.py rviz:=true
#   scripts/dev.sh                      # interactive shell
#
# WHY THIS EXISTS. There is no native ROS 2 or Gazebo on the development
# machine. Everything runs in the container, and every verification step in
# docs/superpowers/plans/ assumes this wrapper.
#
# RENDERING. On Linux with a GPU this passes /dev/dri and the X socket
# straight through, so Gazebo and RViz render on the real GPU and open real
# windows. Set DRIVEBASE_SOFTWARE_GL=1 to force llvmpipe instead, and
# DRIVEBASE_HEADLESS=1 to run with no display at all (CI, tests, anything
# non-interactive).
#
# Under software rendering, what is and is not trustworthy:
#
#   Trustworthy: geometry, tf, kinematics, frame layout, detection logic,
#   state transitions.
#
#   NOT trustworthy: control-loop rates, Nav2 timing, anything phrased as Hz
#   or "how long did it take". docs/STATUS.md records a Nav2 stall that may be
#   nothing more than a starved controller. Timing figures are only worth
#   believing when they come from a GPU run.
#
# BUILD ARTIFACTS LIVE IN DOCKER VOLUMES, not in the repo. The repo sits on an
# NTFS/fuseblk mount that silently drops the executable bit, so a colcon
# install tree written there produces console scripts that cannot be run and
# `ros2 run` fails with nothing useful to go on. build/, install/ and log/ are
# therefore named volumes. Consequence: they are invisible from the host, and
# `scripts/dev.sh rm -rf build install log` is how you clean, not `rm` on the
# host. `scripts/dev.sh --reset` drops the volumes outright.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="drivebase-dev:jazzy"
IMAGE="drivebase-dev:jazzy-pod"
DOCKER="${DOCKER:-docker}"

# `docker images -q`, not `docker image inspect`: inspect has been observed
# returning "No such image" for an image that `docker images` lists by ID in
# the same second, which made this wrapper spuriously try to rebuild.
have_image() {
  [ -n "$($DOCKER images -q "$1" 2>/dev/null)" ]
}

if [ "${1:-}" = "--reset" ]; then
  $DOCKER volume rm -f drivebase-build drivebase-install drivebase-log
  echo "Build volumes dropped. Next colcon build starts from scratch." >&2
  exit 0
fi

if ! $DOCKER info >/dev/null 2>&1; then
  cat >&2 <<'EOF'
Cannot reach the Docker daemon. On a systemd host:

    sudo systemctl enable --now docker
    sudo usermod -aG docker "$USER"     # then log out and back in

Or set DOCKER="sudo docker" to run this wrapper without the group.
EOF
  exit 1
fi

if ! have_image "$BASE_IMAGE"; then
  echo "Building $BASE_IMAGE (first run, this pulls ROS + Gazebo — several minutes)..." >&2
  $DOCKER build -t "$BASE_IMAGE" -f "$REPO/scripts/docker/Dockerfile.base" \
    "$REPO/scripts/docker" >&2
fi

if ! have_image "$IMAGE"; then
  echo "Building $IMAGE (one time)..." >&2
  $DOCKER build -q -t "$IMAGE" -f "$REPO/scripts/docker/Dockerfile" \
    "$REPO/scripts/docker" >&2
fi

DOCKER_ARGS=(
  --rm
  -v "$REPO:/ws"
  -v drivebase-build:/ws/build
  -v drivebase-install:/ws/install
  -v drivebase-log:/ws/log
  -v drivebase-home:/home/dev
  -w /ws
  # Host networking for DDS discovery, and host IPC so Fast DDS shared memory
  # and X's MIT-SHM both work instead of falling back to sockets.
  --net=host
  --ipc=host
  # Match the host user. Not cosmetic: Xwayland authorises local clients by
  # peer UID and there is no Xauthority file to forward, so a root container
  # is refused outright. See Dockerfile.base.
  --user "$(id -u):$(id -g)"
  # gz-transport discovers peers over UDP multicast (239.255.0.7). Under
  # --net=host on a machine with no multicast route it finds nothing, and the
  # failure is silent in the worst way: gz sim starts, the server and GUI
  # processes both live, no window ever opens, `create` loops "Requesting
  # list of world names" forever and /clock is never published — so every
  # downstream node just waits. Pinning discovery to loopback is safe because
  # every gz process shares the host network namespace.
  -e GZ_IP=127.0.0.1
)

# Rendering mode, decided once here so a forgotten export cannot silently turn
# into a Gazebo that starts and renders nothing.
if [ -n "${DRIVEBASE_HEADLESS:-}" ]; then
  MODE="headless (offscreen, software GL)"
  DOCKER_ARGS+=(
    -e LIBGL_ALWAYS_SOFTWARE=1
    -e GALLIUM_DRIVER=llvmpipe
    -e QT_QPA_PLATFORM=offscreen
  )
elif [ -z "${DISPLAY:-}" ]; then
  MODE="no DISPLAY set — offscreen, software GL"
  DOCKER_ARGS+=(
    -e LIBGL_ALWAYS_SOFTWARE=1
    -e GALLIUM_DRIVER=llvmpipe
    -e QT_QPA_PLATFORM=offscreen
  )
else
  # Qt is pinned to xcb rather than left to autodetect: the Wayland socket is
  # deliberately not mounted, and Qt picking wayland and failing produces a
  # plugin error that reads like a missing library.
  DOCKER_ARGS+=(
    -e "DISPLAY=$DISPLAY"
    -e QT_QPA_PLATFORM=xcb
    -e XDG_RUNTIME_DIR=/tmp/runtime-root
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
  # Hyprland's Xwayland runs without an auth file, so XAUTHORITY is usually
  # unset here and there is nothing to forward. Other setups do have one.
  if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
    DOCKER_ARGS+=(
      -e XAUTHORITY=/tmp/.docker.xauth
      -v "$XAUTHORITY:/tmp/.docker.xauth:ro"
    )
  fi

  if [ -n "${DRIVEBASE_SOFTWARE_GL:-}" ]; then
    MODE="windowed on $DISPLAY, software GL (forced)"
    DOCKER_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe)
  elif [ -e /dev/dri ]; then
    MODE="windowed on $DISPLAY, hardware GL via /dev/dri"
    DOCKER_ARGS+=(--device /dev/dri)
    # The render/video GIDs are read off the device nodes rather than
    # hardcoded — they differ between distros, and without membership the
    # non-root container falls back to llvmpipe with no error worth reading.
    for node in /dev/dri/renderD* /dev/dri/card*; do
      [ -e "$node" ] || continue
      DOCKER_ARGS+=(--group-add "$(stat -c '%g' "$node")")
    done
  else
    MODE="windowed on $DISPLAY, software GL (no /dev/dri on this host)"
    DOCKER_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe)
  fi
fi
echo "dev.sh: $MODE" >&2

# Ctrl-C has to reach the launch file, which needs a TTY — but only allocate
# one when there actually is one, or piping this into anything breaks. An
# `if` rather than `&&`: under `set -e` a false test here would exit.
if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_ARGS+=(-it)
fi

if [ $# -eq 0 ]; then
  exec $DOCKER run "${DOCKER_ARGS[@]}" "$IMAGE" \
    bash -lc 'source /opt/ros/jazzy/setup.bash
              [ -f install/setup.bash ] && source install/setup.bash
              exec bash'
fi

exec $DOCKER run "${DOCKER_ARGS[@]}" "$IMAGE" bash -lc '
  source /opt/ros/jazzy/setup.bash
  [ -f install/setup.bash ] && source install/setup.bash
  exec "$@"
' _ "$@"
