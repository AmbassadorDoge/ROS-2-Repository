#!/usr/bin/env bash
# Run a command inside the ROS 2 Jazzy dev container, with this repo mounted.
#
#   scripts/dev.sh colcon build --packages-select drivebase_behaviour
#   scripts/dev.sh python3 -m pytest src/drivebase_behaviour/test -v
#   scripts/dev.sh                      # interactive shell
#
# WHY THIS EXISTS. The development machine is an Apple Silicon Mac, which has
# no native ROS 2 or Gazebo. Everything runs in the container, and every
# verification step in docs/superpowers/plans/ assumes this wrapper.
#
# RENDERING. Docker on macOS has no GPU passthrough, so Gazebo's camera and
# gpu_lidar sensors fall back to llvmpipe software rendering. They DO work -
# measured, images arrive - but well under real time.
#
#   Trustworthy under software rendering: geometry, tf, kinematics, frame
#   layout, detection logic, state transitions.
#
#   NOT trustworthy: control-loop rates, Nav2 timing, anything phrased as Hz
#   or "how long did it take". docs/STATUS.md already records a Nav2 stall
#   that may be nothing more than a starved controller. Re-check every timing
#   figure on a real GPU before believing it.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="drivebase-dev:jazzy"
IMAGE="drivebase-dev:jazzy-pod"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    echo "Base image $BASE_IMAGE not found. Build it first." >&2
    exit 1
  fi
  echo "Building $IMAGE (one time)..." >&2
  docker build -q -t "$IMAGE" -f "$REPO/scripts/docker/Dockerfile" \
    "$REPO/scripts/docker" >&2
fi

# Software rendering is set here rather than per-command so a forgotten export
# cannot silently turn into a Gazebo that starts and renders nothing.
DOCKER_ARGS=(
  --rm
  -v "$REPO:/ws"
  -w /ws
  -e LIBGL_ALWAYS_SOFTWARE=1
  -e GALLIUM_DRIVER=llvmpipe
  -e QT_QPA_PLATFORM=offscreen
)

if [ $# -eq 0 ]; then
  exec docker run -it "${DOCKER_ARGS[@]}" "$IMAGE" \
    bash -lc 'source /opt/ros/jazzy/setup.bash
              [ -f install/setup.bash ] && source install/setup.bash
              exec bash'
fi

exec docker run "${DOCKER_ARGS[@]}" "$IMAGE" bash -lc '
  source /opt/ros/jazzy/setup.bash
  [ -f install/setup.bash ] && source install/setup.bash
  exec "$@"
' _ "$@"
