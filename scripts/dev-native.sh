#!/usr/bin/env bash
# Run the pure-logic parts of this repo without ROS 2 installed.
#
#   scripts/dev-native.sh python -m pytest src/drivebase_behaviour/test -v
#   scripts/dev-native.sh xacro src/drivebase_description/urdf/drivebase.urdf.xacro use_sim:=true
#
# WHY THIS EXISTS, alongside dev.sh. dev.sh runs everything in the ROS 2 Jazzy
# container and is still the only way to run Gazebo, rclpy nodes, or colcon.
# But Phase 2 of the behaviour coordinator (drivebase_msgs aside) is pure
# geometry: numpy, urdf_parser_py and xacro, with no rclpy import anywhere. On
# a machine with no Docker and no ROS - which is where this was picked back up
# - that work is otherwise blocked on a toolchain it does not use.
#
# WHAT IT DOES NOT COVER. Anything importing rclpy, any launch file, any
# colcon invocation, and all of Gazebo. Use dev.sh for those. If a test starts
# failing here with ModuleNotFoundError on a ROS package, that is the boundary,
# not a bug.
#
# THE AMENT INDEX SHIM. xacro resolves `$(find drivebase_description)` through
# ament_index_python, which reads AMENT_PREFIX_PATH and expects an install
# tree. Rather than run colcon, this builds the two directories that lookup
# actually touches - a marker under resource_index/packages, and share/<pkg>
# symlinked at the source - which is enough for `$(find ...)` and nothing else.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${DRIVEBASE_VENV:-$HOME/.venvs/drivebase-dev}"
PREFIX="${DRIVEBASE_AMENT_PREFIX:-$HOME/.cache/drivebase-dev/ament}"

# Both live outside the repo on purpose: the repo sits on an NTFS/fuseblk
# mount that does not preserve the executable bit, so a venv created inside it
# produces console scripts that cannot be run.
if [ ! -x "$VENV/bin/python" ]; then
  echo "Creating $VENV ..." >&2
  command -v uv >/dev/null || { echo "uv not found; install it first." >&2; exit 1; }
  uv venv --python 3.12 "$VENV" >&2
  # ament_index_python is not published on PyPI, so it comes from source. The
  # rest are ordinary packages; urdf_parser_py and xacro are pure Python and
  # identical to what the container's ros-jazzy-* debs install.
  UV_LINK_MODE=copy uv pip install --python "$VENV" \
    numpy pytest urdf-parser-py xacro \
    "git+https://github.com/ament/ament_index.git@jazzy#subdirectory=ament_index_python" >&2
fi

rm -rf "$PREFIX"
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
for pkg in "$REPO"/src/*/; do
  name="$(basename "$pkg")"
  touch "$PREFIX/share/ament_index/resource_index/packages/$name"
  ln -sfn "${pkg%/}" "$PREFIX/share/$name"
done

export AMENT_PREFIX_PATH="$PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PATH="$VENV/bin:$PATH"
export PYTHONPATH="$REPO/src/drivebase_behaviour${PYTHONPATH:+:$PYTHONPATH}"

cd "$REPO"
if [ $# -eq 0 ]; then
  exec bash
fi
exec "$@"
