# One layer on top of the existing drivebase-dev:jazzy image.
#
# urdfdom_py is what drivebase_behaviour/kinematics.py parses the arm chain
# with. It is a declared exec_depend in the package manifest, but the base
# image predates that package, so it has to be added here rather than assumed.
FROM drivebase-dev:jazzy

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-jazzy-urdfdom-py \
 && rm -rf /var/lib/apt/lists/*
