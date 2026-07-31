# One layer on top of the drivebase-dev:jazzy base image.
#
# urdfdom_py is what drivebase_behaviour/kinematics.py parses the arm chain
# with. Dockerfile.base now installs it too, so that one is currently a no-op.
#
# The ros2_control packages are for the SO-101 arm (so101_description):
# controller_manager + joint_trajectory_controller drive the mock-hardware
# path, gz-ros2-control the Gazebo one. joint_state_broadcaster is already in
# the base image. Note gz-ros2-control is the Jazzy/Harmonic pairing — it is
# also the stack STATUS.md names as fix candidate 4 for the rotation bug, so
# having it in the image is useful beyond the arm.
FROM drivebase-dev:jazzy

# The base image ends as UID 1000 (see Dockerfile.base — it is required for
# X to accept the connection). apt needs root, so step up and back down.
USER root

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-urdfdom-py \
      ros-jazzy-controller-manager \
      ros-jazzy-joint-trajectory-controller \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2controlcli \
 && rm -rf /var/lib/apt/lists/*

USER 1000:1000
