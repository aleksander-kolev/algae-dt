# Self-contained fallback image for the lab laptop, built by scripts/lab_run.sh only when neither
# a turtlebot3_ws image nor algae-dt:dev is already present.
#
# Same contents as docker/Dockerfile: ROS 2 Jazzy + the full turtlebot3 stack + Nav2 +
# Gazebo Harmonic (ros_gz) + PyQt5 + colcon. No COPY — the workspace is bind-mounted at /ws at run
# time, so the build context is irrelevant and the build stays fast.
#
# Needs internet at build time (base image + apt). Built automatically by lab_run.sh as
# algae-dt:fallback, or manually:  docker build -t algae-dt:fallback -f scripts/Dockerfile scripts
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger \
    LDS_MODEL=LDS-02
SHELL ["/bin/bash", "-lc"]

# The turtlebot3 stack (model/spawn/RSP/Nav2/teleop/SLAM) + Nav2 + Gazebo Harmonic bridge + the
# GUI + Python tooling. Recommends ON so rviz2 / gz-sim GUI / PyQt5 actually render.
RUN apt-get update && apt-get install -y \
      ros-jazzy-turtlebot3 \
      ros-jazzy-turtlebot3-msgs \
      ros-jazzy-turtlebot3-simulations \
      ros-jazzy-turtlebot3-gazebo \
      ros-jazzy-turtlebot3-navigation2 \
      ros-jazzy-turtlebot3-bringup \
      ros-jazzy-turtlebot3-teleop \
      ros-jazzy-turtlebot3-cartographer \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-simple-commander \
      ros-jazzy-ros-gz \
      ros-jazzy-rviz2 \
      python3-colcon-common-extensions \
      python3-pyqt5 \
      iputils-ping \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# Source ROS + the workspace overlay (once built) in every shell. (Some images also expose the
# stack at /opt/turtlebot3_ws/install — lab_run.sh sources that too if present.)
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      'export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02' \
      >> /etc/bash.bashrc
CMD ["bash"]
