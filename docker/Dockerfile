# Development image for algae-dt: ROS 2 Jazzy + the full turtlebot3 stack + Nav2 +
# Gazebo Harmonic (via ros_gz) + PyQt5 + colcon. We build on top of the stock packages and
# don't re-implement any of them.
#
# The workspace isn't copied in; it's bind-mounted at /ws at run time (-v ros2_ws:/ws), so edits
# on the host are live in the container. The build context is just this docker/ dir (no COPY),
# which keeps the build fast and independent of the source tree.
#
#   Build:  docker build -t algae-dt:dev docker          (or docker/build.ps1)
#   Run:    docker/run.sh  /  docker/run.ps1
FROM ros:jazzy

LABEL org.opencontainers.image.title="algae-dt-dev"
LABEL org.opencontainers.image.description="ROS 2 Jazzy + stock turtlebot3 stack + Nav2 + Gazebo Harmonic for the TU/e 2IRR10 algae-dt digital twin"

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger \
    LDS_MODEL=LDS-02 \
    ROS_DOMAIN_ID=36

SHELL ["/bin/bash", "-lc"]

# The turtlebot3 stack (model/spawn/RSP/Nav2/teleop/SLAM) + Nav2 + Gazebo Harmonic bridge + the
# GUI + Python tooling. Recommends stay ON so rviz2 / gz-sim GUI / PyQt5 actually render (WSLg).
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
    && rm -rf /var/lib/apt/lists/*

# Workspace mount point (-v ros2_ws:/ws).
WORKDIR /ws

# Every interactive shell sources ROS, the workspace overlay (once built), and the TB3 env.
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      'export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02' \
      >> /etc/bash.bashrc

CMD ["bash"]
