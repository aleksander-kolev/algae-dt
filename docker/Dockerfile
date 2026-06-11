# Course-faithful development image for algae-dt (TU/e 2IRR10).
#
# This is our reproducible equivalent of the course's provided `turtlebot3_ws` Docker image
# (docs/SETUP.md §0/§1): ROS 2 Jazzy + the FULL stock turtlebot3 stack + Nav2 +
# Gazebo Harmonic (via ros_gz) + PyQt5 + pytest/colcon. We build ON TOP of the stock packages
# exactly as the course mandates (DECISIONS.md D2) — nothing here re-implements them.
#
# The workspace is NOT copied in; it is bind-mounted at /ws at run time (matches the course
# `-v ~/turtlebot3_ws:/ws`), so edits on the host are live in the container. Build context is
# this `docker/` dir only (no COPY) — the build stays fast and independent of the source tree.
#
#   Build:  docker build -t algae-dt:dev docker          (or docker/build.ps1)
#   Run:    see docker/run.ps1 / docs/DOCKER.md
FROM ros:jazzy

LABEL org.opencontainers.image.title="algae-dt-dev"
LABEL org.opencontainers.image.description="ROS 2 Jazzy + stock turtlebot3 stack + Nav2 + Gazebo Harmonic for the TU/e 2IRR10 algae-dt digital twin"

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger \
    LDS_MODEL=LDS-02 \
    ROS_DOMAIN_ID=36

SHELL ["/bin/bash", "-lc"]

# The stock turtlebot3 stack (model/spawn/RSP/Nav2/teleop/SLAM) + Nav2 + Gazebo Harmonic bridge +
# the GUI + Python test tooling. Recommends are kept ON so rviz2 / gz-sim GUI / PyQt5 are fully
# functional for an actual rendered demo (WSLg), not just headless verification.
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
      python3-pytest \
      python3-pytest-cov \
    && rm -rf /var/lib/apt/lists/*

# Workspace mount point (course convention `-v ~/turtlebot3_ws:/ws`).
WORKDIR /ws

# Every interactive shell sources ROS, the workspace overlay (once built), and the TB3 env.
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      'export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02' \
      >> /etc/bash.bashrc

CMD ["bash"]
