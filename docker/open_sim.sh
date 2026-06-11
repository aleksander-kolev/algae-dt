#!/usr/bin/env bash
# Open the sim_only digital-twin stack with GUI windows (operator console + RViz) for hands-on
# testing. Run inside the algae-dt:dev container with a WSLg/X display mounted, e.g. from WSL:
#   docker run -d --name algae_dt_sim --net=host -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/open_sim.sh
#
# gz_gui is OFF by default (the gz 3D client's large VBO crashes on the WSL d3d12/OGRE2 GL and, with
# on_exit_shutdown, would tear the whole launch down). The Gazebo SERVER still renders the LiDAR; you
# drive + visualise via the operator console and RViz. Pass extra launch args through, e.g.
#   bash /ci/open_sim.sh use_rviz:=false        # operator console only
# NOTE: no `set -u` — sourcing ROS/colcon setup references unbound vars (BEST_APPROACHES §Lessons).
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== colcon build --packages-select algae_dt ==="
colcon build --packages-select algae_dt || { echo "!!! COLCON BUILD FAILED"; exit 1; }
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo "=== ros2 launch algae_dt bringup.launch.py mode:=sim_only (gz_gui off, RViz on) ==="
exec ros2 launch algae_dt bringup.launch.py mode:=sim_only gz_gui:=false use_rviz:=true "$@"
