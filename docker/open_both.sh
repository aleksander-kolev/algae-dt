#!/usr/bin/env bash
# Open the FULL hardware-free digital twin with GUI windows for hands-on testing:
#   mode:=both use_fake_robot:=true  ->  fake real robot (bare topics) + Gazebo mirror (/sim/*)
#   + Nav2 + mediator/supervisor/mission/twin_resync + operator console + RViz.
# The `both` sibling of open_sim.sh. Run inside algae-dt:dev with an X display mounted.
#   From WSL:                -v /tmp/.X11-unix:/tmp/.X11-unix
#   From Windows PowerShell: -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix
# (Docker Desktop exposes the host's WSLg under /run/desktop/mnt/host/wslg — verified live.)
#
# gz_gui is OFF by default (the gz 3D client's large VBO crashes on the WSL d3d12/OGRE2 GL); the
# Gazebo SERVER still renders the LiDAR and you watch both robots in the operator console + RViz.
# Pass extra launch args through, e.g.:  bash /ci/open_both.sh use_dynamic_obstacle:=true
# NOTE: no `set -u` — sourcing ROS/colcon setup references unbound vars (BEST_APPROACHES §Lessons).
set -o pipefail
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/xdg-runtime}
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1          # no GPU device in the container -> Mesa llvmpipe
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== colcon build --packages-select algae_dt ==="
colcon build --packages-select algae_dt || { echo "!!! COLCON BUILD FAILED"; exit 1; }
source install/setup.bash
echo "=== ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true (gz_gui off, RViz on) ==="
echo "    1) RViz '2D Pose Estimate' on the fake robot's spot (the twin auto-resyncs after it)"
echo "    2) operator console: place blooms -> Start | E-STOP | RESYNC TWIN"
exec ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true gz_gui:=false "$@"
