#!/usr/bin/env bash
# Launch the full sim_only stack with a visible GUI (Gazebo + RViz + the operator console) for a
# live demo, forwarded to the Windows desktop via WSLg (X11). With software GL (no GPU in the
# container) Gazebo is slow, but the operator console + RViz stay responsive.
set -o pipefail
export XDG_RUNTIME_DIR=/tmp/xdg-runtime
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1          # no GPU device -> Mesa software GL (llvmpipe)
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== building algae_dt ==="
colcon build --packages-select algae_dt >/tmp/build.log 2>&1 || { echo "BUILD FAILED"; tail -25 /tmp/build.log; exit 1; }
source install/setup.bash
echo "=== launching sim_only: Gazebo GUI + RViz + operator console ==="
exec ros2 launch algae_dt bringup.launch.py mode:=sim_only use_rviz:=true
