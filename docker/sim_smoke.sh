#!/usr/bin/env bash
# Headless sim_only integration smoke test (PLAN T1.1 exit check). Launches the full stack
# (gz + our arena + Nav2 + DT layer) headless, verifies the key topics flow with the right types,
# then tears down. Run inside algae-dt:dev with /ws + /ci mounted:
#   docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/sim_smoke.sh
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== build ==="
colcon build --packages-select algae_dt >/tmp/build.log 2>&1 || { echo "BUILD FAILED"; tail -20 /tmp/build.log; exit 1; }
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== launch sim_only headless ==="
ros2 launch algae_dt bringup.launch.py mode:=sim_only headless:=true >/tmp/launch.log 2>&1 &
LP=$!
WARM="${1:-40}"
echo "pid $LP; warming up ${WARM}s..."
sleep "$WARM"

echo "=== nodes ==="; ros2 node list 2>/dev/null | sort
echo "=== /dt + sim topics ==="; ros2 topic list 2>/dev/null | grep -E "/scan|/odom|/cmd_vel|/dt/|/clock|/map$|/tf$" | sort
echo "=== cmd_vel TYPES (expect TwistStamped) ==="
for t in /cmd_vel /dt/cmd_vel_raw; do echo "  $t -> $(ros2 topic type $t 2>/dev/null || echo MISSING)"; done
echo "=== /scan rate (lidar render, 6s) ==="; timeout 7 ros2 topic hz /scan 2>&1 | grep -E "average rate|no new" | head -2
echo "=== /cmd_vel rate (mediator output) ==="; timeout 6 ros2 topic hz /cmd_vel 2>&1 | grep -E "average rate|no new" | head -2
echo "=== /dt/mode ==="; timeout 5 ros2 topic echo --once /dt/mode 2>&1 | head -2
echo "=== /dt/safety ==="; timeout 5 ros2 topic echo --once /dt/safety 2>&1 | head -2
echo "=== /dt/sim_pose (mirror) ==="; timeout 5 ros2 topic echo --once /dt/sim_pose 2>&1 | grep -E "x:|y:" | head -2
echo "=== /dt/health (synthetic battery) ==="; timeout 5 ros2 topic echo --once /dt/health 2>&1 | grep -E "voltage" | head -1
echo "=== AMCL localized? (/amcl_pose) ==="; timeout 6 ros2 topic echo --once /amcl_pose 2>&1 | grep -E "x:|frame_id" | head -3

echo "=== teardown ==="
kill -INT "$LP" 2>/dev/null; sleep 3
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
echo "=== launch.log: errors/warnings ==="
grep -iE "error|exception|traceback|failed|not found" /tmp/launch.log | grep -ivE "INFO|deprecat" | head -25
echo "=== launch.log tail ==="; tail -15 /tmp/launch.log
echo "=== SMOKE DONE ==="
