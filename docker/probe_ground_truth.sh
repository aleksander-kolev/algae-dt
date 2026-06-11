#!/usr/bin/env bash
# Diagnostic probe: what entity names does the Pose_V->TFMessage bridge emit on /sim/ground_truth?
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true headless:=true >/tmp/l.log 2>&1 &
LP=$!
sleep "${1:-80}"
echo '--- gz pose topics ---'
gz topic -l 2>/dev/null | grep -i pose
echo '--- one bridged message: frame ids ---'
out=""
for _ in 1 2 3; do
  out=$(timeout 8 ros2 topic echo --once /sim/ground_truth 2>/dev/null | grep -E 'child_frame_id|frame_id' | sort -u | head -30)
  [ -n "$out" ] && break
done
echo "$out"
echo '--- /sim/ground_truth hz ---'
timeout 8 ros2 topic hz /sim/ground_truth 2>&1 | grep -m1 'average rate'
echo '--- mediator + bridge log lines ---'
grep -iE 'twin_mediator|parameter_bridge' /tmp/l.log | grep -iE 'error|warn|ground' | head -10
kill -INT "$LP" 2>/dev/null; sleep 2
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null
true
