#!/usr/bin/env bash
# Live navigate-and-spray check: launch sim_only headless, run the mission probe, tear down.
#   docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/mission_smoke.sh
# shellcheck disable=SC1091  # ROS/colcon setup files are generated at runtime; not followable here
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
colcon build --packages-select algae_dt >/tmp/build.log 2>&1 || { echo BUILD FAILED; tail -20 /tmp/build.log; exit 1; }
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch algae_dt bringup.launch.py mode:=sim_only headless:=true >/tmp/launch.log 2>&1 &
LP=$!
echo "launched pid $LP; warming up 40s (Nav2 activate + localize)..."
sleep 40

echo "=== running mission probe (up to 150s) ==="
python3 /ci/mission_probe.py 150; PROBE_RC=$?   # the probe now sys.exit()s 0=PASS / 1=INCOMPLETE

echo "=== teardown ==="
kill -INT "$LP" 2>/dev/null; sleep 3
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
echo "=== mission_runner / collision_monitor log lines ==="
grep -iE "mission_runner|navigating|spray|collision_monitor|nav2 not active" /tmp/launch.log | tail -15
if [ "$PROBE_RC" = 0 ]; then
  echo "=== MISSION SMOKE PASS ==="
else
  echo "=== MISSION SMOKE FAILED (probe rc=$PROBE_RC: robot did not navigate+spray+treat) ==="
fi
exit "$PROBE_RC"     # propagate the real result so `bash /ci/mission_smoke.sh` actually gates
