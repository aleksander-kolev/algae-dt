#!/usr/bin/env bash
# Headless `both`-mode integration smoke test (hardware-free: use_fake_robot). Verifies the parts
# of the twin only a LIVE Gazebo can prove — the /sim/* mirror bridge, the GROUND-TRUTH sim pose
# path (gz dynamic_pose/info -> /sim/ground_truth -> /dt/sim_pose), and a REAL resync round-trip:
# /dt/resync_cmd -> twin_resync policy -> gz set_pose against the running server -> /dt/resync_event.
# Run inside algae-dt:dev with /ws + /ci mounted:
#   docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/both_smoke.sh
# shellcheck disable=SC1091  # ROS/colcon setup files are generated at runtime; not followable here
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== build ==="
colcon build --packages-select algae_dt >/tmp/build.log 2>&1 || { echo "BUILD FAILED"; tail -20 /tmp/build.log; exit 1; }
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== launch both (use_fake_robot, headless) ==="
ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true headless:=true \
  >/tmp/launch_both.log 2>&1 &
LP=$!
WARM="${1:-60}"     # gz mirror + Nav2 + DT layer on a software-rendered headless sim: be generous
echo "pid $LP; warming up ${WARM}s..."
sleep "$WARM"

FAIL=0
fail() { echo "  [FAIL] $*"; FAIL=$((FAIL + 1)); }
ok()   { echo "  [ok]   $*"; }

echo "=== nodes ==="; ros2 node list 2>/dev/null | sort
for n in twin_mediator sync_supervisor mission_runner twin_resync fake_robot; do
  found=0
  for _ in 1 2 3 4 5; do
    if ros2 node list 2>/dev/null | grep -q "/$n"; then found=1; break; fi
    sleep 1
  done
  if [ "$found" = 1 ]; then ok "node $n up"; else fail "node $n MISSING from node list"; fi
done

echo "=== topic-collision rule: the mirror lives on /sim/* ==="
ty=$(ros2 topic type /sim/ground_truth 2>/dev/null)
if [ "$ty" = "tf2_msgs/msg/TFMessage" ]; then ok "/sim/ground_truth -> $ty"
else fail "/sim/ground_truth type is '${ty:-MISSING}', expected tf2_msgs/msg/TFMessage (bridge entry)"; fi

echo "=== ground-truth pose path (gz dynamic_pose/info -> bridge -> mediator) ==="
# /sim/ground_truth must FLOW (the scene broadcaster + bridge work), and /dt/sim_pose must be
# derived from it (the mediator's both-mode source). /dt/real_pose comes from the fake robot.
for t in /sim/ground_truth /dt/sim_pose /dt/real_pose /dt/sync_error; do
  out=""
  for _ in 1 2 3 4; do
    out=$(timeout 6 ros2 topic echo --once "$t" 2>/dev/null)
    [ -n "$out" ] && break
  done
  if [ -n "$out" ]; then ok "$t published"; else fail "$t: no data"; fi
done

echo "=== resync round-trip against the LIVE gz server ==="
# Subscribe to the event FIRST (background), then click: a one-shot event must arrive, proving
# /dt/resync_cmd -> policy -> gz set_pose (real service on the running world) -> /dt/resync_event.
EVT=/tmp/resync_event.txt; : >"$EVT"
( timeout 25 ros2 topic echo --once /dt/resync_event >"$EVT" 2>/dev/null ) &
SUB=$!
sleep 3                                   # let the subscriber discover the publisher
ros2 topic pub --once /dt/resync_cmd std_msgs/msg/Empty "{}" >/dev/null 2>&1
wait "$SUB"
if grep -q "snapped" "$EVT"; then
  ok "resync executed: $(grep -m1 data "$EVT" | head -c 120)"
else
  fail "no /dt/resync_event after a RESYNC click (gz set_pose path broken?)"
  echo "  --- twin_resync log lines ---"
  grep -i resync /tmp/launch_both.log | tail -5
fi

echo "=== teardown ==="
kill -INT "$LP" 2>/dev/null; sleep 3
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
echo "=== launch_both.log: errors/warnings ==="
grep -iE "error|exception|traceback|failed|not found" /tmp/launch_both.log | grep -ivE "INFO|deprecat" | head -25
echo "=== launch_both.log tail ==="; tail -15 /tmp/launch_both.log
if [ "$FAIL" -eq 0 ]; then
  echo "=== BOTH SMOKE PASS ==="
else
  echo "=== BOTH SMOKE FAILED ($FAIL check(s) failed) ==="
fi
exit "$FAIL"
