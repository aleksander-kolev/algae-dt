#!/usr/bin/env bash
# Headless sim_only integration smoke test (PLAN T1.1 exit check). Launches the full stack
# (gz + our arena + Nav2 + DT layer) headless, verifies the key topics flow with the right types,
# then tears down. Run inside algae-dt:dev with /ws + /ci mounted:
#   docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/sim_smoke.sh
# shellcheck disable=SC1091  # ROS/colcon setup files are generated at runtime; not followable here
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
WARM="${1:-60}"     # the software-rendered headless sim is slow (~1.4 Hz LiDAR, RTF~0.5); give the
echo "pid $LP; warming up ${WARM}s..."   # full stack (gz + Nav2 activate + DT layer) generous time
sleep "$WARM"

# Each runtime check is now a real ASSERTION that increments FAIL — not a bare echo. The script
# exits non-zero if any check fails, so `bash /ci/sim_smoke.sh` actually GATES a broken sim (it used
# to print empty sections and exit 0 regardless, giving a false PASS — VERIFICATION.md cites it as
# proof that Nav2 is active + cmd_vel is TwistStamped + the topics flow, so those are now enforced).
FAIL=0
fail() { echo "  [FAIL] $*"; FAIL=$((FAIL + 1)); }
ok()   { echo "  [ok]   $*"; }

echo "=== nodes ==="; ros2 node list 2>/dev/null | sort
# `ros2 node list` is eventually-consistent (DDS discovery), so retry briefly before failing — a
# node can publish (proven below) yet not appear in one enumeration snapshot.
for n in twin_mediator sync_supervisor mission_runner; do
  found=0
  for _ in 1 2 3 4 5; do
    if ros2 node list 2>/dev/null | grep -q "/$n"; then found=1; break; fi
    sleep 1
  done
  if [ "$found" = 1 ]; then ok "node $n up"; else fail "node $n MISSING from node list"; fi
done

echo "=== cmd_vel TYPES (must equal geometry_msgs/msg/TwistStamped) ==="
for t in /cmd_vel /dt/cmd_vel_raw; do
  ty=$(ros2 topic type "$t" 2>/dev/null)
  if [ "$ty" = "geometry_msgs/msg/TwistStamped" ]; then ok "$t -> $ty"
  else fail "$t type is '${ty:-MISSING}', expected geometry_msgs/msg/TwistStamped"; fi
done

echo "=== topic rates (must actually publish) ==="
for t in /scan /cmd_vel; do
  out=""
  for _ in 1 2 3; do                  # retry: discovery to a fresh subscriber can lag on a slow sim
    out=$(timeout 9 ros2 topic hz "$t" 2>&1 | grep -m1 "average rate")
    [ -n "$out" ] && break
  done
  if [ -n "$out" ]; then ok "$t ${out}"; else fail "$t: no messages within the window (topic dead)"; fi
done

echo "=== /dt/* state (must publish) ==="
for t in /dt/mode /dt/safety /dt/sim_pose /dt/health; do
  out=""
  for _ in 1 2 3 4; do
    out=$(timeout 6 ros2 topic echo --once "$t" 2>/dev/null)
    [ -n "$out" ] && break
  done
  if [ -n "$out" ]; then ok "$t published"; else fail "$t: no data"; fi
done

echo "=== AMCL localized (/amcl_pose in map frame) ==="
out=$(timeout 8 ros2 topic echo --once /amcl_pose 2>/dev/null | grep -m1 frame_id)
if echo "$out" | grep -q map; then ok "AMCL pose in map frame ($out)"
else fail "/amcl_pose not published in map frame (AMCL not localized)"; fi

echo "=== Nav2 lifecycle active ==="
# Primary: the navigation lifecycle manager's is_active. Fallback: bt_navigator lifecycle state.
isact=$(timeout 8 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null)
btstate=$(timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null)
if echo "$isact" | grep -qi 'success=*[Tt]rue' || echo "$btstate" | grep -qi active; then
  ok "Nav2 active (is_active='$(echo "$isact" | grep -i success | tr -d '\n')' bt_navigator='$btstate')"
else
  fail "Nav2 NOT active (is_active='$isact' bt_navigator='$btstate')"
fi

echo "=== teardown ==="
kill -INT "$LP" 2>/dev/null; sleep 3
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
echo "=== launch.log: errors/warnings ==="
grep -iE "error|exception|traceback|failed|not found" /tmp/launch.log | grep -ivE "INFO|deprecat" | head -25
echo "=== launch.log tail ==="; tail -15 /tmp/launch.log
if [ "$FAIL" -eq 0 ]; then
  echo "=== SMOKE PASS ==="
else
  echo "=== SMOKE FAILED ($FAIL check(s) failed) ==="
fi
exit "$FAIL"
