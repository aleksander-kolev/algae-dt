#!/usr/bin/env bash
# Quick liveness check for a running algae_dt container (exec this inside it).
set -o pipefail
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash >/dev/null 2>&1
echo "--- nodes ---"
ros2 node list 2>/dev/null | sort
echo "--- key topics ---"
for t in /dt/real_pose /dt/sim_pose /dt/sync_error /dt/safety /sim/ground_truth /scan; do
  if timeout 6 ros2 topic echo --once "$t" >/dev/null 2>&1; then
    echo "  OK   $t"
  else
    echo "  DEAD $t"
  fi
done
