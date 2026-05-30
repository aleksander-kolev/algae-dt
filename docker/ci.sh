#!/usr/bin/env bash
# Canonical "definition of done" verification (RULES/HANDOFF): a CLEAN colcon build of ONLY our
# package + the full pytest suite, inside the course-faithful dev image. Headless — no display.
#
# From the host (repo root):
#   docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/ci.sh
# Pass extra pytest args through, e.g. `... bash /ci/ci.sh -k safety`.
# NOTE: no `set -u` — sourcing ROS/colcon setup references unbound vars (AMENT_TRACE_SETUP_FILES)
# and would abort the shell (BEST_APPROACHES §Lessons). pipefail is enough here.
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== colcon build --packages-select algae_dt ==="
colcon build --packages-select algae_dt || { echo "!!! COLCON BUILD FAILED"; exit 1; }
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo "=== pytest src/algae_dt/test ==="
python3 -m pytest src/algae_dt/test -q "$@"
