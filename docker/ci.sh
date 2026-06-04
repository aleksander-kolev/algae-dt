#!/usr/bin/env bash
# Canonical "definition of done" verification (RULES/HANDOFF): a CLEAN colcon build of ONLY our
# package + the full pytest suite, inside the course-faithful dev image. Headless — no display.
#
# From the host (repo root):
#   docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/ci.sh
# Pass extra pytest args through, e.g. `... bash /ci/ci.sh -k safety`.
# NOTE: no `set -u` — sourcing ROS/colcon setup references unbound vars (AMENT_TRACE_SETUP_FILES)
# and would abort the shell (BEST_APPROACHES §Lessons). pipefail is enough here.
# shellcheck disable=SC1091  # ROS/colcon setup files are generated at runtime; not followable here
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
echo "=== clean colcon build --symlink-install --packages-select algae_dt ==="
# HERMETIC: wipe build/install/log first and use --symlink-install (matching the lab build). A plain
# incremental build COPIES sources into install/ and never deletes files that disappear from src, so
# a renamed/deleted module kept resolving from the stale copy and CI stayed green on a tree that does
# not actually build from scratch. Clean + symlink-install eliminates that false-green class.
rm -rf build install log
colcon build --symlink-install --packages-select algae_dt || { echo "!!! COLCON BUILD FAILED"; exit 1; }
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo "=== pytest src/algae_dt/test ==="
python3 -m pytest src/algae_dt/test -q "$@"
