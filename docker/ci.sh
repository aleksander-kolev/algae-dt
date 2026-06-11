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
# DDS ISOLATION: confine every test node to THIS container's loopback. Without it, a concurrently
# running demo container on the same docker bridge (same L2 -> DDS multicast discovery) leaks its
# live /dt/* topics into the suite — e.g. the demo GUI's bloom markers appeared inside
# mission_runner tests and a live /cmd_vel stream broke the fake-robot silence-watchdog test
# (9 phantom failures, all green once isolated). Belt-and-braces: a non-default domain too.
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${CI_ROS_DOMAIN_ID:-77}"
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
