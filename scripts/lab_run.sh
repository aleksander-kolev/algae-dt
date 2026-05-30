#!/usr/bin/env bash
# =============================================================================
# lab_run.sh — ONE bulletproof command to build + run the algae-dt digital twin
# on the lab laptop, inside the course `turtlebot3_ws` Docker container, from a
# fresh `git clone`. Recreates the ~/turtlebot3_ws workspace (which was deleted).
#
# Usage (run it from anywhere inside the cloned repo):
#   ./scripts/lab_run.sh sim_only                       # no robot (home / smoke-test)
#   ROS_DOMAIN_ID=36 ./scripts/lab_run.sh real_only     # real robot only
#   ROS_DOMAIN_ID=36 ./scripts/lab_run.sh both           # real leads, sim mirrors
#   ./scripts/lab_run.sh sim_only --headless --rebuild   # gz server only / clean build
#   TB3_IMAGE=my_image ./scripts/lab_run.sh real_only    # if the course image has another name
#
# WHERE EACH STEP COMES FROM (see docs/RUN_ON_LAB_PC.md §Provenance for the exact citation):
#  - the `docker run --net=host … -v <home>/turtlebot3_ws:/ws … turtlebot3_ws bash` invocation
#    is VERBATIM from the Canvas pages "How to run Gazebo & Modify the 3d World",
#    "Changing Robot Inflation", and "DT Example With Only Gazebo".
#  - `source /opt/ros/jazzy/setup.bash` + `source /opt/turtlebot3_ws/install/setup.bash` +
#    `cd /ws && colcon build && source install/setup.bash` is from those same DT examples and a
#    peer group's instruction.txt (the turtlebot3 stack is a pre-built overlay INSIDE the image).
#  - the custom-package layout `~/turtlebot3_ws/src/<pkg>` + `colcon build --packages-select` is
#    from Canvas "How to create Packages" / "Turtlebot3 Workspace Setup & Package Management".
#  - the bare lab host has NO native turtlebot3 (verified 5 ways — docs/DECISIONS.md D1); the stack
#    is in the image, which is why everything below runs in the container.
#
# WHAT THIS SCRIPT CANNOT GUARANTEE (it FAILS LOUDLY instead of guessing):
#  - that Docker + the `turtlebot3_ws` image actually exist on this laptop (checked in preflight).
#  - that our bringup.launch.py runs end-to-end (still being hardened — see docs/RUN_ON_LAB_PC.md §0).
# =============================================================================
set -euo pipefail

# ---- args ----
MODE="${1:-real_only}"; shift || true
HEADLESS=false; REBUILD=false
for a in "$@"; do
  case "$a" in
    --headless) HEADLESS=true ;;
    --rebuild)  REBUILD=true ;;
    *) echo "unknown arg: $a"; exit 2 ;;
  esac
done
case "$MODE" in sim_only|real_only|both) ;; *)
  echo "FATAL: mode must be sim_only | real_only | both (got '$MODE')"; exit 2 ;;
esac

# ---- paths / config ----
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_SRC="$REPO/ros2_ws/src/algae_dt"
IMAGE="${TB3_IMAGE:-turtlebot3_ws}"
WS="$HOME/turtlebot3_ws"
CONTAINER="turtlebot3_container"

[ -d "$PKG_SRC" ] || { echo "FATAL: package not found at $PKG_SRC — clone the repo and run this from inside it."; exit 1; }

echo "===================== PREFLIGHT ====================="
# Docker present?
if ! command -v docker >/dev/null 2>&1; then
  cat >&2 <<'EOF'
FATAL: `docker` is not on this laptop. The course environment is the `turtlebot3_ws` Docker image;
the bare lab host has NO native turtlebot3 (verified). Without Docker you cannot run the stock stack.
-> Confirm with a TA whether Docker is available, or install rootless Docker (runs as your user, no
   sudo). As a fallback ONLY if the host truly has turtlebot3 natively: `ros2 pkg list | grep turtlebot3`.
EOF
  exit 3
fi
# Image present?
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "FATAL: Docker image '$IMAGE' not found on this laptop. Images available:" >&2
  docker images --format '  {{.Repository}}:{{.Tag}}' >&2 || true
  echo "-> If the course image has a different name, re-run with TB3_IMAGE=<name>; else ask a TA." >&2
  exit 3
fi
# real/both need the robot's ROS_DOMAIN_ID (= robot number)
if [ "$MODE" != "sim_only" ] && [ -z "${ROS_DOMAIN_ID:-}" ]; then
  echo "FATAL: $MODE needs the robot's domain. Run:  ROS_DOMAIN_ID=<robot#> $0 $MODE" >&2; exit 2
fi
DOMAIN="${ROS_DOMAIN_ID:-0}"
echo "OK: docker + image '$IMAGE' present. mode=$MODE headless=$HEADLESS ROS_DOMAIN_ID=$DOMAIN"

# ---- recreate the workspace you deleted, copy the package in (course "How to create Packages" §3) ----
echo "== (re)create $WS/src/algae_dt =="
mkdir -p "$WS/src"
rm -rf "$WS/src/algae_dt"
cp -r "$PKG_SRC" "$WS/src/algae_dt"
if [ "$REBUILD" = true ]; then rm -rf "$WS/build" "$WS/install" "$WS/log"; fi

# ---- robot reminder for real/both ----
if [ "$MODE" != "sim_only" ]; then
  cat <<EOF

>>> BEFORE this runs, the robot must be up (separate terminal, on the robot Pi):
      ssh turtlebot@<robot-ip>
      export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=$DOMAIN
      ros2 launch turtlebot3_bringup robot.launch.py
    Laptop + robot on Wi-Fi AP2IRR10, SAME ROS_DOMAIN_ID=$DOMAIN.
    (In RViz, set "2D Pose Estimate" on the robot's real spot before starting a mission.)

EOF
fi

# ---- X11 for Gazebo/RViz/GUI (best effort; harmless if headless / no X) ----
xhost +local: >/dev/null 2>&1 || true
# remove any stale container of the same name
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "== launching inside '$IMAGE' (Ctrl-C to stop) =="
echo "   teleop in another terminal:  docker exec -it $CONTAINER bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null; source /ws/install/setup.bash; ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw'"
echo

exec docker run --rm -it --name "$CONTAINER" --net=host \
  -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WS:/ws" -w /ws -e HOME=/ws \
  -e TURTLEBOT3_MODEL=burger -e ROS_DOMAIN_ID="$DOMAIN" \
  -e MODE="$MODE" -e HEADLESS="$HEADLESS" \
  --user "$(id -u):$(id -g)" "$IMAGE" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    # the turtlebot3 stack is a pre-built overlay inside the image (peer instruction.txt); source if present
    [ -f /opt/turtlebot3_ws/install/setup.bash ] && source /opt/turtlebot3_ws/install/setup.bash || true
    if ! ros2 pkg prefix turtlebot3_gazebo >/dev/null 2>&1; then
      echo "FATAL: turtlebot3 stack not resolvable inside the image (checked /opt/ros/jazzy and"
      echo "       /opt/turtlebot3_ws/install). This is not the course turtlebot3_ws image."; exit 4
    fi
    echo "-- colcon build --packages-select algae_dt --"
    colcon build --packages-select algae_dt
    source install/setup.bash
    echo "-- ros2 launch algae_dt bringup.launch.py mode:=$MODE headless:=$HEADLESS --"
    exec ros2 launch algae_dt bringup.launch.py mode:="$MODE" headless:="$HEADLESS"
  '
