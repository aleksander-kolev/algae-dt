#!/usr/bin/env bash
# =============================================================================
# lab_run.sh — THE lab-laptop runner. It does ONE thing: bring up the FULL digital-twin demo
# (mode:=both — real Burger leads + Gazebo sim mirror) inside the course Docker container, from a
# fresh `git clone`. If `both` 100% cannot run on this machine, it ERRORS — it never starts a
# half-demo. (Home / sim-only testing is a different job: use `docker/run.sh` + `docker/sim_smoke.sh`.)
#
# Usage (run from anywhere inside the cloned repo):
#   ./scripts/lab_run.sh             # full real+sim demo (robot #36 @ 192.168.8.36, domain 36)
#   ./scripts/lab_run.sh --rebuild   # clean colcon build first
#   TB3_IMAGE=name / ROS_DOMAIN_ID=n / ROBOT_IP=ip   # overrides if image/robot differ
#
# WHAT IT REQUIRES (and ERRORS on if absent — `both` cannot exist without these):
#   1. ROOTFUL Docker. `both` talks to the robot over ROS 2 DDS, which needs real `--net=host`
#      (host network namespace). Rootless Docker's `--net=host` is RootlessKit/slirp4netns — NO LAN
#      multicast — so the robot is unreachable. No sudo can't install rootful → that's a TA blocker.
#   2. A turtlebot3 image (course `turtlebot3_ws` / our `algae-dt:dev`), else it BUILDS one from
#      scripts/Dockerfile (needs internet + ~10–20 min).
#   3. The real robot reachable + publishing `/scan` (verified from inside the container).
#
# PROVENANCE: docker run / sourcing / build / launch are verbatim from Canvas ("How to run Gazebo",
# "Changing Robot Inflation", "DT Example", "Connecting lab laptop to robot") — docs/RUN_ON_LAB_PC.md.
# =============================================================================
set -euo pipefail

log() { printf '[%s] %s\n' "$1" "${*:2}" >&2; }
die() { log FATAL "$*"; exit 3; }

# ---- args ----
REBUILD=false
for a in "$@"; do case "$a" in
  --rebuild) REBUILD=true ;;
  *) die "unknown arg: $a   (this script only runs the full 'both' demo; no mode arg)" ;;
esac; done

MODE="both"   # this is the lab runner — it ONLY does the full real+sim demo

# ---- paths / config ----
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_SRC="$REPO/ros2_ws/src/algae_dt"
WS="$HOME/turtlebot3_ws"
CONTAINER="turtlebot3_container"
FALLBACK_IMAGE="algae-dt:fallback"
FALLBACK_DOCKERFILE="$REPO/scripts/Dockerfile"
IMAGE_CANDIDATES=("${TB3_IMAGE:-turtlebot3_ws}" "algae-dt:dev" "$FALLBACK_IMAGE")
IMAGE=""
DOMAIN="${ROS_DOMAIN_ID:-36}"      # this team's robot #36
ROBOT_IP="${ROBOT_IP:-192.168.8.36}"

[ -d "$PKG_SRC" ] || die "package not found at $PKG_SRC — clone the repo and run this from inside it."

echo "===================== PREFLIGHT (both-only; errors if it can't run) =====================" >&2

# ---- HARD REQUIREMENT 1: rootful Docker (rootless can't reach the robot → can't do 'both') ----
require_rootful_docker() {
  command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1 \
    || die "no usable Docker. 'both' needs ROOTFUL Docker (real --net=host for the robot's DDS). You can't install rootful without sudo, and rootless can't reach the robot — so 'both' cannot run here. Ask a TA to enable Docker / add you to the 'docker' group."
  if docker info --format '{{.SecurityOptions}}' 2>/dev/null | grep -qi rootless; then
    die "this Docker is ROOTLESS. Rootless --net=host (RootlessKit/slirp4netns) has no LAN multicast, so the robot is unreachable over DDS and 'both' cannot run. Use ROOTFUL Docker for the lab demo (ask a TA)."
  fi
  log OK "rootful Docker usable"
}

# ---- HARD REQUIREMENT 2: a turtlebot3 image (build one if none; errors if the build can't) ----
ensure_image() {
  for img in "${IMAGE_CANDIDATES[@]}"; do
    if docker image inspect "$img" >/dev/null 2>&1; then IMAGE="$img"; log OK "using image '$IMAGE'"; return; fi
  done
  log WARN "no turtlebot3 image found (${IMAGE_CANDIDATES[*]}). Building '$FALLBACK_IMAGE' from scripts/Dockerfile — needs internet, ~10–20 min…"
  [ -f "$FALLBACK_DOCKERFILE" ] || die "fallback Dockerfile missing at $FALLBACK_DOCKERFILE."
  docker build -t "$FALLBACK_IMAGE" -f "$FALLBACK_DOCKERFILE" "$REPO/scripts" \
    || die "image build failed (needs internet for the base image + apt). 'both' cannot run without a turtlebot3 image."
  IMAGE="$FALLBACK_IMAGE"; log OK "built + using '$IMAGE'"
}

require_rootful_docker
ensure_image
log OK "image=$IMAGE  ROS_DOMAIN_ID=$DOMAIN  robot=$ROBOT_IP"

# ---- recreate the workspace (you deleted it) + copy the package in (course "How to create Packages" §3) ----
log INFO "(re)creating $WS/src/algae_dt"
mkdir -p "$WS/src"; rm -rf "$WS/src/algae_dt"; cp -r "$PKG_SRC" "$WS/src/algae_dt"
[ "$REBUILD" = true ] && rm -rf "$WS/build" "$WS/install" "$WS/log"

# ---- the robot must be up FIRST (both = real robot). Print the reminder, then require reachability. ----
cat >&2 <<EOF

>>> The robot must already be up (separate terminal, on the robot Pi) BEFORE this:
      ssh turtlebot@$ROBOT_IP
      export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=$DOMAIN
      ros2 launch turtlebot3_bringup robot.launch.py
    Laptop + robot on Wi-Fi AP2IRR10, SAME ROS_DOMAIN_ID=$DOMAIN.
    In RViz set "2D Pose Estimate" on the robot's real spot before starting a mission.

EOF
# ---- HARD REQUIREMENT 3a: robot reachable on the network ----
ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1 \
  || die "robot $ROBOT_IP not reachable — 'both' needs the real robot. Power it on, start the Pi bringup, join Wi-Fi AP2IRR10, then re-run."
log OK "robot $ROBOT_IP reachable"

# ---- X11 + stale container ----
xhost +local: >/dev/null 2>&1 || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

log INFO "launching FULL DEMO (mode:=both) inside '$IMAGE' (Ctrl-C to stop)"
echo "   teleop in another terminal:  docker exec -it $CONTAINER bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null; source /ws/install/setup.bash; ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw'" >&2
echo >&2

# ---- run: rootful uses the course --user $(id -u). HARD REQUIREMENT 3b: /scan visible inside, else FAIL. ----
exec docker run --rm -it --name "$CONTAINER" --net=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WS:/ws" -w /ws -e HOME=/ws \
  -e TURTLEBOT3_MODEL=burger -e ROS_DOMAIN_ID="$DOMAIN" \
  --user "$(id -u):$(id -g)" "$IMAGE" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    [ -f /opt/turtlebot3_ws/install/setup.bash ] && source /opt/turtlebot3_ws/install/setup.bash || true
    if ! ros2 pkg prefix turtlebot3_gazebo >/dev/null 2>&1; then
      echo "FATAL: turtlebot3 stack not resolvable inside the image (checked /opt/ros/jazzy and"
      echo "       /opt/turtlebot3_ws/install). This image lacks the stock turtlebot3 packages."; exit 4
    fi
    echo "-- verifying the real-robot link: waiting up to 40s for /scan from inside the container --"
    ok=0; for _ in $(seq 1 40); do ros2 topic list 2>/dev/null | grep -qx /scan && { ok=1; break; }; sleep 1; done
    if [ "$ok" != 1 ]; then
      echo "FATAL: robot /scan NOT visible from inside the container — Pi bringup not up, wrong"
      echo "       ROS_DOMAIN_ID, not on Wi-Fi AP2IRR10, or Docker networking cannot reach the robot."
      echo "       both cannot run. Fix and re-run; never record a demo on a dead link."; exit 5
    fi
    echo "-- real link OK (/scan visible). colcon build --packages-select algae_dt --"
    colcon build --packages-select algae_dt
    source install/setup.bash
    echo "-- ros2 launch algae_dt bringup.launch.py mode:=both --"
    exec ros2 launch algae_dt bringup.launch.py mode:=both
  '
