#!/usr/bin/env bash
# =============================================================================
# lab_run.sh — ONE bulletproof command to build + run the algae-dt digital twin on the lab laptop,
# inside the course `turtlebot3_ws` Docker container, from a fresh `git clone`. Trusts NOTHING on the
# machine: if Docker is missing it installs rootless Docker; if no turtlebot3 image exists it builds
# one from scripts/Dockerfile; and it recreates the ~/turtlebot3_ws workspace you deleted.
#
# Usage (run from anywhere inside the cloned repo):
#   ./scripts/lab_run.sh                 # DEFAULT = both (FULL DEMO: real leads + sim mirror, robot #36)
#   ./scripts/lab_run.sh sim_only        # no robot (home / smoke-test)
#   ./scripts/lab_run.sh real_only       # real robot only
#   ./scripts/lab_run.sh both --rebuild  # clean colcon build
#   TB3_IMAGE=my_image ./scripts/lab_run.sh         # prefer a specific image name
#   ROS_DOMAIN_ID=42 ROBOT_IP=192.168.8.42 ./scripts/lab_run.sh   # different robot
#
# PROVENANCE (docs/RUN_ON_LAB_PC.md §Provenance):
#  - `docker run --net=host … -v <home>/turtlebot3_ws:/ws … turtlebot3_ws bash` + in-container
#    sourcing/build/launch are VERBATIM from Canvas "How to run Gazebo", "Changing Robot Inflation",
#    "DT Example", and a peer group's instruction.txt.
#  - rootless Docker install = the official https://get.docker.com/rootless script (Docker docs).
#  - the fallback image = scripts/Dockerfile (same apt stack as docker/Dockerfile / the course image).
#  - turtlebot3 is in the IMAGE, not the bare host (verified 5 ways — docs/DECISIONS.md D1).
#
# HONEST LIMITS (the script states these; it never guesses):
#  - rootless Docker needs the `uidmap` tools (root to install) + internet; and rootless `--net=host`
#    may NOT reach the robot over DDS (use rootful Docker for the real/both demo if so).
#  - building the fallback image needs internet (base pull + apt) and ~10–20 min.
#  - it does not prove our bringup.launch.py runs end-to-end (do the sim_only smoke-test once).
# =============================================================================
set -euo pipefail

log() { printf '[%s] %s\n' "$1" "${*:2}" >&2; }
die() { log FATAL "$*"; exit 3; }

# ---- args ----
MODE="${1:-both}"; shift || true     # default = both (full real+sim demo, ready to record)
HEADLESS=false; REBUILD=false
for a in "$@"; do case "$a" in
  --headless) HEADLESS=true ;;
  --rebuild)  REBUILD=true ;;
  *) die "unknown arg: $a" ;;
esac; done
case "$MODE" in sim_only|real_only|both) ;; *) die "mode must be sim_only|real_only|both (got '$MODE')";; esac

# ---- paths / config ----
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_SRC="$REPO/ros2_ws/src/algae_dt"
WS="$HOME/turtlebot3_ws"
CONTAINER="turtlebot3_container"
FALLBACK_IMAGE="algae-dt:fallback"
FALLBACK_DOCKERFILE="$REPO/scripts/Dockerfile"
# image preference order: course image -> our dev image -> built fallback
IMAGE_CANDIDATES=("${TB3_IMAGE:-turtlebot3_ws}" "algae-dt:dev" "$FALLBACK_IMAGE")
ROOTLESS=0
IMAGE=""

[ -d "$PKG_SRC" ] || die "package not found at $PKG_SRC — clone the repo and run this from inside it."

# ---- fallback 1: ensure Docker is usable (install rootless if absent) ----
ensure_docker() {
  if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then log OK "docker usable"; return; fi
  if command -v docker >/dev/null 2>&1; then
    log WARN "docker present but its daemon is unreachable — trying rootless Docker (no sudo)…"
  else
    log WARN "docker NOT installed — installing rootless Docker (no sudo)…"
  fi
  command -v newuidmap >/dev/null 2>&1 && command -v newgidmap >/dev/null 2>&1 \
    || die "rootless Docker needs the 'uidmap' tools (newuidmap/newgidmap), which require ROOT to install. Ask a TA to install 'uidmap' (and enable user namespaces), or provide Docker + the turtlebot3_ws image."
  command -v curl >/dev/null 2>&1 || die "need 'curl' + internet to install rootless Docker."
  curl -fsSL https://get.docker.com/rootless | sh || die "rootless Docker install failed (needs internet)."
  export PATH="$HOME/bin:$PATH"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  export DOCKER_HOST="unix://${XDG_RUNTIME_DIR}/docker.sock"
  if command -v systemctl >/dev/null 2>&1 && systemctl --user start docker 2>/dev/null; then
    log OK "rootless dockerd started via systemd --user"
  else
    "$HOME/bin/dockerd-rootless.sh" >/tmp/dockerd-rootless.log 2>&1 &
    log INFO "rootless dockerd starting (log: /tmp/dockerd-rootless.log)"
  fi
  for _ in $(seq 1 20); do docker info >/dev/null 2>&1 && break; sleep 1; done
  docker info >/dev/null 2>&1 || die "rootless dockerd did not come up (see /tmp/dockerd-rootless.log)."
  ROOTLESS=1
  log OK "rootless docker up"
  if [ "$MODE" != "sim_only" ]; then
    log WARN "rootless Docker + --net=host may NOT reach the robot over DDS. If real/both can't see /scan, use rootful Docker (TA-provided) for the hardware demo; sim_only works fine rootless."
  fi
}

# ---- fallback 2: ensure a turtlebot3 image exists (build from scripts/Dockerfile if none) ----
ensure_image() {
  for img in "${IMAGE_CANDIDATES[@]}"; do
    if docker image inspect "$img" >/dev/null 2>&1; then IMAGE="$img"; log OK "using image '$IMAGE'"; return; fi
  done
  log WARN "no turtlebot3 image found (${IMAGE_CANDIDATES[*]}). Building fallback '$FALLBACK_IMAGE' from scripts/Dockerfile — needs internet, ~10–20 min…"
  [ -f "$FALLBACK_DOCKERFILE" ] || die "fallback Dockerfile missing at $FALLBACK_DOCKERFILE."
  docker build -t "$FALLBACK_IMAGE" -f "$FALLBACK_DOCKERFILE" "$REPO/scripts" \
    || die "fallback image build failed (needs internet for the base image + apt)."
  IMAGE="$FALLBACK_IMAGE"; log OK "built + using '$IMAGE'"
}

echo "===================== PREFLIGHT ====================="
ensure_docker
ensure_image

# This team's robot: #36 @ 192.168.8.36, ROS_DOMAIN_ID=36 (override via env if it changes).
DOMAIN="${ROS_DOMAIN_ID:-36}"
ROBOT_IP="${ROBOT_IP:-192.168.8.36}"
log OK "mode=$MODE headless=$HEADLESS image=$IMAGE ROS_DOMAIN_ID=$DOMAIN robot=$ROBOT_IP rootless=$ROOTLESS"
if [ "$MODE" != "sim_only" ]; then
  if ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then log OK "robot $ROBOT_IP reachable";
  else log WARN "robot $ROBOT_IP not reachable yet — check Wi-Fi AP2IRR10 + that the Pi bringup is up"; fi
fi

# ---- recreate the workspace you deleted, copy the package in (course "How to create Packages" §3) ----
log INFO "(re)creating $WS/src/algae_dt"
mkdir -p "$WS/src"
rm -rf "$WS/src/algae_dt"
cp -r "$PKG_SRC" "$WS/src/algae_dt"
if [ "$REBUILD" = true ]; then rm -rf "$WS/build" "$WS/install" "$WS/log"; fi

# ---- robot bringup reminder for real/both ----
if [ "$MODE" != "sim_only" ]; then
  cat >&2 <<EOF

>>> BEFORE this runs, the robot must be up (separate terminal, on the robot Pi):
      ssh turtlebot@$ROBOT_IP
      export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=$DOMAIN
      ros2 launch turtlebot3_bringup robot.launch.py
    Laptop + robot on Wi-Fi AP2IRR10, SAME ROS_DOMAIN_ID=$DOMAIN.
    (In RViz, set "2D Pose Estimate" on the robot's real spot before starting a mission.)

EOF
fi

# ---- X11 + stale container ----
xhost +local: >/dev/null 2>&1 || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

# rootful uses --user $(id -u) (course command); rootless maps container-root to your user, so omit it
USER_ARG=(--user "$(id -u):$(id -g)")
[ "$ROOTLESS" = 1 ] && USER_ARG=()

log INFO "launching inside '$IMAGE' (Ctrl-C to stop)"
echo "   teleop in another terminal:  docker exec -it $CONTAINER bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null; source /ws/install/setup.bash; ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw'" >&2
echo >&2

exec docker run --rm -it --name "$CONTAINER" --net=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WS:/ws" -w /ws -e HOME=/ws \
  -e TURTLEBOT3_MODEL=burger -e ROS_DOMAIN_ID="$DOMAIN" \
  -e MODE="$MODE" -e HEADLESS="$HEADLESS" \
  "${USER_ARG[@]}" "$IMAGE" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    # the turtlebot3 stack is a pre-built overlay inside the course image; source if present
    [ -f /opt/turtlebot3_ws/install/setup.bash ] && source /opt/turtlebot3_ws/install/setup.bash || true
    if ! ros2 pkg prefix turtlebot3_gazebo >/dev/null 2>&1; then
      echo "FATAL: turtlebot3 stack not resolvable inside the image (checked /opt/ros/jazzy and"
      echo "       /opt/turtlebot3_ws/install). This image lacks the stock turtlebot3 packages."; exit 4
    fi
    echo "-- colcon build --packages-select algae_dt --"
    colcon build --packages-select algae_dt
    source install/setup.bash
    echo "-- ros2 launch algae_dt bringup.launch.py mode:=$MODE headless:=$HEADLESS --"
    exec ros2 launch algae_dt bringup.launch.py mode:="$MODE" headless:="$HEADLESS"
  '
