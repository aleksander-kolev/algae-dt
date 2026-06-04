#!/usr/bin/env bash
# =============================================================================
# lab_run.sh — THE lab-laptop runner. By DEFAULT it brings up the FULL digital-twin demo
# (mode:=both — real Burger leads + Gazebo sim mirror) inside the course Docker container, from a
# fresh `git clone`, and ERRORS rather than start a half-demo if `both` cannot run. If the robot is
# unavailable, pass `--sim` for the hardware-free sim_only fallback (a valid graded demo) instead.
#
# Usage (run from anywhere inside the cloned repo):
#   ./scripts/lab_run.sh             # full real+sim demo (robot #36 @ 192.168.8.36, domain 36)
#   ./scripts/lab_run.sh --sim       # hardware-free sim_only fallback demo (no robot/DDS needed)
#   ./scripts/lab_run.sh --rebuild   # clean colcon build first
#   ./scripts/lab_run.sh --no-gz-gui # skip the gz 3D client (weak GL stack; RViz+console still show all)
#   TB3_IMAGE=name / ROS_DOMAIN_ID=n / ROBOT_IP=ip   # overrides if image/robot differ
#
# RViz opens AUTOMATICALLY in both/real_only (the launch use_rviz default is mode-aware) — it is
# REQUIRED there: the "2D Pose Estimate" click that seeds AMCL has no other UI. The --sim fallback
# also opens RViz and skips the crash-prone gz 3D client (mirrors docker/open_sim.sh).
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
MODE="both"        # default: the lab runner brings up the FULL real+sim demo
GZ_GUI=true
for a in "$@"; do case "$a" in
  --rebuild) REBUILD=true ;;
  --sim|--fallback) MODE="sim_only" ;;
  --no-gz-gui) GZ_GUI=false ;;
  *) die "unknown arg: $a   (use --sim for the hardware-free fallback demo, --rebuild for a clean build, --no-gz-gui to skip the gz 3D client)" ;;
esac; done

# Launch args per mode. both/real_only: use_rviz auto-resolves to true (AMCL seeding needs it).
# --sim: mirror docker/open_sim.sh — RViz on, gz 3D client off (its large OGRE2 VBO crashes weak
# GL stacks; the gz SERVER still renders the LiDAR and the operator console + RViz show everything).
if [ "$MODE" = sim_only ]; then
  LAUNCH_ARGS="use_rviz:=true gz_gui:=false"
else
  LAUNCH_ARGS="gz_gui:=$GZ_GUI"
fi

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

# ---- record exactly which commit we are about to demo (the laptop is wiped / a USB tree can be stale) ----
GIT_REF="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo '?')"
GIT_DIRTY=""; git -C "$REPO" diff --quiet 2>/dev/null || GIT_DIRTY=" (DIRTY working tree — uncommitted changes)"
log INFO "demoing algae_dt @ commit ${GIT_REF}${GIT_DIRTY}  (from $REPO)"

echo "===================== PREFLIGHT (mode=$MODE; errors if it can't run) =====================" >&2

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

if [ "$MODE" = both ]; then
  require_rootful_docker
else
  command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1 \
    || die "no usable Docker. Ask a TA to enable Docker / add you to the 'docker' group."
  log OK "Docker usable (sim_only fallback — rootless is fine; no robot/DDS needed)"
fi
ensure_image
log OK "image=$IMAGE  ROS_DOMAIN_ID=$DOMAIN  mode=$MODE"

# ---- recreate the workspace (you deleted it) + copy the package in (course "How to create Packages" §3) ----
log INFO "(re)creating $WS/src/algae_dt"
mkdir -p "$WS/src"; rm -rf "$WS/src/algae_dt"; cp -r "$PKG_SRC" "$WS/src/algae_dt"
[ "$REBUILD" = true ] && rm -rf "$WS/build" "$WS/install" "$WS/log"

if [ "$MODE" = both ]; then
  # ---- the robot must be up FIRST (both = real robot). Print the reminder, then require reachability. ----
  cat >&2 <<EOF

>>> The robot must already be up (separate terminal, on the robot Pi) BEFORE this:
      ssh turtlebot@$ROBOT_IP
      export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=$DOMAIN
      ros2 launch turtlebot3_bringup robot.launch.py
    Laptop + robot on Wi-Fi AP2IRR10, SAME ROS_DOMAIN_ID=$DOMAIN.
    In RViz set "2D Pose Estimate" on the robot's real spot before starting a mission.

EOF
  # ---- HARD REQUIREMENT 3a: robot reachable (advisory fast-fail; the authoritative gate is /scan inside) ----
  ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1 \
    || die "robot $ROBOT_IP not reachable — 'both' needs the real robot. Power it on, start the Pi bringup, join Wi-Fi AP2IRR10, then re-run.
         For the hardware-free fallback demo instead (a valid graded sim_only demo):  ./scripts/lab_run.sh --sim"
  log OK "robot $ROBOT_IP reachable"
else
  log INFO "sim_only fallback: skipping the robot reachability check (no hardware needed)."
fi

# ---- X11 (a native lab Z-Book uses Xorg cookie auth, unlike home WSLg) + stale container ----
xhost +local: >/dev/null 2>&1 || true
X11_ARGS=(-e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
if [ -f "$XAUTH" ]; then
  X11_ARGS+=(-e XAUTHORITY="$XAUTH" -v "$XAUTH:$XAUTH:ro")     # pass the cookie so the non-root container user can open the display
  log OK "passing X authority cookie ($XAUTH) for the GUI on a native display"
else
  log WARN "no X cookie at $XAUTH; if the GUI won't open on a native display, run:  xhost +SI:localuser:\$(id -un)"
fi
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

log INFO "launching FULL DEMO (mode:=both) inside '$IMAGE' (Ctrl-C to stop)"
echo "   teleop in another terminal:  docker exec -it $CONTAINER bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null; source /ws/install/setup.bash; ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw'" >&2
echo >&2

# ---- run: rootful uses the course --user $(id -u). HARD REQUIREMENT 3b: /scan visible inside, else FAIL. ----
exec docker run --rm -it --name "$CONTAINER" --net=host \
  "${X11_ARGS[@]}" \
  -v "$WS:/ws" -w /ws -e HOME=/ws \
  -e TURTLEBOT3_MODEL=burger -e ROS_DOMAIN_ID="$DOMAIN" -e DT_MODE="$MODE" -e DT_LAUNCH_ARGS="$LAUNCH_ARGS" \
  --user "$(id -u):$(id -g)" "$IMAGE" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    [ -f /opt/turtlebot3_ws/install/setup.bash ] && source /opt/turtlebot3_ws/install/setup.bash || true
    if ! ros2 pkg prefix turtlebot3_gazebo >/dev/null 2>&1; then
      echo "FATAL: turtlebot3 stack not resolvable inside the image (checked /opt/ros/jazzy and"
      echo "       /opt/turtlebot3_ws/install). This image lacks the stock turtlebot3 packages."; exit 4
    fi
    if [ "$DT_MODE" = both ]; then
      echo "-- verifying the real-robot link: waiting up to 40s for /scan from inside the container --"
      ok=0; for _ in $(seq 1 40); do ros2 topic list 2>/dev/null | grep -qx /scan && { ok=1; break; }; sleep 1; done
      if [ "$ok" != 1 ]; then
        echo "FATAL: robot /scan NOT visible from inside the container — Pi bringup not up, wrong"
        echo "       ROS_DOMAIN_ID, not on Wi-Fi AP2IRR10, or Docker networking cannot reach the robot."
        echo "       both cannot run. Fix and re-run (or use --sim); never record a demo on a dead link."; exit 5
      fi
      echo "-- real link OK (/scan visible). --"
    fi
    echo "-- colcon build --packages-select algae_dt --"
    colcon build --packages-select algae_dt
    source install/setup.bash
    echo "-- ros2 launch algae_dt bringup.launch.py mode:=$DT_MODE $DT_LAUNCH_ARGS --"
    # shellcheck disable=SC2086  # DT_LAUNCH_ARGS is a deliberate word-split list of launch args
    exec ros2 launch algae_dt bringup.launch.py mode:="$DT_MODE" $DT_LAUNCH_ARGS
  '
