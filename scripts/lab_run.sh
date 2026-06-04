#!/usr/bin/env bash
# =============================================================================
# lab_run.sh — THE lab-laptop runner. By DEFAULT it brings up the FULL digital-twin demo
# (mode:=both — real Burger leads + Gazebo sim mirror), from a fresh `git clone`, and ERRORS rather
# than start a half-demo if `both` cannot run. If the robot is unavailable, pass `--sim` for the
# hardware-free sim_only fallback (a valid graded demo) instead.
#
# TWO RUNTIMES (auto-detected; force with --docker / --native):
#   docker — the course `turtlebot3_ws` image (or algae-dt:dev, or a fallback it builds). `both`
#            needs ROOTFUL docker: the robot link is ROS 2 DDS over a real `--net=host`, and
#            rootless docker (RootlessKit/slirp4netns) has NO LAN multicast → robot unreachable.
#   native — NO Docker at all (the lab PC as found 2026-06): ROS 2 Jazzy at /opt/ros/jazzy with the
#            turtlebot3 stack resolvable natively — on the lab PC it is built FROM SOURCE in
#            ~/turtlebot3_ws. If that workspace is broken (copied from another PC: wrong CMake
#            cache paths, foreign-owned files), fix it FIRST with  ./scripts/lab_fix_workspace.sh
#            (no sudo; rebuilds it from the fail-safe zip in ~/Downloads).
#   Selection: an explicit flag wins; else docker when usable (rootful when mode=both), else
#   native, else ERROR explaining BOTH paths. Native DDS reaches the robot fine (no docker needed).
#
# Usage (run from anywhere inside the cloned repo):
#   ./scripts/lab_run.sh             # full real+sim demo (robot #36 @ 192.168.8.36, domain 36)
#   ./scripts/lab_run.sh --sim       # hardware-free sim_only fallback demo (no robot/DDS needed)
#   ./scripts/lab_run.sh --native    # force the no-Docker path  (--docker forces the container)
#   ./scripts/lab_run.sh --rebuild   # clean colcon build of algae_dt first (docker: full ws clean)
#   ./scripts/lab_run.sh --no-gz-gui # skip the gz 3D client (weak GL stack; RViz+console still show all)
#   TB3_IMAGE=name / ROS_DOMAIN_ID=n / ROBOT_IP=ip   # overrides if image/robot differ
#   LAB_RUN_DRY=1                    # test hook: do everything except the final launch, then exit 0
#
# RViz opens AUTOMATICALLY in both/real_only (the launch use_rviz default is mode-aware) — it is
# REQUIRED there: the "2D Pose Estimate" click that seeds AMCL has no other UI. The --sim fallback
# also opens RViz and skips the crash-prone gz 3D client (mirrors docker/open_sim.sh).
#
# PROVENANCE: docker run / sourcing / build / launch are verbatim from Canvas ("How to run Gazebo",
# "Changing Robot Inflation", "DT Example", "Connecting lab laptop to robot") — docs/RUN_ON_LAB_PC.md.
# The native path runs the SAME sourcing/build/launch directly on the host (course §2 fallback).
# =============================================================================
set -euo pipefail

log() { printf '[%s] %s\n' "$1" "${*:2}" >&2; }
die() { log FATAL "$*"; exit 3; }

# ---- args ----
REBUILD=false
MODE="both"        # default: the lab runner brings up the FULL real+sim demo
GZ_GUI=true
RUNTIME=""         # "" = auto-detect | docker | native
for a in "$@"; do case "$a" in
  --rebuild) REBUILD=true ;;
  --sim|--fallback) MODE="sim_only" ;;
  --no-gz-gui) GZ_GUI=false ;;
  --docker) RUNTIME=docker ;;
  --native) RUNTIME=native ;;
  *) die "unknown arg: $a   (--sim hardware-free fallback | --native/--docker force the runtime | --rebuild clean build | --no-gz-gui skip the gz 3D client)" ;;
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
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

[ -d "$PKG_SRC" ] || die "package not found at $PKG_SRC — clone the repo and run this from inside it."

# ---- record exactly which commit we are about to demo (the laptop is wiped / a USB tree can be stale) ----
GIT_REF="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo '?')"
GIT_DIRTY=""; git -C "$REPO" diff --quiet 2>/dev/null || GIT_DIRTY=" (DIRTY working tree — uncommitted changes)"
log INFO "demoing algae_dt @ commit ${GIT_REF}${GIT_DIRTY}  (from $REPO)"

echo "===================== PREFLIGHT (mode=$MODE; errors if it can't run) =====================" >&2

# ---- runtime detection: docker (course-canonical) vs native (the no-Docker lab PC) ----
docker_usable()   { command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; }
docker_rootless() { docker info --format '{{.SecurityOptions}}' 2>/dev/null | grep -qi rootless; }
native_usable() {  # ROS Jazzy on the host + turtlebot3_gazebo resolvable (apt OR the from-source ws)
  [ -f "$ROS_SETUP" ] || return 1
  bash -c "set +u; source '$ROS_SETUP' >/dev/null 2>&1
           [ -f '$WS/install/setup.bash' ] && source '$WS/install/setup.bash' >/dev/null 2>&1
           ros2 pkg prefix turtlebot3_gazebo >/dev/null 2>&1"
}
native_diagnosis() {
  if [ ! -f "$ROS_SETUP" ]; then echo "no ROS at $ROS_SETUP"
  else echo "turtlebot3_gazebo not resolvable from $ROS_SETUP + $WS/install — if ~/turtlebot3_ws is broken/copied, rebuild it (no sudo): ./scripts/lab_fix_workspace.sh"
  fi
}

if [ -z "$RUNTIME" ]; then
  if [ "$MODE" = both ]; then
    if docker_usable && ! docker_rootless; then RUNTIME=docker
    elif native_usable; then
      RUNTIME=native
      log INFO "no rootful Docker here → using the NATIVE stack (native DDS reaches the robot fine)"
    else
      die "neither runtime can run 'both' on this machine:
       docker: $(docker_usable && echo "usable but ROOTLESS — no LAN multicast, the robot is unreachable over DDS" || echo "not usable (not installed / daemon down / no docker-group access)")
       native: $(native_diagnosis)
     Fix the native workspace first:  ./scripts/lab_fix_workspace.sh   (no sudo; rebuilds ~/turtlebot3_ws from the fail-safe zip in ~/Downloads)
     Or ask a TA to enable rootful Docker. For the hardware-free fallback demo:  $0 --sim"
    fi
  else
    if docker_usable; then RUNTIME=docker
    elif native_usable; then RUNTIME=native; log INFO "no Docker here → using the NATIVE stack"
    else
      die "neither runtime can run the sim demo on this machine:
       docker: not usable (not installed / daemon down / no docker-group access)
       native: $(native_diagnosis)"
    fi
  fi
else
  # forced runtime — verify it actually works rather than failing later, mysteriously
  if [ "$RUNTIME" = docker ]; then
    docker_usable || die "--docker: no usable Docker (not installed / daemon down / no docker-group access). The no-Docker path is --native."
    [ "$MODE" = both ] && docker_rootless && die "--docker: this Docker is ROOTLESS — no LAN multicast, so the robot is unreachable over DDS and 'both' cannot run. Use --native (the host stack), or ask a TA for rootful Docker."
  else
    native_usable || die "--native: $(native_diagnosis)"
  fi
fi
log OK "runtime=$RUNTIME  ROS_DOMAIN_ID=$DOMAIN  mode=$MODE"

# ---- docker only: a turtlebot3 image (build one if none; errors if the build can't) ----
ensure_image() {
  for img in "${IMAGE_CANDIDATES[@]}"; do
    if docker image inspect "$img" >/dev/null 2>&1; then IMAGE="$img"; log OK "using image '$IMAGE'"; return; fi
  done
  log WARN "no turtlebot3 image found (${IMAGE_CANDIDATES[*]}). Building '$FALLBACK_IMAGE' from scripts/Dockerfile — needs internet, ~10–20 min…"
  [ -f "$FALLBACK_DOCKERFILE" ] || die "fallback Dockerfile missing at $FALLBACK_DOCKERFILE."
  docker build -t "$FALLBACK_IMAGE" -f "$FALLBACK_DOCKERFILE" "$REPO/scripts" \
    || die "image build failed (needs internet for the base image + apt). The container path cannot run without a turtlebot3 image — try --native if the host has the stack."
  IMAGE="$FALLBACK_IMAGE"; log OK "built + using '$IMAGE'"
}
[ "$RUNTIME" = docker ] && ensure_image

# ---- (re)place the package in the workspace (course "How to create Packages" §3) ----
log INFO "(re)creating $WS/src/algae_dt"
mkdir -p "$WS/src"; rm -rf "$WS/src/algae_dt"; cp -r "$PKG_SRC" "$WS/src/algae_dt"
if [ "$REBUILD" = true ]; then
  if [ "$RUNTIME" = docker ]; then
    rm -rf "$WS/build" "$WS/install" "$WS/log"      # turtlebot3 lives in the IMAGE → safe to nuke all
  else
    # NATIVE: the ws install/ holds the FROM-SOURCE turtlebot3 stack — nuking it all would orphan
    # the demo (a --packages-select build would not bring it back). Clean only OUR package.
    log INFO "--rebuild (native): cleaning only build/install of algae_dt (the rest of install/ IS the turtlebot3 stack)"
    rm -rf "$WS/build/algae_dt" "$WS/install/algae_dt"
  fi
fi

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
  # ---- robot reachable (advisory fast-fail; the authoritative gate is /scan in the payload) ----
  ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1 \
    || die "robot $ROBOT_IP not reachable — 'both' needs the real robot. Power it on, start the Pi bringup, join Wi-Fi AP2IRR10, then re-run.
         For the hardware-free fallback demo instead (a valid graded sim_only demo):  $0 --sim"
  log OK "robot $ROBOT_IP reachable"
else
  log INFO "sim_only fallback: skipping the robot reachability check (no hardware needed)."
fi

# ---- THE PAYLOAD (identical for both runtimes): source underlays → require the tb3 stack →
#      (both) gate on /scan → build algae_dt → launch. Single-quoted: expands INSIDE the child bash.
# shellcheck disable=SC2016  # deliberate: $DT_* expand in the child shell, not here
PAYLOAD='
  set -e; set +u
  source /opt/ros/jazzy/setup.bash
  [ -f /opt/turtlebot3_ws/install/setup.bash ] && source /opt/turtlebot3_ws/install/setup.bash || true
  [ -f "$DT_WS/install/setup.bash" ] && source "$DT_WS/install/setup.bash" || true
  if ! ros2 pkg prefix turtlebot3_gazebo >/dev/null 2>&1; then
    echo "FATAL: turtlebot3 stack not resolvable (checked /opt/ros/jazzy, /opt/turtlebot3_ws/install,"
    echo "       $DT_WS/install). Container: this image lacks the stock turtlebot3 packages."
    echo "       Native: rebuild the workspace first:  ./scripts/lab_fix_workspace.sh"; exit 4
  fi
  cd "$DT_WS"
  if [ "$DT_MODE" = both ]; then
    echo "-- verifying the real-robot link: waiting up to 40s for /scan --"
    ok=0; for _ in $(seq 1 40); do ros2 topic list 2>/dev/null | grep -qx /scan && { ok=1; break; }; sleep 1; done
    if [ "$ok" != 1 ]; then
      echo "FATAL: robot /scan NOT visible — Pi bringup not up, wrong ROS_DOMAIN_ID, not on Wi-Fi"
      echo "       AP2IRR10, or (container) Docker networking cannot reach the robot."
      echo "       both cannot run. Fix and re-run (or use --sim); never record a demo on a dead link."; exit 5
    fi
    echo "-- real link OK (/scan visible). --"
  fi
  echo "-- colcon build --packages-select algae_dt --"
  colcon build --packages-select algae_dt
  source install/setup.bash
  if [ -n "${LAB_RUN_DRY:-}" ]; then
    echo "DRY-RUN OK: would exec: ros2 launch algae_dt bringup.launch.py mode:=$DT_MODE $DT_LAUNCH_ARGS"; exit 0
  fi
  echo "-- ros2 launch algae_dt bringup.launch.py mode:=$DT_MODE $DT_LAUNCH_ARGS --"
  exec ros2 launch algae_dt bringup.launch.py mode:="$DT_MODE" $DT_LAUNCH_ARGS
'

if [ "$RUNTIME" = native ]; then
  # =========================== NATIVE (no Docker — the lab PC as found 2026-06) ===========================
  [ -n "${DISPLAY:-}" ] || log WARN "DISPLAY is empty — Gazebo/RViz/the GUI need a graphical session"
  log INFO "launching mode:=$MODE NATIVELY (Ctrl-C to stop)"
  echo "   teleop in another terminal:  bash -lc 'source /opt/ros/jazzy/setup.bash; source ~/turtlebot3_ws/install/setup.bash; export ROS_DOMAIN_ID=$DOMAIN; ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw'" >&2
  echo >&2
  exec env TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID="$DOMAIN" \
       DT_WS="$WS" DT_MODE="$MODE" DT_LAUNCH_ARGS="$LAUNCH_ARGS" LAB_RUN_DRY="${LAB_RUN_DRY:-}" \
       bash -c "$PAYLOAD"
fi

# ================================== DOCKER (course-canonical container) ==================================
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

log INFO "launching mode:=$MODE inside '$IMAGE' (Ctrl-C to stop)"
echo "   teleop in another terminal:  docker exec -it $CONTAINER bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null; source /ws/install/setup.bash; ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw'" >&2
echo >&2

# ---- run: rootful uses the course --user $(id -u). The /scan gate lives in the payload. ----
exec docker run --rm -it --name "$CONTAINER" --net=host \
  "${X11_ARGS[@]}" \
  -v "$WS:/ws" -w /ws -e HOME=/ws \
  -e TURTLEBOT3_MODEL=burger -e ROS_DOMAIN_ID="$DOMAIN" \
  -e DT_WS=/ws -e DT_MODE="$MODE" -e DT_LAUNCH_ARGS="$LAUNCH_ARGS" -e LAB_RUN_DRY="${LAB_RUN_DRY:-}" \
  --user "$(id -u):$(id -g)" "$IMAGE" bash -lc "$PAYLOAD"
