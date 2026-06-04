#!/usr/bin/env bash
# =============================================================================
# lab_fix_workspace.sh — rebuild a HEALTHY ~/turtlebot3_ws on the lab laptop from the workspace
# zip in ~/Downloads, NATIVELY (no Docker) and WITHOUT sudo. Standalone: copy this one file to the
# lab PC (USB / OneDrive / git clone) and run it.
#
# THE PROBLEM IT FIXES — a turtlebot3_ws copied from another machine/user is unsalvageable in place:
#   * build/ + install/ hold CMake caches with the OLD absolute path baked in
#       "CMakeCache.txt directory ... is different than ... /home/test/turtlebot3_ws"
#   * copied files can be owned by another user -> "Permission denied" even on stat()
#   * the folder lands at a wrong path ("~/turtlebot3_ws (Copy)" — the space breaks half the tooling)
#   * ~/.bashrc still sources the old/broken paths
# Only src/ is portable. build/ install/ log/ MUST be regenerated on THIS machine — never repaired.
#
# WHAT IT DOES (all inside $HOME — no sudo anywhere; lab Golden Rule #12 safe):
#   1. preflight: native ROS 2 Jazzy + colcon present (no Docker needed), disk space
#   2. find the workspace archive in ~/Downloads (or --zip PATH); zip / tar.gz both work
#   3. extract ONLY src/ (skips the gigabytes of poisoned build/install) into a staging dir
#   4. quarantine every ~/turtlebot3_ws* dir (incl. "turtlebot3_ws (Copy)") -> turtlebot3_ws.broken.<ts>
#      (rename needs only write on $HOME — works even when the files inside are foreign-owned)
#   5. assemble a fresh ~/turtlebot3_ws/src, take ownership of the modes (chmod -R u+rwX),
#      strip stray CMakeCache.txt / __pycache__
#   6. if run from a cloned algae-dt repo: refresh src/algae_dt from the repo (source of truth)
#   7. repair ~/.bashrc: comment lines pointing at broken paths, install ONE managed env block
#   8. clean `colcon build --symlink-install` in a SANITIZED env (env -i): a shell that ever sourced
#      the broken workspace has a poisoned AMENT_PREFIX_PATH/PYTHONPATH and would poison the build
#   9. verify the key packages resolve; print the exact next steps
#
# Usage:
#   ./lab_fix_workspace.sh                   # full auto-recovery (newest workspace zip in ~/Downloads)
#   ./lab_fix_workspace.sh --zip PATH        # explicit archive (.zip / .tar.gz / .tgz / .tar)
#   ./lab_fix_workspace.sh --from-dir DIR    # no archive: salvage src/ from an existing (broken) ws dir
#   Options: --ws PATH (default ~/turtlebot3_ws)   --domain N (default 36 = our robot number)
#            --no-bashrc   --no-build   --keep-zip-algae   --purge-quarantine
#   Env overrides: ROS_SETUP=/opt/ros/jazzy/setup.bash (the ROS underlay to source)
#
# Exit codes: 0 ok | 2 bad usage | 3 preflight/extract failure | 4 build failure | 5 verify failure
# =============================================================================
set -euo pipefail

log() { printf '[%s] %s\n' "$1" "${*:2}" >&2; }
die() { log FATAL "$*"; exit 3; }

usage() { sed -n '2,38p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2; }

# ---------------------------------------------------------------------------- args
WS="$HOME/turtlebot3_ws"
ARCHIVE=""                 # --zip
FROM_DIR=""                # --from-dir
DOMAIN=36                  # --domain (= robot number; ours is #36)
DO_BASHRC=true
DO_BUILD=true
KEEP_ZIP_ALGAE=false
PURGE_QUARANTINE=false
while [ $# -gt 0 ]; do case "$1" in
  --zip)              ARCHIVE="${2:?--zip needs a path}"; shift 2 ;;
  --from-dir)         FROM_DIR="${2:?--from-dir needs a path}"; shift 2 ;;
  --ws)               WS="${2:?--ws needs a path}"; shift 2 ;;
  --domain)           DOMAIN="${2:?--domain needs a number}"; shift 2 ;;
  --no-bashrc)        DO_BASHRC=false; shift ;;
  --no-build)         DO_BUILD=false; shift ;;
  --keep-zip-algae)   KEEP_ZIP_ALGAE=true; shift ;;
  --purge-quarantine) PURGE_QUARANTINE=true; shift ;;
  -h|--help)          usage; exit 0 ;;
  *) log FATAL "unknown arg: $1"; usage; exit 2 ;;
esac; done
[[ "$DOMAIN" =~ ^[0-9]+$ ]] && [ "$DOMAIN" -le 232 ] || { log FATAL "--domain must be 0..232 (= robot number)"; exit 2; }
[ -n "$ARCHIVE" ] && [ -n "$FROM_DIR" ] && { log FATAL "--zip and --from-dir are mutually exclusive"; exit 2; }

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
TS="$(date +%Y%m%d-%H%M%S)"
# resolve where this script lives NOW, before anything is renamed/quarantined under it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_PKG="$SCRIPT_DIR/../ros2_ws/src/algae_dt"
STAGE=""
cleanup() { [ -n "$STAGE" ] && rm -rf -- "$STAGE" 2>/dev/null || true; }
trap cleanup EXIT

# Run a command line in a SANITIZED environment: env -i drops the (possibly poisoned)
# AMENT_PREFIX_PATH / PYTHONPATH / CMAKE_PREFIX_PATH a broken-workspace shell carries.
run_clean() {
  env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" SHELL=/bin/bash \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      TERM="${TERM:-xterm}" LANG="${LANG:-C.UTF-8}" \
      bash --noprofile --norc -c "$1"
}

# Escape glob metacharacters so a prefix like "turtlebot3_ws (Copy)/" is matched literally
# by unzip/tar include patterns (spaces and parens are safe; * ? [ \ are not).
escape_glob() { local s=$1; s=${s//\\/\\\\}; s=${s//\*/\\*}; s=${s//\?/\\?}; s=${s//\[/\\[}; printf '%s' "$s"; }

# ---------------------------------------------------------------------------- 1. preflight
log INFO "lab workspace recovery — target workspace: $WS  (no sudo, no Docker, all inside \$HOME)"
[ "$(id -u)" = 0 ] && log WARN "running as root is unnecessary — everything here is user-level"

if [ -f "$ROS_SETUP" ]; then
  log OK "ROS underlay found: $ROS_SETUP"
else
  $DO_BUILD && die "ROS setup not found at $ROS_SETUP — this script is for the NATIVE lab laptop (ROS 2 Jazzy preinstalled). Override with ROS_SETUP=... if the distro lives elsewhere."
  log WARN "ROS setup not found at $ROS_SETUP (continuing: --no-build)"
fi
if $DO_BUILD; then
  run_clean "source '$ROS_SETUP' >/dev/null 2>&1; command -v colcon >/dev/null" \
    || die "colcon not found (even after sourcing $ROS_SETUP). On the lab laptop it is preinstalled; if truly absent: pip install --user -U colcon-common-extensions   (no sudo)"
  log OK "colcon available"
fi
command -v python3 >/dev/null || log WARN "python3 not found — archive-extraction fallbacks unavailable"

free_kb="$(df -Pk "$HOME" 2>/dev/null | awk 'NR==2{print $4}')" || free_kb=""
if [ -n "$free_kb" ] && [ "$free_kb" -lt $((4 * 1024 * 1024)) ]; then
  log WARN "less than 4 GB free in \$HOME ($((free_kb / 1024)) MB) — the clean build may run out of disk"
fi

# ---------------------------------------------------------------------------- 2. locate the source of src/
# list_archive ARCHIVE -> member names on stdout (zip via unzip -Z1, else python; tar via tar -tf)
list_archive() {
  local ar=$1
  case "$ar" in
    *.zip)
      if command -v unzip >/dev/null && unzip -Z1 -- "$ar" 2>/dev/null; then return 0; fi
      python3 -c 'import sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as z:
    sys.stdout.write("\n".join(z.namelist()) + "\n")' "$ar" ;;
    *.tar.gz|*.tgz|*.tar) tar -tf "$ar" ;;
    *) return 1 ;;
  esac
}

# src_prefix_of LISTING-FILE -> the path prefix before the workspace "src/" component, by majority
# vote over all */package.xml entries (robust against packages that have their own inner src/ dirs).
# Prints the prefix ("" if src/ is at the archive root); prints "@NOSRC@" if packages exist but
# there is no src/ component at all (someone zipped the CONTENTS of src/).
src_prefix_of() {
  local listing=$1 e p best="" best_n=0 any_pkg=false
  declare -A votes=()
  while IFS= read -r e; do
    case "$e" in */package.xml|package.xml) ;; *) continue ;; esac
    any_pkg=true
    case "$e" in
      src/*)   p="" ;;
      */src/*) p="${e%%/src/*}/" ;;   # shortest prefix before the FIRST /src/ component
      *)       continue ;;
    esac
    votes["$p|"]=$(( ${votes["$p|"]:-0} + 1 ))   # "|" suffix: assoc keys may not be empty
  done < "$listing"
  for p in "${!votes[@]}"; do
    if [ "${votes[$p]}" -gt "$best_n" ]; then best_n=${votes[$p]}; best=$p; fi
  done
  if [ "$best_n" -gt 0 ]; then printf '%s' "${best%|}"; return 0; fi
  $any_pkg && { printf '@NOSRC@'; return 0; }
  return 1
}

find_archive_in_downloads() {
  local d="$HOME/Downloads" cand newest="" listing sz
  [ -d "$d" ] || die "no archive given (--zip) and $d does not exist"
  listing="$(mktemp)"
  shopt -s nullglob
  for cand in "$d"/*.zip "$d"/*.tar.gz "$d"/*.tgz "$d"/*.tar; do
    # zips list from the central directory (cheap at any size); tar has no index — `tar -tf`
    # decompresses the WHOLE stream, so skip huge unrelated tarballs instead of silently hanging
    case "$cand" in
      *.tar.gz|*.tgz|*.tar)
        sz="$(stat -c%s -- "$cand" 2>/dev/null || echo 0)"
        if [ "$sz" -gt $((2 * 1024 * 1024 * 1024)) ]; then
          log WARN "skipping huge tar archive ($((sz / 1024 / 1024)) MB, listing would read it all): $cand — if this IS the workspace, pass it explicitly with --zip"
          continue
        fi ;;
    esac
    log INFO "inspecting: $cand"
    list_archive "$cand" > "$listing" 2>/dev/null || continue
    grep -Eq '(^|/)package\.xml$' "$listing" || continue       # must look like a colcon workspace
    if [ -z "$newest" ] || [ "$cand" -nt "$newest" ]; then newest="$cand"; fi
  done
  shopt -u nullglob
  rm -f -- "$listing"
  [ -n "$newest" ] || die "no workspace archive found in $d (looked for *.zip/*.tar.gz/*.tgz/*.tar containing package.xml files). Pass it explicitly:  $0 --zip /path/to/workspace.zip"
  printf '%s' "$newest"
}

# extract_src ARCHIVE PREFIX DEST — extract only PREFIX + everything below it (PREFIX="" = whole
# archive). Tries the fast native tool first, falls back to python (exact string prefix, no glob).
extract_src() {
  local ar=$1 prefix=$2 dest=$3 pat
  pat="$(escape_glob "$prefix")"
  case "$ar" in
    *.zip)
      if command -v unzip >/dev/null; then
        if [ -n "$prefix" ]; then unzip -q -- "$ar" "${pat}*" -d "$dest" && return 0
        else unzip -q -- "$ar" -d "$dest" && return 0; fi
        log WARN "unzip extraction failed — retrying with the python fallback"
        rm -rf -- "$dest"; mkdir -p -- "$dest"
      fi
      python3 -c 'import sys, os, zipfile
ar, prefix, dest = sys.argv[1:4]
with zipfile.ZipFile(ar) as z:
    members = [m for m in z.infolist() if m.filename.startswith(prefix)]
    if not members:
        sys.exit("no members under prefix %r" % prefix)
    for m in members:
        out = z.extract(m, dest)                      # zipfile sanitizes ../ traversal
        if not m.is_dir():
            mode = (m.external_attr >> 16) & 0o7777   # restore unix mode bits, keep them readable
            if mode:
                os.chmod(out, mode | 0o600)' "$ar" "$prefix" "$dest" ;;
    *.tar.gz|*.tgz|*.tar)
      if [ -n "$prefix" ]; then
        tar -xf "$ar" -C "$dest" --wildcards -- "${pat}*" 2>/dev/null && return 0
        log WARN "tar selective extraction failed — retrying with the python fallback"
        rm -rf -- "$dest"; mkdir -p -- "$dest"
        python3 -c 'import sys, tarfile
ar, prefix, dest = sys.argv[1:4]
with tarfile.open(ar) as t:
    members = [m for m in t.getmembers() if m.name.startswith(prefix)]
    if not members:
        sys.exit("no members under prefix %r" % prefix)
    t.extractall(dest, members=members, filter="data")' "$ar" "$prefix" "$dest"
      else
        tar -xf "$ar" -C "$dest"
      fi ;;
    *) return 1 ;;
  esac
}

STAGE="$(mktemp -d "$HOME/.lab_fix_stage.XXXXXX")"
STAGED_SRC=""                                        # will point at the staged src/ tree

if [ -n "$FROM_DIR" ]; then
  # ---- salvage path: copy src/ out of an existing (broken) directory ----
  [ -d "$FROM_DIR" ] || die "--from-dir: $FROM_DIR is not a directory"
  local_src="$FROM_DIR"
  [ -d "$FROM_DIR/src" ] && local_src="$FROM_DIR/src"
  log INFO "salvaging src/ from directory: $local_src"
  if ! cp -a -- "$local_src" "$STAGE/src" 2>"$STAGE/cp.err"; then
    sed 's/^/    /' "$STAGE/cp.err" >&2 || true
    die "could not read everything under $local_src (foreign-owned files?). Use the zip instead: $0 --zip ~/Downloads/<workspace>.zip"
  fi
  STAGED_SRC="$STAGE/src"
else
  # ---- archive path (the normal case) ----
  if [ -z "$ARCHIVE" ]; then
    ARCHIVE="$(find_archive_in_downloads)"
    log OK "workspace archive found: $ARCHIVE"
  else
    [ -f "$ARCHIVE" ] || die "--zip: $ARCHIVE not found"
  fi
  LISTING="$STAGE/.listing"
  list_archive "$ARCHIVE" > "$LISTING" || die "cannot list $ARCHIVE (unsupported or corrupt archive)"
  PREFIX="$(src_prefix_of "$LISTING")" || die "$ARCHIVE contains no package.xml — not a colcon workspace archive"
  if [ "$PREFIX" = "@NOSRC@" ]; then
    log WARN "archive has packages but no src/ component (the CONTENTS of src/ were zipped) — extracting everything as src/"
    mkdir -p "$STAGE/raw"
    extract_src "$ARCHIVE" "" "$STAGE/raw" || die "extraction failed"
    STAGED_SRC="$STAGE/raw"
  else
    log INFO "workspace root inside the archive: '${PREFIX:-<archive root>}' — extracting ONLY '${PREFIX}src/' (build/ install/ are machine-specific poison and are skipped)"
    extract_src "$ARCHIVE" "${PREFIX}src/" "$STAGE" || die "extraction failed"
    STAGED_SRC="$STAGE/${PREFIX}src"
  fi
fi

# we own everything we just wrote — normalize modes so mode-000 leftovers can't hurt us again
chmod -R u+rwX -- "$STAGE" 2>/dev/null || true
[ -d "$STAGED_SRC" ] || die "internal: staged src not found at $STAGED_SRC"
n_pkgs="$(find "$STAGED_SRC" -name package.xml -not -path '*/node_modules/*' | wc -l)"
[ "$n_pkgs" -gt 0 ] || die "no package.xml under the staged src/ — wrong archive? ($ARCHIVE)"
log OK "staged src/ holds $n_pkgs package manifest(s)"

# ---------------------------------------------------------------------------- 4. quarantine broken dirs
QUARANTINED_NAMES=()
quarantine() {
  local d=$1 dest n=0
  dest="$HOME/turtlebot3_ws.broken.$TS"
  while [ -e "$dest" ]; do n=$((n + 1)); dest="$HOME/turtlebot3_ws.broken.$TS.$n"; done
  mv -- "$d" "$dest" || die "could not move '$d' aside (need write on \$HOME)"
  QUARANTINED_NAMES+=("$(basename -- "$d")")
  log OK "quarantined: '$d'  ->  '$dest'"
}
shopt -s nullglob
for d in "$HOME"/turtlebot3_ws*; do
  [ -d "$d" ] || continue
  case "$(basename -- "$d")" in turtlebot3_ws.broken.*) continue ;; esac
  quarantine "$d"
done
shopt -u nullglob
[ -d "$WS" ] && quarantine "$WS"      # --ws outside $HOME/turtlebot3_ws*
if [ ${#QUARANTINED_NAMES[@]} -eq 0 ]; then log INFO "no pre-existing workspace dirs to quarantine"; fi

# ---------------------------------------------------------------------------- 5. assemble the fresh workspace
mkdir -p -- "$WS"
mv -- "$STAGED_SRC" "$WS/src"
# strip the only build poison that can hide inside src/: in-source CMake caches + python bytecode
find "$WS/src" -name CMakeCache.txt -type f -delete
find "$WS/src" -depth -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
chmod -R u+rwX -- "$WS"
find "$WS/src" -type f -name '*.sh' -exec chmod u+x {} +
log OK "fresh workspace assembled at $WS (src only — build/install/log will be regenerated)"

# ---------------------------------------------------------------------------- 6. refresh algae_dt from the repo
if [ -d "$REPO_PKG" ] && ! $KEEP_ZIP_ALGAE; then
  # drop whatever algae_dt the archive carried (possibly stale), use the repo checkout instead
  while IFS= read -r -d '' manifest; do
    grep -q '<name>algae_dt</name>' "$manifest" 2>/dev/null || continue
    rm -rf -- "$(dirname -- "$manifest")"
  done < <(find "$WS/src" -name package.xml -print0)
  rm -rf -- "$WS/src/algae_dt"           # belt-and-braces: cp -r into an existing dir would NEST
  cp -r -- "$REPO_PKG" "$WS/src/algae_dt"
  ref="$(git -C "$SCRIPT_DIR/.." rev-parse --short HEAD 2>/dev/null || echo '?')"
  log OK "src/algae_dt refreshed from the repo checkout (@ $ref) — use --keep-zip-algae to keep the archive's copy"
elif [ -d "$REPO_PKG" ]; then
  log INFO "--keep-zip-algae: keeping the archive's algae_dt"
else
  log INFO "no repo checkout next to this script — keeping the archive's algae_dt (if any)"
fi

# ---------------------------------------------------------------------------- inventory
INVENTORY="$(run_clean "source '$ROS_SETUP' >/dev/null 2>&1 || true; cd '$WS' && colcon list --names-only --base-paths src 2>/dev/null" || true)"
if [ -n "$INVENTORY" ]; then
  log OK "packages in src/: $(echo "$INVENTORY" | tr '\n' ' ')"
elif $DO_BUILD; then
  die "colcon recognizes NO packages under $WS/src — refusing to 'build' nothing and call it recovered (incomplete archive, or invalid package manifests)"
else
  log WARN "colcon list found no packages — a later build would do nothing"
fi
for want in turtlebot3_msgs turtlebot3_description turtlebot3_gazebo turtlebot3_navigation2 \
            turtlebot3_bringup turtlebot3_teleop algae_dt; do
  echo "$INVENTORY" | grep -qx "$want" || log WARN "expected package NOT in src/: $want (the archive may be incomplete)"
done

# ---------------------------------------------------------------------------- 7. repair ~/.bashrc
MARK_START='# >>> algae-dt lab_fix_workspace >>>'
MARK_END='# <<< algae-dt lab_fix_workspace <<<'
repair_rc() {
  local rc=$1 tmp line commented=0
  [ -f "$rc" ] || return 0
  cp -p -- "$rc" "$rc.lab_fix.bak.$TS"
  tmp="$(mktemp "$rc.XXXXXX")"
  # 1) drop any previous managed block (idempotent re-runs)
  awk -v s="$MARK_START" -v e="$MARK_END" '$0==s{skip=1} !skip{print} $0==e{skip=0}' "$rc" > "$tmp"
  # 2) comment out lines that still point at broken paths
  local patterns=("/home/test/turtlebot3_ws" "turtlebot3_ws (")
  local q; for q in "${QUARANTINED_NAMES[@]:-}"; do
    [ -n "$q" ] && [ "$q" != "turtlebot3_ws" ] && patterns+=("$q")
  done
  local out; out="$(mktemp "$rc.XXXXXX")"
  while IFS= read -r line || [ -n "$line" ]; do
    local hit=false p trimmed
    trimmed="${line#"${line%%[![:space:]]*}"}"        # ltrim — a leading '#' means already commented
    if [[ "$trimmed" != \#* ]]; then
      for p in "${patterns[@]}"; do
        [[ "$line" == *"$p"* ]] && { hit=true; break; }
      done
    fi
    if $hit; then
      printf '# [disabled by lab_fix_workspace %s] %s\n' "$TS" "$line" >> "$out"
      commented=$((commented + 1))
    else
      printf '%s\n' "$line" >> "$out"
    fi
  done < "$tmp"
  rm -f -- "$tmp"
  chmod --reference="$rc" "$out" 2>/dev/null || chmod 644 "$out"
  mv -- "$out" "$rc"
  [ "$commented" -gt 0 ] && log OK "$rc: commented out $commented line(s) pointing at broken paths (backup: $rc.lab_fix.bak.$TS)"
  return 0
}
if $DO_BASHRC; then
  repair_rc "$HOME/.bashrc"
  repair_rc "$HOME/.bash_aliases"
  {
    printf '%s\n' "$MARK_START"
    printf 'source %s\n' "$ROS_SETUP"
    printf '[ -f "%s/install/setup.bash" ] && source "%s/install/setup.bash"\n' "$WS" "$WS"
    printf 'export TURTLEBOT3_MODEL=burger\n'
    printf 'export LDS_MODEL=LDS-02\n'
    printf 'export ROS_DOMAIN_ID=%s   # = robot number (sticker) — robot and laptop MUST match\n' "$DOMAIN"
    printf '%s\n' "$MARK_END"
  } >> "$HOME/.bashrc"
  log OK "$HOME/.bashrc: managed env block installed (ROS_DOMAIN_ID=$DOMAIN, TURTLEBOT3_MODEL=burger, LDS_MODEL=LDS-02)"
else
  log INFO "--no-bashrc: leaving shell config untouched"
fi

# ---------------------------------------------------------------------------- 8. clean build
if $DO_BUILD; then
  log INFO "clean colcon build (sanitized environment — this regenerates build/ install/ for THIS machine)…"
  SECONDS=0
  build_cmd=""
  printf -v build_cmd 'source %q && cd %q && colcon build --symlink-install' "$ROS_SETUP" "$WS"
  if run_clean "$build_cmd"; then
    log OK "colcon build succeeded in ${SECONDS}s"
  else
    log FATAL "colcon build FAILED (see the colcon output above; full logs in $WS/log/latest_build/)."
    log FATAL "Common causes on the lab laptop:"
    log FATAL "  * a missing system dependency -> you cannot sudo; ask a TA to install it (course process)"
    log FATAL "  * an optional package failing (e.g. turtlebot3_cartographer) -> rebuild without it:"
    log FATAL "      cd $WS && colcon build --symlink-install --packages-skip <failing-pkg>"
    exit 4
  fi
else
  log INFO "--no-build: skipped. Build later with:"
  log INFO "    source $ROS_SETUP && cd $WS && colcon build --symlink-install"
fi

# ---------------------------------------------------------------------------- 9. verify
VERIFY_FAILED=false
VERIFIED_N=0
if $DO_BUILD; then
  for pkg in $INVENTORY; do
    case "$pkg" in
      turtlebot3_msgs|turtlebot3_description|turtlebot3_gazebo|turtlebot3_navigation2|turtlebot3_bringup|turtlebot3_teleop|algae_dt)
        printf -v vcmd 'source %q >/dev/null 2>&1 && source %q/install/setup.bash >/dev/null 2>&1 && ros2 pkg prefix %q >/dev/null' \
          "$ROS_SETUP" "$WS" "$pkg"
        if run_clean "$vcmd"; then
          log OK "verified: $pkg resolves from the new install/"
          VERIFIED_N=$((VERIFIED_N + 1))
        else
          log FATAL "package '$pkg' built but does NOT resolve — workspace still broken"
          VERIFY_FAILED=true
        fi ;;
    esac
  done
  if [ "$VERIFIED_N" = 0 ] && ! $VERIFY_FAILED; then
    log WARN "none of the key turtlebot3/algae_dt packages were in src/ to verify — check the inventory above"
  fi
fi
$VERIFY_FAILED && exit 5

# ---------------------------------------------------------------------------- summary
echo >&2
log DONE "workspace recovered at $WS"
if [ ${#QUARANTINED_NAMES[@]} -gt 0 ]; then
  if $PURGE_QUARANTINE; then
    for d in "$HOME"/turtlebot3_ws.broken."$TS"*; do
      chmod -R u+rwX -- "$d" 2>/dev/null || true
      if rm -rf -- "$d" 2>/dev/null; then
        log OK "purged quarantine: $d"
      else
        log WARN "could not fully delete $d (foreign-owned files need a TA with sudo; it only costs disk space meanwhile)"
      fi
    done
  else
    log INFO "old dir(s) kept as ~/turtlebot3_ws.broken.$TS* — delete later with --purge-quarantine (or ask a TA if rm fails)"
  fi
fi
cat >&2 <<EOF

NEXT STEPS
  1. open a NEW terminal (or: source ~/.bashrc) — old terminals carry the broken environment
  2. sanity check:        ros2 pkg list | grep -E 'turtlebot3|algae_dt'
  3. robot side (ssh):    ssh turtlebot@192.168.8.36
                          export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=$DOMAIN
                          ros2 launch turtlebot3_bringup robot.launch.py
  4. laptop demo:         ros2 launch algae_dt bringup.launch.py mode:=both     # or mode:=sim_only
  (full runbook: docs/RUN_ON_LAB_PC.md in the algae-dt repo)
EOF
