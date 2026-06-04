#!/usr/bin/env bash
# =============================================================================
# test_lab_fix_workspace.sh — end-to-end tests for lab_fix_workspace.sh.
#
# Runs INSIDE a ROS 2 Jazzy Linux container (algae-dt:dev / ros:jazzy + colcon) AS ROOT:
# root is used ONLY to stage the breakage the lab PC has (files owned by a foreign uid, mode-000
# dirs, a "turtlebot3_ws (Copy)" dir) — the script under test always runs as the unprivileged
# user `tester` (no sudo), exactly like team36 on the lab Z-Book.
#
# Run from the repo root on the dev machine (PowerShell):
#   docker run --rm -v ${PWD}:/repo algae-dt:dev bash /repo/scripts/test_lab_fix_workspace.sh
#
# Failure modes replicated from the real lab incident:
#   * build/install in the archive poisoned with CMakeCache.txt pointing at /home/test/turtlebot3_ws
#   * install/.../local_setup.dsv with mode 000 (the "Permission denied even on stat()" error)
#   * on-disk "~/turtlebot3_ws (Copy)" with foreign-owned unreadable dirs (undeletable without sudo)
#   * a second on-disk ~/turtlebot3_ws holding the user's local modifications (must be PRESERVED)
#   * ~/.bashrc sourcing /home/test/... and the "(Copy)" path
#   * archive whose internal folder name is arbitrary (NOT "(Copy)") — the real fail-safe zip
# =============================================================================
set -uo pipefail

PASS=0; FAIL=0
ok()   { PASS=$((PASS + 1)); printf 'PASS  %s\n' "$*"; }
bad()  { FAIL=$((FAIL + 1)); printf 'FAIL  %s\n' "$*"; }
check() { # check <description> <command...>
  local desc=$1; shift
  if "$@" >/dev/null 2>&1; then ok "$desc"; else bad "$desc"; fi
}
section() { printf '\n========== %s ==========\n' "$*"; }

[ "$(id -u)" = 0 ] || { echo "run me as root inside the container (root stages the breakage; the script under test runs as 'tester')"; exit 2; }
[ -f /opt/ros/jazzy/setup.bash ] || { echo "needs ROS 2 Jazzy (run inside algae-dt:dev / ros:jazzy + colcon)"; exit 2; }
command -v colcon >/dev/null || { echo "needs colcon in the image"; exit 2; }

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T=/tmp/labfix; rm -rf "$T"; mkdir -p "$T/bin"
# strip CRLF defensively (the repo may be checked out on Windows) and stage the script STANDALONE
# (no repo next to it) so the algae_dt-refresh path stays off unless a test enables it
tr -d '\r' < "$SELF_DIR/lab_fix_workspace.sh" > "$T/bin/lab_fix_workspace.sh"
SCRIPT="$T/bin/lab_fix_workspace.sh"

id tester >/dev/null 2>&1 || useradd -m -u 1234 -s /bin/bash tester
FOREIGN_UID=4321   # simulates the other machine's user — files tester cannot read or delete

run_fix() { # run_fix <home> <logfile> [args…] — run the script under test as the unprivileged user
  local h=$1 lg=$2; shift 2
  runuser -u tester -- env HOME="$h" USER=tester LOGNAME=tester \
    bash "$SCRIPT" "$@" >"$lg" 2>&1
}

# ---------------------------------------------------------------------------- fixture builders
make_pkg_py() { # make_pkg_py <dir> <name> — minimal valid ament_python package
  local d=$1 n=$2
  mkdir -p "$d/resource"; : > "$d/resource/$n"
  cat > "$d/package.xml" <<EOF
<?xml version="1.0"?>
<package format="3">
  <name>$n</name><version>0.0.1</version>
  <description>test fixture</description>
  <maintainer email="test@example.com">t</maintainer><license>MIT</license>
  <export><build_type>ament_python</build_type></export>
</package>
EOF
  cat > "$d/setup.py" <<EOF
from setuptools import setup
setup(name='$n', version='0.0.1', packages=[],
      data_files=[('share/ament_index/resource_index/packages', ['resource/$n']),
                  ('share/$n', ['package.xml'])])
EOF
}

make_pkg_cmake() { # make_pkg_cmake <dir> <name> — minimal valid ament_cmake package
  local d=$1 n=$2
  mkdir -p "$d"
  cat > "$d/package.xml" <<EOF
<?xml version="1.0"?>
<package format="3">
  <name>$n</name><version>0.0.1</version>
  <description>test fixture</description>
  <maintainer email="test@example.com">t</maintainer><license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF
  cat > "$d/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.8)
project($n)
find_package(ament_cmake REQUIRED)
ament_package()
EOF
}

# Build the fail-safe-zip workspace tree exactly like the real one: arbitrary root name (the user
# said it will NOT contain "Copy"), real package nesting (DynamixelSDK 3 levels deep, an inner
# src/ dir inside a package), poisoned build/install, decoy package.xml under log/.
make_ws_tree() { # make_ws_tree <parent> <wsname>
  local root=$1/$2
  make_pkg_cmake "$root/src/turtlebot3_msgs"                       turtlebot3_msgs
  make_pkg_cmake "$root/src/DynamixelSDK/ros/dynamixel_sdk"        dynamixel_sdk
  make_pkg_cmake "$root/src/turtlebot3/turtlebot3_node"            turtlebot3_node
  mkdir -p "$root/src/turtlebot3/turtlebot3_node/src"
  echo 'int main(){return 0;}' > "$root/src/turtlebot3/turtlebot3_node/src/dummy.c"  # inner src/!
  make_pkg_py    "$root/src/turtlebot3/turtlebot3_teleop"          turtlebot3_teleop
  make_pkg_py    "$root/src/algae_dt"                              algae_dt
  echo 'STALE-ZIP-COPY' > "$root/src/algae_dt/ZIP_MARKER"
  # the poison: build/install generated on the OTHER machine (/home/test) + mode-000 artifact
  mkdir -p "$root/build/turtlebot3_msgs" "$root/install/turtlebot3_msgs/share/turtlebot3_msgs"
  printf 'CMAKE_CACHEFILE_DIR:INTERNAL=/home/test/turtlebot3_ws/build/turtlebot3_msgs\n' \
    > "$root/build/turtlebot3_msgs/CMakeCache.txt"
  : > "$root/install/turtlebot3_msgs/share/turtlebot3_msgs/local_setup.dsv"
  chmod 000 "$root/install/turtlebot3_msgs/share/turtlebot3_msgs/local_setup.dsv"
  # decoy package.xml NOT under the real src/ (must not fool the prefix detection)
  mkdir -p "$root/log/build/src/decoy"
  echo '<package><name>decoy</name></package>' > "$root/log/build/src/decoy/package.xml"
  # a mode-000 directory WITH content inside src/ (zip stores it; the script must recover modes)
  mkdir -p "$root/src/turtlebot3_msgs/locked_dir"
  echo 'locked content' > "$root/src/turtlebot3_msgs/locked_dir/file.txt"
  chmod 000 "$root/src/turtlebot3_msgs/locked_dir"
}

zip_tree() { # zip_tree <parent> <wsname> <out.zip> — python zipfile preserves unix modes
  python3 - "$1" "$2" "$3" <<'PY'
import os, sys, zipfile
parent, name, out = sys.argv[1:4]
os.chdir(parent)
with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
    for root, dirs, files in os.walk(name):
        for d in dirs:
            p = os.path.join(root, d)
            z.write(p, p)            # dir entry, mode preserved in external_attr
        for f in files:
            p = os.path.join(root, f)
            z.write(p, p)
PY
}

# Stage a broken $HOME like the lab PC's: the "(Copy)" build attempt with foreign-owned files,
# the locally-modified original ws, a poisoned .bashrc, and the fail-safe zip in ~/Downloads.
make_broken_home() { # make_broken_home <home> <wsname-in-zip>
  local h=$1 wsname=$2
  rm -rf "$h"; mkdir -p "$h/Downloads"
  local fx="$T/fixture.$RANDOM"; mkdir -p "$fx"
  make_ws_tree "$fx" "$wsname"
  zip_tree "$fx" "$wsname" "$h/Downloads/ws_failsafe.zip"
  chmod -R u+rwX "$fx" && rm -rf "$fx"
  # on-disk broken copy (the one from the error log) with files tester cannot read or delete
  mkdir -p "$h/turtlebot3_ws (Copy)/install/turtlebot3_msgs"
  echo poisoned > "$h/turtlebot3_ws (Copy)/install/turtlebot3_msgs/local_setup.dsv"
  mkdir -p "$h/turtlebot3_ws (Copy)/foreign_dir"
  echo unreachable > "$h/turtlebot3_ws (Copy)/foreign_dir/file"
  chown -R "$FOREIGN_UID:$FOREIGN_UID" "$h/turtlebot3_ws (Copy)/foreign_dir"
  chmod 700 "$h/turtlebot3_ws (Copy)/foreign_dir"
  # on-disk modified original (the "tested random things" one) — must be preserved, not deleted
  mkdir -p "$h/turtlebot3_ws/src"
  echo 'my random modifications' > "$h/turtlebot3_ws/src/LOCAL_MODS"
  # .bashrc as left behind: stale sources + one already-commented line + one innocent line
  cat > "$h/.bashrc" <<'EOF'
alias gs='git status'
source /home/test/turtlebot3_ws/install/setup.bash
source "$HOME/turtlebot3_ws (Copy)/install/setup.bash"
# old note: source /home/test/turtlebot3_ws/install/setup.bash
EOF
  chown -R tester:tester "$h" 2>/dev/null || true
  chown -R "$FOREIGN_UID:$FOREIGN_UID" "$h/turtlebot3_ws (Copy)/foreign_dir"
}

# ============================================================================ T1: full recovery
section "T1: full recovery from the fail-safe zip (arbitrary ws name, no 'Copy')"
H1="$T/home1"; make_broken_home "$H1" "my_old_ws"
if run_fix "$H1" "$T/t1.log"; then ok "T1 script exit 0"; else bad "T1 script exit 0 (see $T/t1.log)"; sed 's/^/    | /' "$T/t1.log" | tail -40; fi

WS1="$H1/turtlebot3_ws"
check "T1 fresh ws/src exists"                         test -d "$WS1/src"
check "T1 pkg turtlebot3_msgs extracted"               test -f "$WS1/src/turtlebot3_msgs/package.xml"
check "T1 nested DynamixelSDK extracted"               test -f "$WS1/src/DynamixelSDK/ros/dynamixel_sdk/package.xml"
check "T1 inner-src package extracted"                 test -f "$WS1/src/turtlebot3/turtlebot3_node/src/dummy.c"
# the build legitimately REGENERATES install/.../local_setup.dsv — so assert the poison is gone by
# meaning, not by filename: no install artifacts under src/, the regenerated .dsv is readable (the
# zip's was mode 000), and the new CMake cache points at THIS machine instead of /home/test
check "T1 no install artifacts under src/"             bash -c "! find '$WS1/src' -name local_setup.dsv | grep -q ."
check "T1 regenerated .dsv readable (not the 000 one)" runuser -u tester -- test -r "$WS1/install/turtlebot3_msgs/share/turtlebot3_msgs/local_setup.dsv"
check "T1 new CMake cache points at THIS machine"      bash -c "grep -q 'CMAKE_CACHEFILE_DIR:INTERNAL=$WS1/build/turtlebot3_msgs' '$WS1/build/turtlebot3_msgs/CMakeCache.txt' && ! grep -q '/home/test' '$WS1/build/turtlebot3_msgs/CMakeCache.txt'"
check "T1 poisoned build/ NOT extracted"               bash -c "! find '$WS1/src' -name CMakeCache.txt | grep -q ."
check "T1 decoy log/ tree NOT extracted"               bash -c "! test -e '$WS1/src/decoy' && ! find '$WS1' -path '*log/build*' -name package.xml | grep -q ."
check "T1 mode-000 dir recovered + readable"           runuser -u tester -- cat "$WS1/src/turtlebot3_msgs/locked_dir/file.txt"
check "T1 build/ regenerated on THIS machine"          test -f "$WS1/install/setup.bash"
check "T1 cmake pkg built"                             test -d "$WS1/install/turtlebot3_msgs"
check "T1 python pkg built"                            test -d "$WS1/install/turtlebot3_teleop"
check "T1 '(Copy)' dir quarantined"                    bash -c "! test -e '$H1/turtlebot3_ws (Copy)'"
check "T1 modified original quarantined+preserved"     bash -c "cat \"\$(ls -d '$H1'/turtlebot3_ws.broken.*/src/LOCAL_MODS 2>/dev/null | head -1)\" | grep -q 'random modifications'"
check "T1 foreign-owned file still intact (not lost)"  bash -c "find '$H1'/turtlebot3_ws.broken.* -name file -path '*foreign_dir*' | grep -q ."
Q_COUNT=$(ls -d "$H1"/turtlebot3_ws.broken.* 2>/dev/null | wc -l)
[ "$Q_COUNT" = 2 ] && ok "T1 exactly 2 dirs quarantined" || bad "T1 exactly 2 dirs quarantined (got $Q_COUNT)"

check "T1 bashrc: /home/test line disabled"            bash -c "grep -q '^# \[disabled by lab_fix_workspace.*source /home/test/turtlebot3_ws' '$H1/.bashrc'"
check "T1 bashrc: (Copy) line disabled"                bash -c "grep -q '^# \[disabled by lab_fix_workspace.*turtlebot3_ws (Copy)' '$H1/.bashrc'"
check "T1 bashrc: innocent alias untouched"            bash -c "grep -qx \"alias gs='git status'\" '$H1/.bashrc'"
check "T1 bashrc: pre-commented line NOT re-commented" bash -c "grep -qx '# old note: source /home/test/turtlebot3_ws/install/setup.bash' '$H1/.bashrc'"
check "T1 bashrc: managed block present once"          bash -c "[ \"\$(grep -c '^# >>> algae-dt lab_fix_workspace >>>$' '$H1/.bashrc')\" = 1 ]"
check "T1 bashrc: ROS_DOMAIN_ID=36 set"                bash -c "grep -q '^export ROS_DOMAIN_ID=36' '$H1/.bashrc'"
check "T1 bashrc: backup created"                      bash -c "ls '$H1'/.bashrc.lab_fix.bak.* | grep -q ."
check "T1 a NEW login shell resolves the packages"     runuser -u tester -- env HOME="$H1" bash -c 'source $HOME/.bashrc >/dev/null 2>&1; ros2 pkg prefix turtlebot3_msgs >/dev/null && ros2 pkg prefix algae_dt >/dev/null'
check "T1 verify messages in output"                   bash -c "grep -q 'verified: turtlebot3_msgs' '$T/t1.log'"

# ============================================================================ T2: idempotent re-run
section "T2: second run on the same home (idempotency)"
sleep 1   # distinct quarantine/backup timestamps
if run_fix "$H1" "$T/t2.log"; then ok "T2 script exit 0"; else bad "T2 script exit 0"; sed 's/^/    | /' "$T/t2.log" | tail -40; fi
check "T2 ws rebuilt again"                            test -f "$WS1/install/setup.bash"
check "T2 managed block STILL present exactly once"    bash -c "[ \"\$(grep -c '^# >>> algae-dt lab_fix_workspace >>>$' '$H1/.bashrc')\" = 1 ]"
check "T2 disabled lines not double-commented"         bash -c "! grep -q '^# \[disabled by lab_fix_workspace.*# \[disabled by lab_fix_workspace' '$H1/.bashrc'"

# ============================================================================ T3: failure paths
section "T3: failure paths exit non-zero with clear messages"
H3="$T/home3"; rm -rf "$H3"; mkdir -p "$H3/Downloads"; chown -R tester:tester "$H3"
run_fix "$H3" "$T/t3a.log"; rc=$?
[ "$rc" = 3 ] && ok "T3 empty Downloads -> exit 3" || bad "T3 empty Downloads -> exit 3 (got $rc)"
check "T3 message names --zip"                         bash -c "grep -q -- '--zip' '$T/t3a.log'"

run_fix "$H3" "$T/t3b.log" --zip /nonexistent.zip; rc=$?
[ "$rc" = 3 ] && ok "T3 missing --zip file -> exit 3" || bad "T3 missing --zip file -> exit 3 (got $rc)"

printf 'not a workspace' > "$H3/notes.txt"
python3 -c "import zipfile; zipfile.ZipFile('$H3/Downloads/junk.zip','w').write('$H3/notes.txt','notes.txt')"
chown tester:tester "$H3/Downloads/junk.zip"
run_fix "$H3" "$T/t3c.log"; rc=$?
[ "$rc" = 3 ] && ok "T3 zip without package.xml -> exit 3" || bad "T3 zip without package.xml -> exit 3 (got $rc)"

run_fix "$H3" "$T/t3d.log" --zip /x.zip --from-dir /y; rc=$?
[ "$rc" = 2 ] && ok "T3 --zip + --from-dir -> exit 2" || bad "T3 --zip + --from-dir -> exit 2 (got $rc)"
run_fix "$H3" "$T/t3e.log" --domain banana; rc=$?
[ "$rc" = 2 ] && ok "T3 bad --domain -> exit 2" || bad "T3 bad --domain -> exit 2 (got $rc)"

# ============================================================================ T4: odd archive layouts
section "T4: src/ at archive root, and src CONTENTS zipped without src/"
H4="$T/home4"; rm -rf "$H4"; mkdir -p "$H4/Downloads"
FX4="$T/fx4"; rm -rf "$FX4"; mkdir -p "$FX4/wsroot"
make_pkg_py "$FX4/wsroot/src/just_one_pkg" just_one_pkg
( cd "$FX4/wsroot" && python3 -c "
import os, zipfile
with zipfile.ZipFile('$H4/Downloads/root_is_ws.zip','w') as z:
    for r, ds, fs in os.walk('src'):
        for x in ds + fs: z.write(os.path.join(r, x))
" )
chown -R tester:tester "$H4"
if run_fix "$H4" "$T/t4a.log" --no-bashrc; then ok "T4a src/ at archive root: exit 0"; else bad "T4a src/ at archive root: exit 0"; sed 's/^/    | /' "$T/t4a.log" | tail -20; fi
check "T4a package extracted + built"                  test -d "$H4/turtlebot3_ws/install/just_one_pkg"

H4b="$T/home4b"; rm -rf "$H4b"; mkdir -p "$H4b/Downloads"
( cd "$FX4/wsroot/src" && python3 -c "
import os, zipfile
with zipfile.ZipFile('$H4b/Downloads/src_contents.zip','w') as z:
    for r, ds, fs in os.walk('just_one_pkg'):
        for x in ds + fs: z.write(os.path.join(r, x))
" )
chown -R tester:tester "$H4b"
if run_fix "$H4b" "$T/t4b.log" --no-bashrc --no-build; then ok "T4b src CONTENTS zipped: exit 0"; else bad "T4b src CONTENTS zipped: exit 0"; sed 's/^/    | /' "$T/t4b.log" | tail -20; fi
check "T4b package landed under ws/src"                test -f "$H4b/turtlebot3_ws/src/just_one_pkg/package.xml"

# ============================================================================ T5: algae_dt repo refresh
section "T5: algae_dt refreshed from a repo checkout next to the script"
H5="$T/home5"; make_broken_home "$H5" "ws_for_t5"
FAKE_REPO="$T/fakerepo"; rm -rf "$FAKE_REPO"; mkdir -p "$FAKE_REPO/scripts"
cp "$SCRIPT" "$FAKE_REPO/scripts/lab_fix_workspace.sh"
make_pkg_py "$FAKE_REPO/ros2_ws/src/algae_dt" algae_dt
echo 'FRESH-REPO-COPY' > "$FAKE_REPO/ros2_ws/src/algae_dt/REPO_MARKER"
chown -R tester:tester "$FAKE_REPO"
if runuser -u tester -- env HOME="$H5" USER=tester LOGNAME=tester \
     bash "$FAKE_REPO/scripts/lab_fix_workspace.sh" --no-bashrc --no-build >"$T/t5.log" 2>&1; then
  ok "T5 script exit 0"
else bad "T5 script exit 0"; sed 's/^/    | /' "$T/t5.log" | tail -20; fi
check "T5 repo copy installed"                         test -f "$H5/turtlebot3_ws/src/algae_dt/REPO_MARKER"
check "T5 stale zip copy gone"                         bash -c "! test -e '$H5/turtlebot3_ws/src/algae_dt/ZIP_MARKER'"

H5b="$T/home5b"; make_broken_home "$H5b" "ws_for_t5b"
if runuser -u tester -- env HOME="$H5b" USER=tester LOGNAME=tester \
     bash "$FAKE_REPO/scripts/lab_fix_workspace.sh" --no-bashrc --no-build --keep-zip-algae >"$T/t5b.log" 2>&1; then
  ok "T5b --keep-zip-algae exit 0"
else bad "T5b --keep-zip-algae exit 0"; fi
check "T5b zip copy kept"                              test -f "$H5b/turtlebot3_ws/src/algae_dt/ZIP_MARKER"

# ============================================================================ T6: --from-dir salvage
section "T6: --from-dir salvage (readable) and refusal (foreign-owned src)"
H6="$T/home6"; rm -rf "$H6"; mkdir -p "$H6"
SAL="$H6/old_ws_dir"; make_pkg_py "$SAL/src/salvage_pkg" salvage_pkg
chown -R tester:tester "$H6"
if run_fix "$H6" "$T/t6a.log" --from-dir "$SAL" --no-bashrc --no-build; then ok "T6a salvage exit 0"; else bad "T6a salvage exit 0"; sed 's/^/    | /' "$T/t6a.log" | tail -20; fi
check "T6a salvaged package present"                   test -f "$H6/turtlebot3_ws/src/salvage_pkg/package.xml"

H6b="$T/home6b"; rm -rf "$H6b"; mkdir -p "$H6b"
SAL2="$H6b/old_ws_dir"; make_pkg_py "$SAL2/src/salvage_pkg" salvage_pkg
echo secret > "$SAL2/src/salvage_pkg/unreadable"
chown -R tester:tester "$H6b"
chown "$FOREIGN_UID:$FOREIGN_UID" "$SAL2/src/salvage_pkg/unreadable"
chmod 600 "$SAL2/src/salvage_pkg/unreadable"
run_fix "$H6b" "$T/t6b.log" --from-dir "$SAL2" --no-bashrc --no-build; rc=$?
[ "$rc" = 3 ] && ok "T6b foreign-owned src file -> exit 3 (points at the zip)" || bad "T6b foreign-owned src file -> exit 3 (got $rc)"
check "T6b message recommends the zip"                 bash -c "grep -qi 'zip' '$T/t6b.log'"

# ============================================================================ T7: tar.gz support
section "T7: tar.gz archive support"
H7="$T/home7"; rm -rf "$H7"; mkdir -p "$H7/Downloads"
FX7="$T/fx7"; rm -rf "$FX7"; make_ws_tree "$FX7" "tar_ws"
tar -C "$FX7" -czf "$H7/Downloads/ws_failsafe.tar.gz" "tar_ws"
chmod -R u+rwX "$FX7" && rm -rf "$FX7"
chown -R tester:tester "$H7"
if run_fix "$H7" "$T/t7.log" --no-bashrc --no-build; then ok "T7 tar.gz exit 0"; else bad "T7 tar.gz exit 0"; sed 's/^/    | /' "$T/t7.log" | tail -20; fi
check "T7 packages extracted from tar.gz"              test -f "$H7/turtlebot3_ws/src/turtlebot3_msgs/package.xml"
check "T7 poison skipped from tar.gz"                  bash -c "! find '$H7/turtlebot3_ws' -name local_setup.dsv | grep -q ."

# ============================================================================ T8: unzip-less fallback
section "T8: python fallback when unzip is broken/absent"
H8="$T/home8"; make_broken_home "$H8" "ws_for_t8"
STUB="$T/stub"; mkdir -p "$STUB"
printf '#!/bin/sh\nexit 127\n' > "$STUB/unzip"; chmod 755 "$STUB/unzip"
if runuser -u tester -- env HOME="$H8" USER=tester LOGNAME=tester PATH="$STUB:/usr/local/bin:/usr/bin:/bin" \
     bash "$SCRIPT" --no-bashrc --no-build >"$T/t8.log" 2>&1; then
  ok "T8 exit 0 with broken unzip"
else bad "T8 exit 0 with broken unzip"; sed 's/^/    | /' "$T/t8.log" | tail -20; fi
check "T8 packages extracted via python fallback"      test -f "$H8/turtlebot3_ws/src/turtlebot3_msgs/package.xml"
check "T8 poison still skipped"                        bash -c "! find '$H8/turtlebot3_ws' -name local_setup.dsv | grep -q ."
check "T8 mode-000 dir still recovered"                runuser -u tester -- cat "$H8/turtlebot3_ws/src/turtlebot3_msgs/locked_dir/file.txt"

# ============================================================================ summary
printf '\n==================== RESULT: %d passed, %d failed ====================\n' "$PASS" "$FAIL"
[ "$FAIL" = 0 ]
