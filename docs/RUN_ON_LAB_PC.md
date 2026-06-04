# RUN_ON_LAB_PC.md — run the lab demo (real Burger ⇄ Gazebo twin)

Repo: **github.com/aleksander-kolev/algae-dt**.

**How the lab PC actually runs this (verified on the machine, 2026-06): NATIVELY — the lab laptop
has NO Docker.** The host has ROS 2 Jazzy at `/opt/ros/jazzy` and the full turtlebot3 stack **built
from source** in `~/turtlebot3_ws/src` (`turtlebot3`, `turtlebot3_msgs`, `DynamixelSDK`,
`turtlebot3_simulations`); our `algae_dt` builds on top in the same workspace. There is no sudo on
the laptop; nothing below needs it.

`scripts/lab_run.sh` is the runner. It auto-detects the runtime — on the lab PC that is **native**
(`--native` forces it; on machines that do have Docker, e.g. at home, it uses the container). It
does **only `both`** (full real+sim demo) by default and **errors** rather than start a half-demo;
`--sim` is the hardware-free fallback (a valid graded demo).

## 0. Workspace sanity (first thing, every session — 10 s)
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
ros2 pkg list | grep -E 'turtlebot3|nav2_simple_commander'
```
Expect `turtlebot3_gazebo/_bringup/_navigation2/_teleop` + `nav2_simple_commander`. If that fails —
or a build errors with a foreign path (`CMakeCache.txt directory … /home/test/turtlebot3_ws`),
`PermissionError: [Errno 13]` on `install/**/local_setup.dsv`, or the tree sits at
`~/turtlebot3_ws (Copy)` — the workspace was copied from another machine/user and is unsalvageable
in place. **Recover it (no sudo, no Docker; the fail-safe zip lives in `~/Downloads`):**
```bash
./scripts/lab_fix_workspace.sh
```
It extracts ONLY `src/` from the zip (`build/install` are machine-specific poison), renames every
`~/turtlebot3_ws*` dir to `turtlebot3_ws.broken.<ts>` (your local experiments are KEPT; rename
needs no sudo even on foreign-owned files), normalizes permissions, repairs `~/.bashrc` (stale
lines commented; one managed block sets `ROS_DOMAIN_ID=36`, `TURTLEBOT3_MODEL=burger`,
`LDS_MODEL=LDS-02`; backups kept), clean-rebuilds the whole workspace in a sanitized environment,
and verifies the packages resolve. **Then open a NEW terminal** (old ones carry the broken env).
Options: `--zip PATH` · `--from-dir DIR` · `--domain N` · `--no-bashrc` · `--no-build` ·
`--keep-zip-algae` · `--purge-quarantine`. Run from a cloned algae-dt repo it also refreshes
`src/algae_dt` to the repo's copy. Tested end-to-end by `scripts/test_lab_fix_workspace.sh`.

## Start the demo — the script way (normal)
```bash
# 1) on the lab laptop: clone (once)
git clone https://github.com/aleksander-kolev/algae-dt.git && cd algae-dt

# 2) on the ROBOT Pi (separate ssh — the script can't do this):
ssh turtlebot@192.168.8.36
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36
ros2 launch turtlebot3_bringup robot.launch.py          # leave running

# 3) on the laptop: one command does the rest
./scripts/lab_run.sh
```
Step 3 self-heals: picks the runtime (native on the lab PC), copies `algae_dt` into
`~/turtlebot3_ws/src`, **verifies it sees the robot's `/scan`**, builds, then launches the twin +
GUI **+ RViz** (the launch opens RViz automatically in `both`/`real_only` — required for the 2D
Pose Estimate). Then do the **2D Pose Estimate** (below). The script prints the exact teleop line
for a second terminal. Weak/odd GPU? `./scripts/lab_run.sh --no-gz-gui` skips the gz 3D window
(RViz + operator console still show everything; a gz 3D crash no longer kills the launch either).

## Start the demo — fully manual (no script)
```bash
# laptop, once: put the package where the workspace expects it
cp -r algae-dt/ros2_ws/src/algae_dt ~/turtlebot3_ws/src/

# Terminal A — build + launch (native; no docker anywhere):
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
cd ~/turtlebot3_ws && colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=36
ros2 topic hz /scan                          # robot link OK? (~5 Hz) — robot Pi must be up
ros2 launch algae_dt bringup.launch.py mode:=both

# more terminals: re-run the two source lines + the exports (incl. ROS_DOMAIN_ID=36 — else zero
#   topics cross on the shared AP2IRR10 Wi-Fi)
#   teleop:  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw
```
(Robot Pi bringup = step 2 above, same as the script way.)

## 2D Pose Estimate — the one manual click (do it every run)
Nav2's **AMCL** starts NOT knowing where the robot is (it seeds at the map origin). Five robots share
the arena, so the real start never matches the origin — you must tell AMCL the true pose once:
1. In **RViz** (opens automatically in `both`/`real_only`; force with `use_rviz:=true`) click
   **“2D Pose Estimate”** in the top toolbar.
2. **Click on the map at the robot's actual position**, and **hold + drag** in the direction the
   robot is **physically facing**, then release. The green arrow = (position, heading).
3. **Verify it took:** the LiDAR points (red) should snap onto the map walls and the costmap aligns
   with the robot. If they're off, just repeat — do it until the scan sits on the walls.
4. **Only then** place blooms / press Start. Do this **before** any mission, and **again after any
   AMCL/Nav2 restart** (a restart re-seeds to the origin). Skipping it → Nav2 plans from a wrong pose
   → the robot drives wrong or skips goals.

## Prerequisites (the script errors clearly if any fail)
- **A healthy native workspace** — `turtlebot3_gazebo` resolvable from `/opt/ros/jazzy` +
  `~/turtlebot3_ws/install` (§0; broken → `./scripts/lab_fix_workspace.sh`).
- **Robot reachable + publishing `/scan`** (laptop + robot on Wi-Fi `AP2IRR10`, same
  `ROS_DOMAIN_ID=36` — the robot number on the sticker, set on BOTH sides).
- Overrides: `ROS_DOMAIN_ID=` / `ROBOT_IP=`. Hardware-free at home:
  `ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true`. Shut the robot down with
  `ssh turtlebot@192.168.8.36 'sudo shutdown now'` before the power switch.

> Sources: the sourcing/build/launch commands are verbatim from the Canvas pages ("How to run
> Gazebo", "Changing Robot Inflation", "DT Example", "Connecting lab laptop to robot");
> `docs/DECISIONS.md` D1 records how the lab runtime was verified.
