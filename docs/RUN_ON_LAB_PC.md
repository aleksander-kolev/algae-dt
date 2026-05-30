# RUN_ON_LAB_PC.md — pull the repo on the lab laptop & run with the real robot

Repo: **https://github.com/aleksander-kolev/algae-dt** (private; default branch `main`).
Flow: **verify at home → pull on the lab laptop → build → connect robot → run.**

---

## 0. Readiness verdict — READ THIS FIRST (honest, as of the current commit)
| Component | Status | Note |
|---|---|---|
| Pure logic (safety/blooms/sync/metrics/geometry/pgm) | **VERIFIED** | 82 unit tests pass. |
| Nodes + launch (structure) | **COMPLETE but UNVERIFIED in ROS/Gazebo** | Never run with `colcon`/`gz` yet — only unit-tested. |
| `sim_only` end-to-end | **NEEDS A HOME SMOKE-TEST** | **Risk:** the launch starts `gz_sim` + spawn + robot_state_publisher but **no explicit `ros_gz` `parameter_bridge`** — if `/scan`,`/odom`,`/cmd_vel`,`/clock` don't appear in ROS, add a bridge (or use turtlebot3_gazebo's world launch, which bundles it). |
| `real_only` | **Should work after smoke-test** | Verify Nav2's final velocity reaches `/dt/cmd_vel_raw` (the launch rewrites `collision_monitor.cmd_vel_out_topic`; confirm that key exists in turtlebot3_navigation2 `param/burger.yaml`). |
| `both` (real↔sim mirror) | **NOT READY** | The launch spawns the sim on **bare** topics, but the mediator expects the mirror sim on `/sim/*`. The sim is not namespaced (PLAN T5.1). Finish that before relying on `both`. |

**Bottom line:** the build + unit tests are green, but **the integrated launch has not been run in a
real ROS/Gazebo environment.** Do **NOT** assume "pull and it runs." Smoke-test in the Docker
container at home and fix the two flagged items first (§1). The lab is for testing a *verified* build.

---

## 1. VERIFY AT HOME FIRST (mandatory — you have the Docker image)
Do this in WSL+Docker (or the dev image `docker/Dockerfile`) BEFORE a lab session:
```bash
git clone https://github.com/aleksander-kolev/algae-dt.git
mkdir -p ~/turtlebot3_ws/src && cp -r algae-dt/ros2_ws/src/algae_dt ~/turtlebot3_ws/src/
# in the turtlebot3_ws container:
cd ~/turtlebot3_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt && source install/setup.bash
python3 -m pytest src/algae_dt/test -q                       # expect all green
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only headless:=true   # watch for errors
# In another shell: confirm the sim is actually publishing to ROS:
ros2 topic hz /scan ; ros2 topic echo /clock --once ; ros2 topic list | grep -E "/scan|/odom|/cmd_vel"
```
- If `/scan`/`/clock` are **missing** → add the `ros_gz` bridge (the §0 sim risk). Fix, commit, push.
- Then test a bloom mission in the GUI (`mode:=sim_only`, not headless) and E-STOP.
- **Commit + push any fix so the lab laptop pulls a verified build.** Re-record the `sim_only` baseline
  video here too (the backup, per `docs/DEMO_SCRIPT.md`).

## 2. Pull on the lab laptop
The laptop is shared + wiped, with no sudo. Pick one clone method:
- **Easiest (password-free):** make the repo public for the session
  (`gh repo edit aleksander-kolev/algae-dt --visibility public`), `git clone https://…/algae-dt.git`,
  then set it back to `--visibility private` after. (Safe — `CREDENTIALS.txt`/videos are git-ignored.)
- **Or** keep it private and use a fine-grained **PAT** over HTTPS, or `gh auth login` (device flow).
- **Backup:** the USB you must already carry (RULES §A-6) with the package.
```bash
git clone https://github.com/aleksander-kolev/algae-dt.git
cp -r algae-dt/ros2_ws/src/algae_dt ~/turtlebot3_ws/src/
```

## 3. Build on the lab laptop
```bash
cd ~/turtlebot3_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=<robot#>
ros2 pkg list | grep -E "turtlebot3|nav2_simple_commander"     # sanity
```
(Dirty-symlink error → `rm -rf build/ install/ log/` then rebuild. Lab = testing: build only our pkg.)

## 4. Connect the real robot (full detail: `docs/SETUP.md` §3)
```bash
# robot Pi:  ssh turtlebot@<ip> ; export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=<robot#>
#            ros2 launch turtlebot3_bringup robot.launch.py
# laptop:    ros2 topic hz /scan (~5 Hz) ; ros2 topic type /cmd_vel  -> TwistStamped
```

## 5. Run + demo (real robot)
```bash
ros2 launch algae_dt bringup.launch.py mode:=real_only use_rviz:=true   # safest single-robot path
# (only after §0 `both` namespacing is finished:)
ros2 launch algae_dt bringup.launch.py mode:=both use_rviz:=true
```
- In RViz set **2D Pose Estimate** on the robot's real spot; wait for AMCL lock **before** Start.
- Drive scenarios from `docs/SCENARIOS_LAB.md`; record per `docs/DEMO_SCRIPT.md` (the 3 rubric usages).

## 6. Fallback — manual multi-terminal (course-proven, if the combined launch misbehaves)
```
T1: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=~/turtlebot3_ws/src/algae_dt/maps/map.yaml
T2: ros2 run algae_dt twin_mediator   ; T3: ros2 run algae_dt sync_supervisor
T4: ros2 run algae_dt mission_runner  ; T5: ros2 run algae_dt operator_gui
T6: ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw
```

## 7. Shutdown + backup
`ssh turtlebot@<ip> 'sudo shutdown now'` BEFORE the power switch. Commit/push + copy to USB/OneDrive
before leaving (the laptop may be wiped).
