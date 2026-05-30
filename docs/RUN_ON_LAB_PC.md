# RUN_ON_LAB_PC.md — run the lab demo (real Burger ⇄ Gazebo twin)

Repo: **github.com/aleksander-kolev/algae-dt**. Everything runs inside the course `turtlebot3_ws`
Docker container (turtlebot3 is in the image, not the bare host — `docs/DECISIONS.md` D1).
The lab runner does **only `both`** (full real+sim demo) and **errors** if `both` can't run.

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
Step 3 self-heals (builds the image if missing, recreates `~/turtlebot3_ws`, builds the package),
**verifies it sees the robot's `/scan`**, then launches the twin + GUI. Then do the **2D Pose
Estimate** (below). Teleop in another terminal: the script prints the exact `docker exec …` line.

## Start the demo — fully manual (no script)
```bash
# laptop, once: put the package where the container mounts it
mkdir -p ~/turtlebot3_ws/src && cp -r algae-dt/ros2_ws/src/algae_dt ~/turtlebot3_ws/src/

# start the container (Terminal A)
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/turtlebot3_ws:/ws --user $(id -u):$(id -g) turtlebot3_ws bash
# inside Terminal A:
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null
colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=36
ros2 topic hz /scan                          # robot link OK? (~5 Hz) — robot Pi must be up
ros2 launch algae_dt bringup.launch.py mode:=both

# more terminals: docker exec -it turtlebot3_container bash  (then source the 3 lines above)
#   teleop:  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw
```
(Robot Pi bringup = step 2 above, same as the script way.)

## 2D Pose Estimate — the one manual click (do it every run)
Nav2's **AMCL** starts NOT knowing where the robot is (it seeds at the map origin). Five robots share
the arena, so the real start never matches the origin — you must tell AMCL the true pose once:
1. In **RViz** (opens with the stack) click **“2D Pose Estimate”** in the top toolbar.
2. **Click on the map at the robot's actual position**, and **hold + drag** in the direction the
   robot is **physically facing**, then release. The green arrow = (position, heading).
3. **Verify it took:** the LiDAR points (red) should snap onto the map walls and the costmap aligns
   with the robot. If they're off, just repeat — do it until the scan sits on the walls.
4. **Only then** place blooms / press Start. Do this **before** any mission, and **again after any
   AMCL/Nav2 restart** (a restart re-seeds to the origin). Skipping it → Nav2 plans from a wrong pose
   → the robot drives wrong or skips goals.

## Prerequisites (the script errors clearly if any fail)
- **Rootful Docker** — `both` reaches the robot over DDS via real `--net=host`; **rootless can't**
  (no LAN multicast). No-sudo can't install rootful → a TA must enable it.
- **A turtlebot3 image** (else it builds one from `scripts/Dockerfile`; needs internet + ~10–20 min).
- **Robot reachable + publishing `/scan`** (laptop + robot on Wi-Fi `AP2IRR10`, same `ROS_DOMAIN_ID=36`).
- Overrides: `ROS_DOMAIN_ID=` / `ROBOT_IP=` / `TB3_IMAGE=`. Hardware-free at home:
  `ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true`. Shut the robot down with
  `ssh turtlebot@192.168.8.36 'sudo shutdown now'` before the power switch.

> Sources: the `docker run`/build/launch commands are verbatim from the Canvas pages ("How to run
> Gazebo", "Changing Robot Inflation", "DT Example", "Connecting lab laptop to robot"); see
> `docs/DECISIONS.md` for why turtlebot3 lives in the image.
