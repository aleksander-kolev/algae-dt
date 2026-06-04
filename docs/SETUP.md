# SETUP.md — environments, build, robot connection (course-faithful)

Three environments. **Develop at home; test in the lab.** Reading order matches the Canvas pages:
*"Software Setup and Usage"*, *"Setting up your workspace in WSL"*, *"How to create Packages"*,
*"Connecting lab laptop to lab robot"*, *"Transferring files"*.

---

## 0. One-time, before the first lab session (Canvas "must-read" PDFs)
- Follow **"WSL+Docker Setup and Installation.pdf"** → WSL2 + Docker Desktop + the `turtlebot3_ws`
  image. (Or VM via "VirtualBox+VM Installation.pdf" — slower; WSL+Docker is strongly recommended.)
- Follow **"Setting up your workspace in WSL.pdf"** → creates `~/turtlebot3_ws` and the workspace.
- Read **"Connecting lab laptop to lab robot.pdf"** and **"Transferring files.pdf"**.
- Confirm you can run the provided container and `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`.

---

## 1. HOME — WSL + Docker (primary development environment)
Place this repo's package at the workspace path the course uses:
```
~/turtlebot3_ws/src/algae_dt        # (= this repo's ros2_ws/src/algae_dt)
```
Start the course container — `--net=host` shares the network namespace (needed for ROS 2 DDS) and
the X11 mounts let Gazebo/RViz/the PyQt5 GUI render via WSLg. **Note:** at home there is no robot —
home is `sim_only`. Real-robot connectivity is validated on the **lab laptop** (§3), where the stack
and the robot share `AP2IRR10` + `ROS_DOMAIN_ID`. (On Docker-Desktop/WSL2, `--net=host` joins the WSL
VM's network, not the Windows LAN; that's irrelevant at home and a non-issue on the native lab laptop.)
```bash
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/turtlebot3_ws:/ws \
  --user $(id -u):$(id -g) turtlebot3_ws bash
```
Inside the container:
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```
Attach more terminals to the *same* container with: `docker exec -it turtlebot3_container bash`.

**PyQt5 in the container:** if `import PyQt5` fails, `pip install --user PyQt5` inside the container
(no sudo). GUI needs the X11 mounts above (WSLg provides the display on Windows 11).

**Gazebo hygiene (course GazeboTutorial):** if a sim wedges, `pkill -9 -f "gz sim"` before relaunch.

### 1b. Alternative: VirtualBox VM (slower; only if WSL+Docker is unavailable)
Per "VirtualBox+VM Installation.pdf" + "Running the VM.pdf": import the provided Ubuntu VM, then the
workspace lives natively at `~/turtlebot3_ws/src/algae_dt` (no `docker run`; use the native build
commands from §2). **Mac users:** "MAC OS users.pdf" — VM only on Intel Macs; less-tested. The team
is on Windows → use WSL+Docker (§1). The VM table from the course: WSL+Docker is faster, handles many
nodes, stable in long sessions — "strongly recommended for 2IRR10".

### 1c. Asset provenance (so the package is self-contained)
- `worlds/algae_arena.world` = the course **`new_world.world`** (from `Simulation Files.zip`) — the
  digital model of the lab arena. Edit object sizes via the `<collision>`/`<visual>` `<size>` pairs
  (GazeboTutorial / "Changing Robot Inflation.pdf").
- `maps/map.{pgm,yaml}` = the course **`mapFiles.zip`** base map (res 0.05, origin [-2.051,-4.194]).
  You may set the robot start in `map.yaml` or via the Gazebo Tutorial to match the physical start.

---

## 2. LAB — the HP Z-Book (TESTING ONLY)
**The lab PC runs everything NATIVELY — it has NO Docker (verified on the machine, 2026-06).**
ROS 2 Jazzy lives at `/opt/ros/jazzy` and the turtlebot3 stack is **built from source** in
`~/turtlebot3_ws/src` (`turtlebot3`, `turtlebot3_msgs`, `DynamixelSDK`, `turtlebot3_simulations`);
`algae_dt` builds on top in the same workspace. Use the one script: `./scripts/lab_run.sh` (full
`both` demo) or `./scripts/lab_run.sh --sim` (hardware-free fallback) — it auto-detects the
runtime and on the lab PC lands on native (`--native` forces). See `docs/RUN_ON_LAB_PC.md`.

**First thing, every session — verify the workspace (10 s):**
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
ros2 pkg list | grep turtlebot3        # expect turtlebot3_gazebo, _bringup, _navigation2, _teleop
ros2 pkg list | grep nav2_simple_commander   # mission_runner's BasicNavigator import depends on it
```
If `nav2_simple_commander` is missing (rare), it's the one runtime dep to flag to a TA per the
no-sudo process (RULES §A-3). Run the same two `ros2 pkg list` checks at home in the container.
If the turtlebot3 packages don't resolve, the workspace is broken — recover it with
`./scripts/lab_fix_workspace.sh` (below), don't hand-patch it.

Get the package onto the laptop (course "How to create Packages.pdf", **Section 3 = copy full package**):
```bash
# transfer ros2_ws/src/algae_dt via USB / OneDrive / Google Drive, then:
cp -r /path/to/algae_dt ~/turtlebot3_ws/src/
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```
**Back up before you leave** — the laptop can be factory-reset without notice: push to git +
OneDrive every session.

If you ever see `failed to create symbolic link … existing path cannot be removed`, the workspace is
dirty — clean rebuild (course Lidar-DT "Common Problems"):
```bash
cd ~/turtlebot3_ws && rm -rf build/ install/ log/ && colcon build --symlink-install && source install/setup.bash
```

If the workspace was **copied from another machine/user** (CMake errors naming a foreign path like
`/home/test/turtlebot3_ws`, `PermissionError` on `install/**/local_setup.dsv`, a
`turtlebot3_ws (Copy)` folder), a clean rebuild in place is NOT enough — the copied tree has the old
absolute paths baked in and files you don't own. Recover with **`./scripts/lab_fix_workspace.sh`**
(no sudo, no Docker: extracts only `src/` from the fail-safe zip in `~/Downloads`, quarantines the
broken dirs, repairs `~/.bashrc`, clean-rebuilds natively, verifies). Details:
`docs/RUN_ON_LAB_PC.md` §"Broken/copied workspace".

---

## 3. Connecting to the real robot (course "Connecting lab laptop to lab robot.pdf")
Laptop and robot must be on Wi-Fi **`AP2IRR10`** and share **`ROS_DOMAIN_ID = robot number`**.
```bash
# Terminal A — bring up the robot on its Pi (IP + number are on stickers on the robot):
ssh turtlebot@<ROBOT_IP>            # e.g. ssh turtlebot@192.168.8.36
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=<ROBOT_NUMBER>
ros2 launch turtlebot3_bringup robot.launch.py     # leave running

# Terminal B — on the laptop, confirm topics arrive BEFORE launching our stack:
export ROS_DOMAIN_ID=<ROBOT_NUMBER>
source ~/turtlebot3_ws/install/setup.bash
ros2 topic list            # expect /scan /odom /cmd_vel /tf /battery_state
ros2 topic hz /scan        # ~5 Hz on the LDS-02
ros2 topic type /cmd_vel   # geometry_msgs/msg/TwistStamped
```
If `/scan` is empty: wrong `ROS_DOMAIN_ID`, wrong Wi-Fi, or the robot Pi isn't fully up.

**Real / both mode needs an AMCL initial pose:** in RViz click **2D Pose Estimate** and click-drag on
the robot's actual spot/heading (five robots share the arena, so the start differs from the map
origin). Re-do it after any AMCL/Nav2 restart. `sim_only` needs none of this.

**Shutdown:** on the robot Pi `sudo shutdown now` **before** flipping the power switch (avoid SD-card
corruption). `Ctrl-C` the laptop launches.

**Arena rules (Golden Rules):** robot moves only inside the wooden arena; one person in the arena at
a time; if testing at a table, prop the wheels off the surface.

---

## 4. Launch recipes (what runs in each mode)
**Every terminal that launches our stack in `real_only`/`both` must first**
`export ROS_DOMAIN_ID=<ROBOT_NUMBER>` (same as the robot, §3) — else zero topics cross, and on the
shared `AP2IRR10` an unset/default domain risks cross-team topic bleed. `bringup.launch.py`
orchestrates the stock packages + our `algae_dt` layer:
- **`mode:=sim_only`** → `turtlebot3_gazebo` (our `algae_arena.world`) + `turtlebot3_navigation2`
  (Nav2/AMCL, `use_sim_time:=true`) + `twin_mediator` + `sync_supervisor` + `mission_runner` +
  `operator_gui`. No robot needed. **This is the home dev target.**
- **`mode:=real_only`** → robot bringup (on the Pi, §3) + `turtlebot3_navigation2` on the laptop
  (`use_sim_time:=false`) + our DT layer. 2D Pose Estimate required.
- **`mode:=both`** → real leads + `turtlebot3_gazebo` namespaced to `/sim/*` mirroring 1:1 + our DT
  layer (wall time). Topic-collision rule enforced (sim on `/sim/*`, sim TF off global `/tf`).

The course's manual multi-terminal style (separate `ros2 launch` per component) is documented in
`docs/BEST_APPROACHES.md` as the TA-familiar fallback if a combined launch misbehaves in the lab.
