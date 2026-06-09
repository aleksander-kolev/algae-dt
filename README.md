# algae-dt — TurtleBot3 Burger digital twin

> New to the codebase? **[`CODE_EXPLAINED.md`](CODE_EXPLAINED.md)** explains the whole project —
> concepts, architecture, every node and library — in plain language, self-contained.

A bidirectional digital twin for a TU/e 2IRR10 algae-bloom cleaning robot. A real TurtleBot3
Burger and a Gazebo Harmonic twin run in parallel with state synchronisation between them. An
operator places algae blooms on a map; the active robot navigates to each one with Nav2 (avoiding
static and dynamic obstacles), drives to the centre and spins three full turns in place
("spraying"), then moves on. A 25 cm dual-LiDAR safety gate stops forward motion in either world.

It is built the way the course recommends: ROS 2 **Jazzy** + the stock **turtlebot3** packages
(`turtlebot3_gazebo`, `turtlebot3_navigation2`, `turtlebot3_bringup`, `turtlebot3_teleop`) with one
custom package, `algae_dt`, holding the digital-twin logic on top. We don't re-implement anything
the stock stack already provides.

There are three run modes:

| mode | what runs | needs a robot? |
|------|-----------|----------------|
| `sim_only` *(default)* | Gazebo robot + Nav2 + the DT layer. **This is the home/dev target.** | no |
| `real_only` | the real Burger (bringup on its Pi) + Nav2 on the laptop + the DT layer | yes |
| `both` | real Burger leads, a Gazebo twin mirrors it 1:1 on `/sim/*` | yes |

`both` can also be run hardware-free at home with a kinematic stand-in (`use_fake_robot:=true`).

---

## Repository layout

```
algae-dt/
├─ ros2_ws/src/algae_dt/          the ROS 2 package (build this with colcon)
│  ├─ algae_dt/
│  │  ├─ twin_mediator.py         command chokepoint + 25 cm dual-LiDAR safety gate; mirrors
│  │  │                           pose/scan/battery onto /dt/*; owns /dt/estop and /dt/mode
│  │  ├─ sync_supervisor.py       measures real-vs-sim discrepancy (pose, sensor, command->motion
│  │  │                           latency, stop skew); publishes /dt/sync_*; logs a CSV + alerts
│  │  ├─ mission_runner.py        Nav2 goal to each bloom (projected clear of walls), then a
│  │  │                           closed-loop 3-revolution spin-spray; publishes markers + state
│  │  ├─ operator_gui.py          PyQt5 console: map canvas, real/sim pose + live scan overlay,
│  │  │                           click-to-place blooms, Start/Stop/Clear/E-STOP, status banners
│  │  ├─ fake_robot.py            kinematic stand-in for the real robot (run real_only/both at home)
│  │  ├─ dynamic_obstacle.py      sweeps a box across the path for the "environment change" demo
│  │  └─ lib/                     pure helpers, no ROS imports (the unit-testable building blocks)
│  │     ├─ geometry.py           world <-> map-pixel transforms, angle math
│  │     ├─ safety.py             the dual-LiDAR front-stop gate (fail-safe)
│  │     ├─ blooms.py             immutable bloom field + state transitions
│  │     ├─ sync.py               pose/sensor discrepancy + unicycle integrator
│  │     ├─ metrics.py            latency calc + sync-CSV row formatting
│  │     ├─ hud.py                battery/scan formatting for the console
│  │     ├─ pgm.py                P2/P5 PGM map parser (for the GUI canvas)
│  │     ├─ occupancy.py          static-map occupancy + goal projection off walls
│  │     └─ trajectory.py         sinusoidal path for the moving obstacle
│  ├─ launch/bringup.launch.py    orchestrates the stock stack + Nav2 + our layer for each mode
│  ├─ config/twin.yaml            all tunables (safety, sync tolerances, spray, battery, map dims)
│  ├─ config/sim_bridge.yaml      ros_gz bridge that puts the mirror sim on /sim/* (both mode)
│  ├─ maps/map.pgm, map.yaml      arena occupancy map (Nav2/AMCL + the GUI canvas)
│  ├─ worlds/algae_arena.world    Gazebo model of the lab arena
│  ├─ worlds/obstacle_box.sdf     a spawnable box obstacle
│  └─ package.xml, setup.py, setup.cfg, resource/
├─ docker/                        dev image + helpers (home / WSL2)
│  ├─ Dockerfile                  ROS 2 Jazzy + turtlebot3 stack + Nav2 + Gazebo Harmonic + PyQt5
│  ├─ build.sh / build.ps1        build the image (tag: algae-dt:dev)
│  ├─ run.sh / run.ps1            start the container (run.sh wires up the X11 display on WSL)
│  ├─ demo_run.sh                 one-shot inside the container: build + launch sim_only with RViz
│  └─ open_sim.sh                 sim_only with the Gazebo 3D window off (WSL GL workaround) + RViz
├─ scripts/
│  ├─ lab_run.sh                  the lab-laptop runner: full real+sim (both), or --sim fallback;
│  │                              auto-detects Docker vs the lab PC's native (no-Docker) stack
│  ├─ lab_fix_workspace.sh        rebuild a broken/copied ~/turtlebot3_ws from a zip (no sudo)
│  ├─ test_lab_fix_workspace.sh   container test suite for the recovery script (57 assertions)
│  └─ Dockerfile                  fallback image the lab runner builds if none is present
├─ .gitattributes, .gitignore
```

---

## Prerequisites

- **WSL2 + Docker Desktop** on Windows 11 (with WSLg, which provides the X display for Gazebo / RViz
  / the PyQt5 GUI). Docker on native Linux works the same way.
- About **10–20 minutes and internet** for the first image build (it pulls ROS 2 Jazzy and the
  turtlebot3 stack). After that the image is cached.

You do **not** need ROS installed on the host; everything runs inside the container.

---

## Python Dependencies

Install the required packages to run 'bloom_predictor.py'

  py -m pip install -r requirements.txt

## Run the simulation (WSL2 + Docker)

Do this from a WSL terminal, in the repo root.

**1. Build the dev image** (once, or after changing `docker/Dockerfile`):

```bash
bash docker/build.sh                 # WSL / Linux
# Windows PowerShell:  docker\build.ps1
# either way this is just:  docker build -t algae-dt:dev docker
```

**2. Start the container** (mounts `ros2_ws` at `/ws` and wires up the display):

```bash
bash docker/run.sh                   # drops you into a shell at /ws
```

**3. Build the package and launch sim_only** (inside the container):

```bash
cd /ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```

Gazebo, RViz, and the operator console come up. Click on the map in the console to place blooms,
then press **Start**. Attach another terminal to the same container with
`docker exec -it algae_dt_container bash`.

### One-shot helpers

If you'd rather not type the build/launch by hand, the two scripts in `docker/` do it for you. Run
them from the repo root; they mount the workspace and the scripts dir, then build and launch inside
the container:

```bash
# full GUI (Gazebo 3D window + RViz + console):
docker run --rm -it --net=host -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/demo_run.sh

# same, but with the Gazebo 3D window OFF (use this if Gazebo's window crashes on WSL — the
# server still renders the LiDAR, you drive and watch via RViz + the console):
docker run -d --name algae_dt_sim --net=host -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/open_sim.sh
docker logs -f algae_dt_sim          # watch it;  docker rm -f algae_dt_sim  to stop
```

### Headless (no display)

For a quick check on a machine with no display (CI, a server), run the server-only Gazebo with no
GUI/RViz:

```bash
ros2 launch algae_dt bringup.launch.py mode:=sim_only headless:=true
```

---

## Modes and launch options

```bash
ros2 launch algae_dt bringup.launch.py mode:=<sim_only|real_only|both> [options]
```

| argument | default | meaning |
|----------|---------|---------|
| `mode` | `sim_only` | which world(s) to bring up (see the table at the top) |
| `headless` | `false` | Gazebo server only, no GUI/RViz (for CI / no display) |
| `use_rviz` | `auto` | `auto` opens RViz in `real_only`/`both` (the **2D Pose Estimate** that seeds AMCL lives there — required) and keeps it off in `sim_only`; force with `true`/`false` |
| `use_fake_robot` | `false` | in `real_only`/`both`, spawn the kinematic `fake_robot` so you can run those modes at home with no hardware |
| `gz_gui` | `true` | in `sim_only`/`both`, run Gazebo's 3D client. Set `gz_gui:=false` if that window crashes on WSL/weak GL — the server still renders the LiDAR (and a 3D-client crash no longer takes the launch down). |
| `use_dynamic_obstacle` | `false` | in `sim_only`/`both`, sweep a box across the robot's path (the scripted "live environment change") |

Run `both` at home, no robot required:

```bash
ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true
```

---

## Run on the lab laptop (real robot)

The lab HP Z-Book runs everything **natively — it has no Docker**: ROS 2 Jazzy at `/opt/ros/jazzy`
with the turtlebot3 stack built from source in `~/turtlebot3_ws`. `scripts/lab_run.sh` does the
whole `both` demo (real Burger + Gazebo mirror) and fails clearly if it can't — it auto-detects the
runtime (native on the lab PC; on a Docker-capable machine it uses the container instead, where
`both` needs *rootful* Docker for the robot's DDS traffic), and it requires the robot reachable and
publishing `/scan`.

If the laptop's `~/turtlebot3_ws` is broken — builds failing with another machine's paths in
`CMakeCache.txt`, `PermissionError` on `install/**/local_setup.dsv`, a `turtlebot3_ws (Copy)`
folder — recover it first (no sudo needed; uses the workspace zip in `~/Downloads`):

```bash
./scripts/lab_fix_workspace.sh
```

The robot we used last time: **#36 at `192.168.8.36`, `ROS_DOMAIN_ID=36`**, on Wi-Fi `AP2IRR10`,
LiDAR LDS-02. The IP and number are on stickers on each robot; set `ROS_DOMAIN_ID` to the robot
number on **both** sides.

```bash
# 1) on the laptop: clone (once)
git clone https://github.com/aleksander-kolev/algae-dt.git && cd algae-dt

# 2) on the ROBOT's Pi, in a separate ssh session (the script can't do this part):
ssh turtlebot@192.168.8.36
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36
ros2 launch turtlebot3_bringup robot.launch.py        # leave this running

# 3) back on the laptop — one command builds, checks the robot link, and launches the twin:
./scripts/lab_run.sh                 # full real+sim demo
./scripts/lab_run.sh --sim           # hardware-free sim_only fallback (no robot needed)
./scripts/lab_run.sh --rebuild       # clean colcon build first
```

Overrides if your robot differs: `ROBOT_IP=… ROS_DOMAIN_ID=… ./scripts/lab_run.sh`. Force a
runtime with `--native` / `--docker`.

Teleop in another terminal (the script prints the exact line for the runtime it picked, remapped
onto the safety bus — native version shown):

```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=36
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw
```

### Set the robot's start pose (every real run)

Nav2's AMCL starts assuming the robot is at the map origin, but the real start never matches it. In
RViz, click **2D Pose Estimate**, then click the robot's actual spot on the map and drag in the
direction it's physically facing. The red LiDAR points should snap onto the map walls — if not,
repeat until they do. Only then place blooms and press Start. Re-do this after any Nav2/AMCL
restart. (`sim_only` does this automatically.)

### Shutting down

Stop the laptop launches with `Ctrl-C`. Power the robot off cleanly **before** flipping its switch:

```bash
ssh turtlebot@192.168.8.36 'sudo shutdown now'
```

---

## Operator console

- Click anywhere on the map to drop a bloom; **Clear** removes them.
- **Start** runs the mission (nearest bloom first, navigate, spin-spray, repeat). **Stop** halts it.
- **E-STOP** latches forward motion off everywhere; **Resume** clears it. (Space starts a mission,
  Esc triggers E-STOP.)
- Bloom colour = state: yellow pending, blue active, green treated, grey skipped.
- Banners show mode, mission state, sync status + error, command->motion latency, battery, the
  safety gate, and E-STOP.

---

## How the pieces talk (topics)

All commands funnel through one chokepoint so a single safety gate covers every source:

```
teleop / GUI / Nav2 controller  ──►  /dt/cmd_vel_raw  ──►  twin_mediator (safety gate)
                                                              ├──►  /cmd_vel        (active robot)
                                                              └──►  /sim/cmd_vel    (mirror, in `both`)
```

The active robot is always on the bare topics (`/scan`, `/odom`, `/cmd_vel`, `/battery_state`); in
`both` the mirror sim lives entirely on `/sim/*` so the two never collide. Everything the GUI and the
sync supervisor consume is published on `/dt/*` (`/dt/real_pose`, `/dt/sim_pose`, `/dt/scan_active`,
`/dt/sync_error`, `/dt/latency_ms`, `/dt/sync_ok`, `/dt/alerts`, `/dt/safety`, `/dt/health`,
`/dt/estop`, `/dt/mode`, `/dt/markers`, `/dt/mission_state`).

Note for Jazzy: `/cmd_vel` is `geometry_msgs/TwistStamped` (both the real bringup and Gazebo expect
it stamped), so the whole bus is `TwistStamped`. The launch sets `enable_stamped_cmd_vel:true` on
Nav2 and remaps its controller output onto `/dt/cmd_vel_raw`.

---

## Configuration

Every tunable lives in `config/twin.yaml` (nodes never hard-code these) — the safety stop distance
and front sector, the Burger speed limits, the sync tolerances and latency budget, the spray
revolution count and spin rate, the battery thresholds, and the map dimensions. The launch binds
the whole file to all nodes via the `/**` wildcard. Map metadata in `twin.yaml` matches
`maps/map.yaml`/`maps/map.pgm`.

---

## Troubleshooting

- **Gazebo's 3D window crashes on WSL** (OGRE2 on the d3d12 GL): launch with `gz_gui:=false`, or use
  `docker/open_sim.sh`. The Gazebo server still renders the LiDAR; drive and watch via RViz + the
  console.
- **A sim wedges / won't relaunch:** `pkill -9 -f "gz sim"` inside the container, then relaunch.
- **`import PyQt5` fails:** `pip install --user PyQt5` inside the container (no sudo needed).
- **No `/scan` on the lab laptop:** wrong `ROS_DOMAIN_ID`, not on Wi-Fi `AP2IRR10`, or the robot Pi
  isn't fully up. Check with `ros2 topic hz /scan` (~5 Hz on the LDS-02).
- **Robot starts then does nothing near a wall:** that bloom sits inside Nav2's costmap inflation;
  `mission_runner` projects goals `goal_clearance_m` off walls and skips ones that are truly boxed
  in. Place blooms a little away from walls.
- **Workspace build complains about symlinks / a stale tree:**
  `rm -rf build/ install/ log/ && colcon build --packages-select algae_dt`.
- **Workspace was copied from another machine** (CMake errors naming a foreign path,
  `PermissionError` on `install/**.dsv`, a `turtlebot3_ws (Copy)` folder): a clean rebuild in place
  is not enough — run `./scripts/lab_fix_workspace.sh` (no sudo; rebuilds `~/turtlebot3_ws` from
  the workspace zip in `~/Downloads`, keeps your old dirs renamed aside, repairs `~/.bashrc`).

---

## AI assistance and authorship

The code in this repository is authored and owned by the algae-dt team (TU/e 2IRR10); the
repository is owned by Aleksandar Kolev (GitHub `aleksander-kolev`). Claude (Anthropic), used as an
AI coding assistant under the team's direction, helped implement the team's design and assisted
with debugging and this documentation. See [`AI_USAGE.md`](AI_USAGE.md) for
the full declaration, the team members, and the prompt log, provided for transparency in line with
the course's academic-integrity guidelines on the use of generative AI.
