# algae-dt — TurtleBot3 Burger digital twin

> New to the codebase? **[`CODE_EXPLAINED.md`](CODE_EXPLAINED.md)** explains the whole project —
> concepts, architecture, every node and library — in plain language, self-contained.

A bidirectional digital twin for a TU/e 2IRR10 algae-bloom cleaning robot. A real TurtleBot3
Burger and a Gazebo Harmonic twin run in parallel with state synchronisation between them. An
operator places algae blooms on a map; the active robot navigates to each one with Nav2 (avoiding
static and dynamic obstacles), drives to the centre and spins three full turns in place
("spraying"), then moves on. A 25 cm dual-LiDAR safety gate stops forward motion in either world —
and the stop is **sticky**: it releases only after the path ahead stays clear (0.35 m for 0.3 s),
so allowed rotation can never flicker the obstacle out of the front cone and creep past it. An
obstacle placed in the *virtual* world enters the real robot's costmaps too (`/dt/scan_nav`), so
Nav2 visibly plans around things that exist only in the twin.
The real-vs-sim divergence is continuously measured against documented tolerances — and **bounded**:
when the pose error stays out of tolerance, the twin snaps the mirror back onto the real robot
(an operator **RESYNC** button, or automatically), with every correction logged as evidence.

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
│  │  ├─ twin_resync.py           bounded-drift correction (both): snaps the sim onto the real
│  │  │                           pose via gz set_pose — RESYNC button or auto on sustained error
│  │  ├─ mission_runner.py        Nav2 goal to each bloom (projected clear of walls), then a
│  │  │                           closed-loop 3-revolution spin-spray; publishes markers + state
│  │  │                           (partial completions are loud: alerts + an explicit summary)
│  │  ├─ operator_gui.py          PyQt5 console: map canvas, real/sim pose + live scan overlay,
│  │  │                           click-to-place blooms, Start/Stop/Clear/E-STOP/RESYNC, banners
│  │  ├─ fake_robot.py            stand-in for the real robot, with a LiDAR raycast from the
│  │  │                           course map (AMCL can localize on it; run real_only/both at home)
│  │  ├─ dynamic_obstacle.py      sweeps a box across the path for the "environment change" demo
│  │  └─ lib/                     pure helpers, no ROS imports (the unit-testable building blocks)
│  │     ├─ geometry.py           world <-> map-pixel transforms, angle math
│  │     ├─ safety.py             the dual-LiDAR front-stop gate (fail-safe)
│  │     ├─ blooms.py             immutable bloom field + state transitions
│  │     ├─ sync.py               pose/sensor discrepancy + unicycle integrator
│  │     ├─ metrics.py            latency calc + sync-CSV row formatting
│  │     ├─ hud.py                battery/scan formatting for the console
│  │     ├─ pgm.py                P2/P5 PGM map parser (for the GUI canvas)
│  │     ├─ occupancy.py          static-map occupancy + goal projection + the map-raycast LiDAR
│  │     ├─ trajectory.py         sinusoidal path for the moving obstacle
│  │     ├─ resync.py             the drift-correction policy (manual/auto, sustain, cooldown)
│  │     ├─ gzcli.py              gz service CLI helpers (set_pose/create request plumbing)
│  │     ├─ scanmerge.py          overlays the trusted mirror's LiDAR onto the real scan, by angle
│  │     │                        (what lets virtual obstacles shape REAL navigation)
│  │     └─ nav2check.py          fail-loud preflight: the stock Nav2 params must carry every key
│  │                              the launch rewrites, or the safety chokepoint silently no-ops
│  ├─ launch/bringup.launch.py    orchestrates the stock stack + Nav2 + our layer for each mode
│  ├─ config/twin.yaml            all tunables (safety, sync tolerances, spray, battery, map dims)
│  ├─ config/sim_bridge.yaml      ros_gz bridge that puts the mirror sim on /sim/* (both mode)
│  ├─ maps/map.pgm, map.yaml      arena occupancy map (Nav2/AMCL + the GUI canvas)
│  ├─ worlds/algae_arena.world    Gazebo model of the lab arena
│  ├─ worlds/obstacle_box.sdf     a spawnable box obstacle
│  ├─ worlds/burger_sim_gt.sdf    the `both`-mode mirror robot: stock burger + a ground-truth
│  │                              pose publisher (what makes the resync teleport genuine)
│  └─ package.xml, setup.py, setup.cfg, resource/
├─ docker/                        dev image + helpers (home / WSL2)
│  ├─ Dockerfile                  ROS 2 Jazzy + turtlebot3 stack + Nav2 + Gazebo Harmonic + PyQt5
│  ├─ build.sh / build.ps1        build the image (tag: algae-dt:dev)
│  ├─ run.sh / run.ps1            start the container (run.sh wires up the X11 display on WSL)
│  ├─ demo_run.sh                 one-shot inside the container: build + launch sim_only with RViz
│  ├─ open_sim.sh                 sim_only with the Gazebo 3D window off (WSL GL workaround) + RViz
│  ├─ open_both.sh                the full hardware-free twin (`both use_fake_robot`) with GUIs
│  ├─ verify_running.sh           one-shot liveness check to exec inside a running container
│  └─ probe_ground_truth.sh       diagnostic: dump the bridged ground-truth frame names
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

# the FULL hardware-free twin (`both` with the fake robot): real-side stand-in + Gazebo mirror +
# RViz + console, AMCL auto-seeded — drive it, place blooms, try the RESYNC TWIN button:
docker run -d --name algae_dt_both --net=host -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/open_both.sh
# (from Windows PowerShell + Docker Desktop, replace the X11 mount with
#  -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix and -e DISPLAY=:0)
docker exec algae_dt_both bash /ci/verify_running.sh   # liveness check: nodes + key topics
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

In this hardware-free `both`, AMCL is auto-seeded at the origin (the fake robot deterministically
starts there, matching the sim spawn) so Nav2 activates with no RViz click, and the fake robot's
LiDAR is raycast from the course map — AMCL genuinely localizes on it and driving at a wall trips
the 25 cm gate for real. On the real robot, the operator's RViz **2D Pose Estimate** stays the
localization source, exactly as the course teaches.

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
./scripts/lab_run.sh --no-log        # skip the default evidence recording (below)
```

Overrides if your robot differs: `ROBOT_IP=… ROS_DOMAIN_ID=… ./scripts/lab_run.sh`. Force a
runtime with `--native` / `--docker`.

**Every run records evidence by default** into `~/turtlebot3_ws/lab_logs/<UTC-stamp>_<commit>/`:
the full launch console (`console.log`), a replayable `ros2 bag` of `/dt/*` + Nav2's plans and
commands + both scans + TF + `/rosout` (`bag/` — inspect with `ros2 bag info`, replay with
`ros2 bag play`), the per-node ROS logs (`ros/`), and the sync supervisor's `sync_metrics_*.csv`.
Copy that folder to USB/OneDrive **before leaving** — the laptop can be wiped. If the script exits
with **code 6**, even after auto-repair this machine's `turtlebot3_navigation2` params file cannot
host the safety chokepoint (Nav2 would drive the robot UNGATED — only a file with no
`collision_monitor` section at all trips this): update `~/turtlebot3_ws/src` (turtlebot3, jazzy
branch) and rebuild, or demo with `--sim`. Ordinary key differences are repaired automatically and
each applied repair is printed at launch.

### If the script doesn't work — fully manual

This works no matter where the repo sits — including **the whole repo dropped inside the
workspace** (`~/turtlebot3_ws/src/algae-dt/`): colcon discovers packages recursively, so the
nested `ros2_ws/src/algae_dt` is found without copying anything around.

```bash
# 1) fresh terminal — source the underlays (ROS + the lab's from-source turtlebot3 stack)
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# 2) ONE copy only: if a previous run copied the package to src/algae_dt AND the whole repo
#    is also under src/, colcon aborts with "duplicate package algae_dt" — remove the copy:
rm -rf ~/turtlebot3_ws/src/algae_dt        # keep the repo; skip if it doesn't exist

# 3) build just our package (colcon finds it wherever it is under src/)
cd ~/turtlebot3_ws
colcon build --packages-select algae_dt
source install/setup.bash

# 4) environment — repeat these in EVERY terminal you open
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36 ROS_LOCALHOST_ONLY=0

# 5) robot link check (the Pi bringup from step 2 above must already be running)
ros2 topic hz /scan                        # ~5 Hz on the LDS-02

# 6) launch (RViz opens automatically in `both`; do the 2D Pose Estimate, wait for the
#    SYNC banner to go green — the twin auto-aligns — then place blooms and Start)
ros2 launch algae_dt bringup.launch.py mode:=both
# no robot available?  mode:=sim_only  (skip step 5)
```

If something in that sequence fails: a build error naming **another machine's paths** /
`PermissionError` on `install/**` → the workspace itself is broken, run
`./scripts/lab_fix_workspace.sh` first; `turtlebot3_gazebo`/`turtlebot3_navigation2`/`nav2_*`/
`rviz2` **not found** → the stack isn't in this workspace (recover it, or flag a missing system
package to a TA — no sudo on the lab PC); `import PyQt5` fails → `pip install --user PyQt5`;
zero topics from the robot → wrong `ROS_DOMAIN_ID`, not on `AP2IRR10`, or a leftover
`ROS_LOCALHOST_ONLY=1` in the shell (step 4 neutralizes it).

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
restart. (`sim_only` and the hardware-free `both` do this automatically.)

After the estimate the twin aligns itself: the mirror spawns at the map origin, so the pose error
exceeds tolerance and `twin_resync` auto-snaps the mirror onto the real robot within a few seconds
(or press **RESYNC TWIN**). Wait for the SYNC banner to go green, then Start. The same correction
fires any time drift exceeds the documented tolerance mid-session — it's logged evidence, not a
failure.

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
- **RESYNC TWIN** (`both` only) snaps the Gazebo mirror onto the real robot's pose; the same
  correction fires automatically when the sync error stays out of tolerance. It waits for AMCL to
  be localized — a click before the 2D Pose Estimate queues and executes once localization lands.
- Bloom colour = state: yellow pending, blue active, green treated, grey skipped. A skip is loud:
  an alert names the bloom and why, and the final mission state spells it out
  (`complete (1 treated, 1 skipped)`, amber banner) — plain green `complete` means all treated.
- Banners show mode, mission state, sync status + error (with a note when the twin recently
  resynced), command->motion latency, battery, the safety gate, E-STOP, and alerts.

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
`/dt/scan_nav`, `/dt/sync_error`, `/dt/latency_ms`, `/dt/sync_ok`, `/dt/alerts`, `/dt/safety`,
`/dt/health`, `/dt/estop`, `/dt/mode`, `/dt/localized`, `/dt/markers`, `/dt/mission_state`).

`/dt/scan_nav` is what Nav2's *obstacle* sources (costmap layers + collision monitor) read instead
of the raw `/scan`: normally a verbatim pass-through, but in `both` — while the twin is in sync —
the trusted mirror's laser returns are merged in (by angle; the LDS-02 and the Gazebo lidar index
their beams differently), so an obstacle dropped in the virtual world makes the REAL robot plan
around it. AMCL deliberately keeps the bare `/scan`: localization must never see virtual returns,
and an out-of-sync mirror is excluded exactly as it is from the safety gate. In `both` the mirror's
`/sim/cmd_vel` is also scaled by the measured real-time factor (clamped 1/RTF), so a slow sim still
covers the real robot's ground per wall-clock second instead of structurally under-travelling.

In `both`, `/dt/sim_pose` is the mirror's Gazebo **ground-truth** pose (`/sim/ground_truth`, from
the pose publisher in `worlds/burger_sim_gt.sdf`; the world frame equals the map frame by
construction). That is what makes the drift correction genuine: when `twin_resync` teleports the
mirror onto the real robot's pose (`/dt/resync_cmd` from the console, or automatically after a
sustained out-of-tolerance error), the displayed pose, the mirror's LiDAR viewpoint, and the
measured sync error all move together, and the executed correction is published on
`/dt/resync_event` and stamped into the sync CSV's `resync` column.

Note for Jazzy: `/cmd_vel` is `geometry_msgs/TwistStamped` (both the real bringup and Gazebo expect
it stamped), so the whole bus is `TwistStamped`. The launch reroutes Nav2's *final* velocity (the
collision monitor's `cmd_vel_out_topic`) onto `/dt/cmd_vel_raw` and caps Nav2's planned speed at
the Burger's real 0.22 m/s ceiling (the stock file plans 0.3 — saturated wheels executed every
fast arc tighter than the RViz plan). These overrides are **checked and repaired in**
(`lib/nav2check.py`): the stock params file is loaded, every needed key is replaced *or added*
with path-aware placement (a lab machine's file shipping without `use_sim_time` keys is
auto-fixed, the TwistStamped chain is enforced), and Nav2 launches on the patched copy. A real
mode refuses to start only if the file genuinely cannot host the safety chokepoint (no
`collision_monitor` section) — anything less is a named warning, never a blocked demo.

---

## Configuration

Every tunable lives in `config/twin.yaml` (nodes never hard-code these) — the safety stop distance
and front sector, the Burger speed limits, the sync tolerances and latency budget, the resync
policy (`resync_auto_enable`, sustain, cooldown), the spray revolution count and spin rate, the
battery thresholds, the fake robot's LiDAR, the dynamic obstacle's sweep, and the map dimensions.
The launch binds the whole file to all nodes via the `/**` wildcard. Map metadata in `twin.yaml`
matches `maps/map.yaml`/`maps/map.pgm`.

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
- **Real robot navigates but won't spray-spin (the sim spins):** the spin command reached both
  (the sim proves it) — the real side is actuation. Real modes spray at the derated
  `spray_omega_real_radps` (1.5 rad/s; a 2.8 rad/s spin needs ~0.224 m/s wheel speed = the
  Burger's motor ceiling, unreachable under battery sag). Isolate with
  `ros2 topic pub -r 10 /dt/cmd_vel_raw geometry_msgs/msg/TwistStamped "{twist: {angular: {z: 1.0}}}"`,
  stepping `z` up; check `/battery_state` voltage under load (want ≥ ~11.5 V).
- **Real robot doesn't follow the RViz path / weaves toward walls (RViz itself looks perfect):**
  FIRST check there is exactly **one** publisher on the motor topic: `ros2 topic info -v /cmd_vel`
  must list only `twin_mediator` — two or more means Nav2 is bypassing the safety bus (the params
  file lost the chokepoint keys; the `nav2check` preflight should have refused to start). Then
  watch the console SAFETY banner — if it flashes BLOCKED while the real path is clear, that was
  the *diverged sim mirror* grazing virtual geometry and phantom-braking the real robot through
  the dual-LiDAR gate (forward chopped, rotation preserved → the robot curls off its path).
  Fixed: the mirror's scan now vetoes the real robot only while the twin is in sync
  (`/dt/sync_ok`); the real robot's own LiDAR always gates. Also fixed at the source: Nav2's plan
  is capped at the Burger's true 0.22 m/s (saturated wheels used to bend every fast arc tighter
  than planned) and the command clamp preserves curvature. If nav still misbehaves with SAFETY
  green: re-check localization mid-drive (red scan points must sit ON the map walls — if they
  detach, re-do the 2D Pose Estimate), remember the shared arena (other robots/people are real
  obstacles NOT on the map — Nav2 legitimately detours), and tune inflation at runtime:
  `ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25` (and the
  same on `/local_costmap/local_costmap`), then clear both costmaps.
- **The safety stop seems to "not work" while navigating at an obstacle:** the robot used to
  rotate the box out of its ±20° front cone and lurch-creep past it (forward unblocked the instant
  the cone read clear). The stop now **latches** — forward stays cut until the cone reads beyond
  0.35 m for 0.3 s sustained; rotation/back-up keep working throughout. Every lab run's bag
  (`lab_logs/<stamp>/bag`) records `/dt/safety`, `/dt/scan_nav` and `/cmd_vel` for after-the-fact
  proof of exactly when and why the gate held.
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
