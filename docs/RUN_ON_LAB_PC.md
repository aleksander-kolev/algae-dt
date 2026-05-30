# RUN_ON_LAB_PC.md — pull the repo & run on the lab laptop (one script, all Docker)

Repo: **https://github.com/aleksander-kolev/algae-dt** (private; default branch `main`).
**Everything runs inside the course `turtlebot3_ws` Docker container** — the bare lab host has **no
native turtlebot3** (verified 5 ways, `docs/DECISIONS.md` D1); the stack is in the image. The script
**recreates the `~/turtlebot3_ws` folder you deleted**, copies the package in, builds, and launches.

---

## 0. Readiness verdict — READ FIRST (honest, current commit)
| Thing | Status | Note |
|---|---|---|
| Pure logic (6 libs) | ✅ VERIFIED | 82 unit tests pass. |
| Nodes + launch (structure) | ✅ written, ❌ **never run in ROS/Gazebo** | only unit-tested; needs a container smoke-test. |
| **Docker + `turtlebot3_ws` image on the lab laptop** | ⚠️ **UNVERIFIED — the one true prerequisite** | the script checks it and aborts with guidance if missing (it can't be papered over). |
| `sim_only` end-to-end | ⚠️ smoke-test needed | risk: launch starts `gz_sim`+spawn but no explicit `ros_gz` bridge → confirm `/scan`,`/clock` reach ROS. |
| `real_only` | ⚠️ likely after smoke-test | verify Nav2's final `cmd_vel` reaches `/dt/cmd_vel_raw`. |
| `both` (real↔sim mirror) | ❌ NOT READY | sim isn't namespaced to `/sim/*` yet (PLAN T5.1). |

**Bottom line:** build + unit tests are green; the integrated launch is unproven. Do the home
smoke-test (`./scripts/lab_run.sh sim_only` in your dev container) and fix the flagged items before
the lab. The script never *guesses* — it fails loudly if Docker/image/turtlebot3 are absent.

---

## 1. The one command (from a fresh clone)
```bash
git clone https://github.com/aleksander-kolev/algae-dt.git
cd algae-dt
./scripts/lab_run.sh                  # DEFAULT = both: FULL DEMO (real leads + sim mirror), robot #36
./scripts/lab_run.sh sim_only         # home / no robot (smoke-test)
./scripts/lab_run.sh real_only        # real robot only (no sim mirror)
# options:  --headless   --rebuild   TB3_IMAGE=<name>   ROS_DOMAIN_ID=<n> ROBOT_IP=<ip> (if not #36)
```
No argument → **`both`**, so the whole twin (real + Gazebo mirror + GUI) comes up ready to record.
Robot defaults are baked in (**#36 / 192.168.8.36 / ROS_DOMAIN_ID=36**); override with the env vars
above if you're given a different robot. The script pings the robot and warns if it's not up yet.
`scripts/lab_run.sh` does all of it: preflight (Docker + image + domain) → recreate `~/turtlebot3_ws/src`
→ copy the package → `docker run --net=host … turtlebot3_ws` → inside: source ROS + the turtlebot3
overlay, `colcon build --packages-select algae_dt`, `source install/setup.bash`,
`ros2 launch algae_dt bringup.launch.py mode:=<mode>`. The GUI/RViz/Gazebo render via X11.
**Teleop in another terminal** (the script prints the exact line): `docker exec -it turtlebot3_container …
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw`.

## 2. Self-healing — the script trusts NOTHING on the machine
`lab_run.sh` bootstraps whatever's missing, in order, before it runs:
1. **Docker missing / daemon unreachable → installs rootless Docker** (official
   `https://get.docker.com/rootless`, no sudo) and starts the daemon.
2. **No turtlebot3 image** (`turtlebot3_ws` / `algae-dt:dev` / `algae-dt:fallback`) **→ builds one**
   from **`scripts/Dockerfile`** (the full stock turtlebot3 + Nav2 + Gazebo stack), tagged
   `algae-dt:fallback`.
3. **`~/turtlebot3_ws` missing → recreates it**, copies the package, builds.
4. Verifies turtlebot3 resolves **inside** the container (`ros2 pkg prefix turtlebot3_gazebo`) and
   aborts loudly if not. (On the bare host `ros2 pkg list | grep turtlebot3` is EXPECTED EMPTY.)

**Honest limits of the fallbacks (the script says these, never silently fails):**
- Rootless Docker needs the **`uidmap`** tools (root to install) + **internet**; and rootless
  `--net=host` **may not reach the robot over DDS** → for the real/both hardware demo prefer rootful
  Docker (TA-provided); `sim_only` works fine rootless.
- Building the fallback image needs **internet** (base pull + apt) and **~10–20 min**.
- If neither Docker can be obtained nor an image built (fully offline, no `uidmap`) → that's a
  genuine blocker for a TA, not something any script can conjure.

## 3. Robot bringup (robot Pi — native; the script prints this for real/both)
```bash
ssh turtlebot@192.168.8.36
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36
ros2 launch turtlebot3_bringup robot.launch.py
```
Laptop + robot on Wi-Fi `AP2IRR10`, same `ROS_DOMAIN_ID=36`. In RViz set **2D Pose Estimate** before Start.

## 4. Manual fallback (if the combined launch misbehaves — each terminal: `docker exec -it turtlebot3_container bash`)
```
source /opt/ros/jazzy/setup.bash; source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null; source /ws/install/setup.bash
T1: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=/ws/src/algae_dt/maps/map.yaml
T2: ros2 run algae_dt twin_mediator   T3: ros2 run algae_dt sync_supervisor
T4: ros2 run algae_dt mission_runner  T5: ros2 run algae_dt operator_gui
T6: ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw
```

## 5. Shutdown + backup
`ssh turtlebot@<ip> 'sudo shutdown now'` BEFORE the power switch. Commit/push + copy to USB before
leaving (laptop may be wiped).

---

## Provenance — exactly where I'm sure this works, and where I'm NOT
**SURE — verbatim from the Canvas material you pasted:**
- The `docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <home>/turtlebot3_ws:/ws --user $(id -u):$(id -g) turtlebot3_ws bash`
  invocation → Canvas **"How to run Gazebo & Modify the 3d World"**, **"Changing Robot Inflation"**,
  **"DT Example With Only Gazebo"** (identical command in all three).
- In-container `source /opt/ros/jazzy/setup.bash` → `source /opt/turtlebot3_ws/install/setup.bash` →
  `cd /ws && colcon build && source install/setup.bash` → `export TURTLEBOT3_MODEL=burger` →
  `ros2 launch …`, and `docker exec -it turtlebot3_container bash` for extra terminals → the same DT
  example pages + a peer group's `instruction.txt`.
- Custom-package layout `~/turtlebot3_ws/src/<pkg>` + `colcon build --packages-select` → Canvas
  **"How to create Packages"** / **"Turtlebot3 Workspace Setup & Package Management"** (Sections 1–3).
- Robot connection (Wi-Fi `AP2IRR10`, `ROS_DOMAIN_ID=<robot#>`, `ssh turtlebot@<ip>`,
  `turtlebot3_bringup robot.launch.py`) → Canvas **"Connecting lab laptop to lab robot"**.
- turtlebot3 is **not** on the bare host (only in the image) → our own 5-way verification
  (`dpkg`, `ros2 pkg list`, `ls ~/turtlebot3_ws/{src,install}/turtlebot3*`) — `docs/DECISIONS.md` D1.

**NOT verified — must be checked on the actual machine (the script fails safe, never silent):**
- That **Docker + the `turtlebot3_ws` image exist on this lab laptop.** The Canvas docs assume it; our
  forensics couldn't confirm Docker on the bare host. → script preflight aborts with guidance if absent.
- That the **exact image overlay path** is `/opt/turtlebot3_ws/install` (seen in a *peer* group's
  instructions). → script sources it *if present* and otherwise verifies turtlebot3 resolves at all,
  aborting if not.
- That **our `bringup.launch.py` runs end-to-end** (the `ros_gz` bridge + `both`-mode `/sim/*`
  namespacing are still being hardened — §0). → do the `sim_only` smoke-test first.
- X11/`HOME`/`--user` specifics on the exact laptop (the script sets `HOME=/ws`, mounts X11, and best-effort
  `xhost +local:`; if the GUI doesn't show, run `--headless` or `xhost +local:` manually).
