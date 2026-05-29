# CLAUDE.md — algae-dt (course-faithful TurtleBot3 Digital Twin)

Persistent guidance for any session on this project. **Read this first.** This project is a
**clean restart** of the old `algae-twin` repo, rebuilt to follow the **TU/e 2IRR10 course's
recommended method exactly**: WSL+Docker (or the native lab laptop), the **default turtlebot3
packages**, and **one custom ROS 2 package** for our digital-twin logic. Append to
`docs/BEST_APPROACHES.md` §Lessons as you learn things.

> **Why the restart (read this — it is the whole point):** the old repo hand-built a Burger
> SDF/URDF, its own `robot_state_publisher` replacement, its own teleop and Nav2 wiring, to avoid
> turtlebot3 packages it believed were absent. The course material proves that was the wrong target:
> the **provided `turtlebot3_ws` Docker image ships the turtlebot3 stack**, the course *mandates*
> building a **custom package on top of the default packages**, and the lab laptop runs that same
> stack. We stop fighting the environment and use it. This deletes ~60–70% of the old code and the
> hardest bugs with it. Full reasoning: `docs/BEST_APPROACHES.md` §"Why we use the stock stack".

## 1. What this is
A bidirectional **digital twin** for a TU/e 2IRR10 algae-bloom cleaning robot (**TurtleBot3
Burger**, ROS 2 **Jazzy**, **Gazebo Harmonic**). A real Burger and a Gazebo twin run in parallel
with **bidirectional communication + state synchronization**. An operator places algae blooms on a
map; the active robot **Nav2-navigates** to each (avoiding static + dynamic obstacles), drives to
the centre and **spins in place 5 s** ("spraying"), then advances. A **dual-LiDAR 25 cm safety
gate** stops forward motion in *either* world. The deliverable runs on the **lab HP Z-Book** and the
**lab Burger** via the course's documented commands.

Authoritative docs: design + rubric `docs/RUBRIC_MAP.md`; plan `docs/PLAN.md`; environment
`docs/SETUP.md`; rules `docs/RULES.md`; patterns/lessons `docs/BEST_APPROACHES.md`.

## 2. The course method (NON-NEGOTIABLE — this is how it is graded & how it runs in the lab)
- **Environment:** WSL2 + Docker with the course's **`turtlebot3_ws` image** at home; the **native
  lab laptop** (`~/turtlebot3_ws`) in the lab. Both ship ROS 2 Jazzy + the turtlebot3 stack. See
  `docs/SETUP.md`. (VM is the slower alternative; WSL+Docker is "strongly recommended for 2IRR10".)
- **Use the DEFAULT turtlebot3 packages** — they are *officially provided & recommended*:
  `turtlebot3_gazebo` (model + spawn + `robot_state_publisher` + ros_gz bridge),
  `turtlebot3_bringup` (real-robot SBC + rviz2), `turtlebot3_navigation2` (Nav2 + AMCL + tuned
  params + map), `turtlebot3_cartographer` (SLAM, optional), `turtlebot3_teleop` (keyboard).
  **Do NOT re-implement what these provide.**
- **One CUSTOM package `algae_dt`** holds *only* our digital-twin logic (mediator, sync supervisor,
  mission runner, operator GUI, pure libs). This mirrors the course's `my_tb3_world` / `tb3_safety_stop`
  / `tb3_weather_dt` pattern.
- **Workspace path is `~/turtlebot3_ws/src/algae_dt`** on every machine (course convention).
- **Development happens at HOME (in sim); the lab is for TESTING ONLY.** (Course rule, repeated
  everywhere.) Everything must pass in `sim_only` at home before a lab session.
- **No sudo / no system or network setting changes on lab equipment** (Golden Rule #12). The
  turtlebot3 stack is already present, so you never need to install it. Extra packages → ask a TA
  (course process); for our own Python deps prefer `pip install --user`/in-container, no sudo.

## 3. Lab facts (verified)
- Lab laptop: **HP Z-Book**, provided per session, pre-configured account, **factory-resettable
  without notice → back up to git/OneDrive every session.** Hostname seen in course logs: `2IRR10-NB26`.
- Robot: TB3 Burger, number on a sticker; **IP on a sticker** → `ssh turtlebot@<robot-ip>`.
  Our team's robot last time: **#36, `192.168.8.36`**. **`ROS_DOMAIN_ID = robot number`** on BOTH sides.
- Wi-Fi **`AP2IRR10`** (pre-configured; do not change). LiDAR **LDS-02** (360 beams, 0.12–3.5 m).
- Robot bringup on the Pi: `export TURTLEBOT3_MODEL=burger; export LDS_MODEL=LDS-02;
  ros2 launch turtlebot3_bringup robot.launch.py`. Shutdown **`sudo shutdown now` on the robot Pi
  BEFORE flipping the power switch** (the robot Pi grants this sudo; the laptop does NOT).
- **Jazzy `/cmd_vel` is `geometry_msgs/TwistStamped`** by default (real bringup AND turtlebot3_gazebo).

## 4. Architecture — `algae_dt` custom package (thin DT layer on the stock stack)
Nodes:
- **`twin_mediator`** — the course's **DT Integration Node (fan-in / fan-out)**. Subscribes the
  pre-safety command bus `/dt/cmd_vel_raw` + both scans; applies the **25 cm dual-LiDAR safety gate**;
  fans the safe command out to **`/cmd_vel` (TwistStamped, real)** AND **`/sim/cmd_vel` (sim)**.
  Mirrors `/odom`→`/dt/real_pose`, sim pose→`/dt/sim_pose`, `/battery_state`→`/dt/health`. Owns the
  latched **`/dt/estop`**. Republishes the ACTIVE robot's scan/odom to `/dt/scan_active` + `/dt/odom_active`.
- **`sync_supervisor`** — the course's **Real-Time Synchronization & Tolerances** deliverable.
  Computes pose/sensor discrepancy + command→motion latency; publishes `/dt/sync_error`,
  `/dt/latency_ms`, `/dt/sync_ok` (Bool); **logs a CSV** and **publishes `/dt/alerts` when out of
  documented tolerance.** (This is the most-overlooked graded item — we make it first-class.)
- **`mission_runner`** — Nav2 `BasicNavigator` goal to each bloom centre, then 5 s spin-spray on
  `/dt/cmd_vel_raw`; `/dt/markers`, `/dt/mission_state`. Mission loop on a worker thread.
- **`operator_gui`** (PyQt5) — map canvas (in-tree `lib/pgm.py`), real+sim pose overlay, live scan
  overlay, bloom markers, click-to-place, Start/Stop/Clear/E-STOP, banners (mode/sync/latency/
  battery/safety/mission). Subscribes `/dt/*` only.
- Teleop = stock **`turtlebot3_teleop`** remapped `-r /cmd_vel:=/dt/cmd_vel_raw` (no custom teleop).
- Pure libs (no ROS, unit-tested): `lib/{geometry,safety,blooms,sync,metrics,pgm}.py`.

### Modes (`ros2 launch algae_dt bringup.launch.py mode:=...`)
- `sim_only` (default; fully developed/tested at home): `turtlebot3_gazebo` + Nav2 + our DT layer.
- `real_only`: `turtlebot3_bringup` (on Pi) + `turtlebot3_navigation2` (AMCL) + our DT layer.
- `both`: real **leads**, sim **mirrors 1:1** (mediator fan-out). Sim namespaced to `/sim/*`.

### Topic contract (course Mini-Project-3 contract, extended)
- Command bus: `/dt/cmd_vel_raw` (teleop OR GUI OR Nav2 controller, remapped) → mediator gates →
  **`/cmd_vel`(TwistStamped) + `/sim/cmd_vel`**.
- Real (bare): `/scan /odom /cmd_vel(TwistStamped) /battery_state /tf`.
- Sim (`/sim/*`): `/sim/scan /sim/odom /sim/cmd_vel /sim/tf /clock`.
- Digital (`/dt/*`): `/dt/cmd_vel_raw /dt/real_pose /dt/sim_pose /dt/scan_active /dt/odom_active
  /dt/sync_error /dt/latency_ms /dt/sync_ok(Bool) /dt/alerts(String) /dt/safety(Bool) /dt/mode(String)
  /dt/health /dt/estop(Bool,latched) /dt/blooms(MarkerArray) /dt/markers /dt/mission_state /dt/mission_cmd`.
- **Topic-collision rule (course-stated, critical): real and sim must NEVER publish the same topic
  name.** Real = bare, sim = `/sim/*`, sim TF off the global `/tf` in `both`.

## 5. Build & run (course commands — full detail in docs/SETUP.md)
Home (WSL+Docker), inside the container:
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select algae_dt
source install/setup.bash && export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```
Lab laptop (native): identical but no `docker run`; `cd ~/turtlebot3_ws`, build, launch.

## 6. Workflow
Investigate → write the failing test (pure libs) → implement minimal → **build+run in `sim_only` at
home** → keep `docs/PLAN.md` task list current → only take *tested* artifacts to the lab. Verify any
new dependency is in the `turtlebot3_ws` image / lab laptop before relying on it. Commit before every
lab session (the laptop can be wiped).

## 7. Pointers
- `docs/SETUP.md` — exact WSL+Docker, lab-laptop, and robot-connection commands.
- `docs/RULES.md` — hard constraints (course golden rules + technical invariants + coding style).
- `docs/BEST_APPROACHES.md` — patterns, lessons & gotchas (ported + reframed for the stock stack).
- `docs/PLAN.md` — phased 12/12 plan mapped to the 7 lab sessions + Week-4/8 reviews + Week-9 video.
- `docs/RUBRIC_MAP.md` — every deliverable → rubric pillar → evidence artifact → demo step.
