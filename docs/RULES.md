# RULES.md — hard constraints (never violate)

Three groups: **A) course/lab golden rules**, **B) technical invariants** (break these → silent
runtime failure), **C) coding & process conventions**.

---

## A. Course & lab golden rules (from "Lab Rules and Regulations" — violations affect the grade)
1. **No sudo / no admin on any lab laptop or robot.** Do not attempt to gain it. (The turtlebot3
   stack is already installed, so you never need to.) The robot Pi's `sudo shutdown now` is the only
   sanctioned sudo, on the robot, not the laptop.
2. **Do not change system or network settings** on lab equipment. Wi-Fi `AP2IRR10`, IPs, ROS IDs are
   pre-configured and labelled — use them, don't edit them.
3. **Need an extra package?** Install it on your VM/Docker, show the TAs the topics/nodes + why it's
   needed; they add it for the next session. Never expect to install system packages in the lab.
4. **Robot handling:** never move/rotate a powered robot off the ground; test only inside the wooden
   arena; one person in the arena at a time; prop wheels off the table if testing at a desk; never
   disassemble or add/remove components; handle LiPo batteries with care, report swelling/smell/beeps.
5. **Shut the robot down properly** (`sudo shutdown now` on the Pi) **before** the power switch.
6. **Back up every session — carry a USB.** The laptop may be factory-reset without notice AND a
   freshly-wiped laptop may not reach OneDrive/git (sign-in under no-admin/network limits). So a USB
   stick with the `algae_dt` package + rebuild commands is **mandatory**, not just git/OneDrive.
7. **Lab = testing, home = development.** Do not write/iterate code in the lab. In the lab, build
   ONLY our package: `colcon build --packages-select algae_dt`. The full `rm -rf build/ install/
   log/ && colcon build` is reserved for the documented "dirty symlink" failure (never the default —
   it rebuilds the whole turtlebot3 workspace and burns testing time).
8. **Attendance:** 2–3 members per session, rotating. Decide roles (driver/spotter/logger/presenter)
   before arriving; bring a 30–60 min test plan (see `docs/PLAN.md`). **End-of-session:** the logger
   confirms the robot was shut down properly (`sudo shutdown now` on the Pi) before the power switch.

## B. Technical invariants (verified — each one, broken, fails silently)
1. **cmd_vel type contract (Jazzy, doc-confirmed) — the whole bus is `TwistStamped`.** On Jazzy the
   real `turtlebot3_node` runs `enable_stamped_cmd_vel: true` (burger.yaml) → it **subscribes
   `TwistStamped`** on `/cmd_vel`, and `turtlebot3_teleop` **publishes `TwistStamped`**. So
   `/dt/cmd_vel_raw` is **`TwistStamped`** (one type — never mix `Twist`/`TwistStamped` on it). The
   mediator forwards `TwistStamped`→ real `/cmd_vel`, and the **runtime-verified** type → `/sim/cmd_vel`
   (`ros2 topic type /sim/cmd_vel`; on Jazzy turtlebot3_gazebo this is typically `TwistStamped` too).
2. **Set `enable_stamped_cmd_vel: true` on Nav2** (controller_server / behavior_server /
   velocity_smoother) — Nav2 on Jazzy defaults to plain `Twist` (per Nav2 migration/Jazzy docs), so
   without this its commands won't match the `TwistStamped` bus and **the robot activates but never
   moves** (silent). Inject it via a **`params_file`** passed to `navigation2.launch.py` (the
   `/**`-wildcard in `twin.yaml` does NOT reach Nav2 — Nav2 loads its own params). Also **remap Nav2's
   controller `cmd_vel` → `/dt/cmd_vel_raw` in the launch** (via `SetRemap` around the
   `turtlebot3_navigation2` include — NOT on `mission_runner`, which is a BasicNavigator action client
   and publishes no `cmd_vel`), so the mediator stays the single safety chokepoint for autonomy.
3. **Topic-collision rule (course-stated):** real and sim must NEVER publish the same topic name.
   Real = bare (`/scan /odom /cmd_vel /tf`), sim = `/sim/*`, sim TF off the global `/tf` in `both`.
4. **`use_sim_time:=true` ONLY in `sim_only`.** In `real_only`/`both` the real robot leads on wall
   time; the sim mirrors and its sim-stamped topics are re-stamped, never reaching Nav2/TF.
5. **The safety gate fails SAFE.** A *considered* scan that goes stale (had data, now older than
   `max_data_age_s`) is treated as blocked. Startup with no data yet stays unblocked (so the robot
   can warm up). Either real OR sim scan < `stop_distance_m` (0.25) zeroes forward motion on BOTH.
6. **Auto-E-STOP is PUBLISHED, latched.** The mediator owns `/dt/estop` (latched), sets `True` on
   real critical battery (≤ `battery_critical_v`); mission + GUI obey it. Distributed state lives on
   a topic, never in one node's variable. RESUME clears it.
7. **Nav failure must NOT spray / mark treated.** Only a Nav2 `SUCCEEDED` + (sim-only) generous
   `center_tol_m` gross check sprays and marks treated. `nav_failed`/`nav_timeout` → bloom skipped
   (grey). A spray cut short by Stop/E-STOP leaves the bloom PENDING (honest), not treated.
8. **Generous `center_tol_m` (0.5 m), sim-only.** Nav2's `xy_goal_tolerance` (0.15) on the
   controller TF pose is authoritative arrival. The gross check uses ground-truth/odom pose which
   lags during deceleration — a tight margin false-rejects real arrivals. In `real_only`/`both`,
   `/dt/real_pose` is odom-frame and not map-comparable, so the gross check runs ONLY in sim.
9. **`ROS_DOMAIN_ID = robot number` on both sides**, same Wi-Fi. Mismatch → zero topics cross.
10. **Isolate back-to-back full-stack sim tests by `ROS_DOMAIN_ID`** (+ a settle delay): sequential
    gz+Nav2 stacks on one domain collide (duplicate nodes → `planner_server` SIGABRT).

## C. Coding & process conventions (also `~/.claude/rules/common/*`)
- **Reuse before building.** The stock turtlebot3 packages do model/spawn/RSP/Nav2/teleop/SLAM — use
  them. Our package contains ONLY digital-twin logic. (This is the project's founding lesson.)
- **No hardcoded tunables** — everything in `config/twin.yaml`, loaded as ROS params (`/**` wildcard).
- **Immutability** — never mutate; return new objects (esp. `lib/blooms.py`).
- **Small, focused files** (200–400 lines typical, 800 max). Hard logic in **pure libs with unit
  tests**; ROS nodes stay thin wrappers around the libs.
- **Validate at boundaries; handle errors explicitly; never swallow silently.** User-facing messages
  in the GUI; detailed logs elsewhere.
- **TDD, ≥ 80% coverage on pure libs.** Write the failing test first; nodes are integration-tested in
  `sim_only`. Pure libs (`geometry/safety/blooms/sync/metrics/pgm`) carry the logic so they're
  testable without ROS/hardware.
- **Git:** conventional commits (`feat|fix|refactor|docs|test|chore`). Commit before every lab
  session. Branch off `main` for features; PRs summarise the full diff.
- **Match surrounding style.** Build+test in the course container (or native lab laptop) — never
  claim "done" without a green `colcon build` + `pytest` + a `sim_only` run.
