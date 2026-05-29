# BEST_APPROACHES.md — patterns, lessons & gotchas

Living document. **Append to §Lessons as you learn.** Most lessons below were paid for in the old
`algae-twin` build; they're ported and **reframed for the stock-turtlebot3 approach** (many old
problems simply vanish because the stock packages own that code now).

---

## Why we use the stock stack (the founding decision)
The old repo hand-built a Burger SDF/URDF, a `robot_state_publisher` replacement
(`ground_truth_localizer`), custom teleop, and hand-tuned Nav2 params — all to avoid turtlebot3
packages it believed were absent from the lab PC. The course material proves the intended
environment is **WSL+Docker with the provided `turtlebot3_ws` image** (or the native lab laptop),
both shipping the turtlebot3 stack, and the course **mandates a custom package on top of the default
packages**. So:
- **Reuse, don't rebuild.** `turtlebot3_gazebo` = model + spawn + `robot_state_publisher` + ros_gz
  bridge. `turtlebot3_navigation2` = Nav2 + AMCL + tuned params + map handling. `turtlebot3_bringup`
  = real-robot SBC + rviz2. `turtlebot3_teleop` = keyboard. `turtlebot3_cartographer` = SLAM.
- **Our package is thin.** Only the DT layer (mediator, sync supervisor, mission, GUI, pure libs).
- This deletes ~60–70% of the old code and the hardest bugs (TF chain, SDF spawn, mesh/resource
  paths, Nav2 param wiring) with it, and it's the TA-supported path so help is available in the lab.

## Core DT patterns (these ARE the graded deliverables)
- **Fan-in / fan-out mediator (single chokepoint).** Everything that commands motion goes to
  `/dt/cmd_vel_raw`; the mediator applies the safety gate ONCE and fans out to `/cmd_vel` +
  `/sim/cmd_vel`. Never let Nav2 or teleop drive a robot directly — remap their `cmd_vel` to
  `/dt/cmd_vel_raw`. (Course Mini-Project-3 contract; we generalise it to autonomy.)
- **Sync with documented tolerances + alerts (don't skip this).** Publish the *measured* numbers, not
  just a boolean: `/dt/sync_error` (Δxy, Δyaw, sensor-range delta), `/dt/latency_ms` (command→motion,
  scan age). Compare against documented thresholds in `config/twin.yaml`; when out of tolerance,
  publish `/dt/alerts` AND append a CSV row. This is the course's "measured latency and sync error
  reported, tolerance thresholds documented, alerts/logging when out of tolerance" — the single
  highest-value, most-overlooked deliverable.
- **Safety fails safe.** Stale considered-scan → blocked; startup no-data → unblocked. Belt-and-
  suspenders to Nav2's costmap, and the explicit course "25 cm stops both robots" requirement.
- **Distributed state on topics, latched.** `/dt/estop`, `/dt/mode`, `/dt/sync_ok` are latched so a
  late subscriber (GUI) gets the current value. Bump transient state pubs (mission_state) to
  latched-depth-10 so fast transitions (`nav_failed`→`complete`) aren't dropped; assert the STABLE
  final marker colour in tests, not transient states.
- **Honest task accounting.** Spray + mark-treated only on full-duration spin after Nav2 SUCCEEDED.
  Aborted/short spray → bloom stays pending. Nav failure → grey/skipped. Prevents a half-treated
  bloom being wrongly skipped next start by `nearest_untreated`.

## Lessons & gotchas (APPEND as you learn)
- **`tf2_echo` lies under sim time.** The CLI uses wall clock; verify TF via `ros2 topic echo /tf |
  grep frame_id` or a node with `use_sim_time:=true`.
- **`BasicNavigator(node_name='…')`** is accepted on Jazzy (`__init__(self, node_name='basic_navigator',
  namespace='')`). Keep `namespace=''` so action names resolve globally. **Do NOT put `name=` on the
  mission_runner `Node(...)` in the launch** — a `launch_ros` `name=` injects a process-wide
  `__node:=` remap that renames EVERY node the process spawns (incl. BasicNavigator's helper) →
  duplicate `/mission_runner`, no helper. Let the node self-name; params bind via the `/**` wildcard.
- **ros_gz `/cmd_vel` is `gz.msgs.Twist`.** When you bridge the sim's cmd_vel, publish the type the
  bridge expects. With turtlebot3_gazebo on Jazzy the ROS-side `/cmd_vel` is TwistStamped — **verify
  the sim cmd_vel type empirically** (`ros2 topic type /sim/cmd_vel`) before wiring the fan-out; this
  is the #1 "sim activates but doesn't move" trap. (Old lesson: enable_stamped_cmd_vel default FALSE.)
- **Namespacing turtlebot3_gazebo to `/sim/*` for `both` mode.** Launch the sim under a `sim`
  namespace (or remap its bridge topics) so it never collides with the real robot's bare topics, and
  route sim TF to `/sim/tf`. This is the main integration task for `both` — budget time for it.
- **Inflation tuning at runtime (course "Changing Robot Inflation.pdf"), no sudo, no rebuild:**
  `ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25` etc., then
  clear costmaps (`ros2 service call /local_costmap/clear_entirely_local_costmap …`). Persist by a
  small post-launch script (params reset on Nav2 restart). Place blooms ≥ 0.3 m from walls (footprint
  0.105 m + inflation > goal tolerance, else inflation-blocked → grey/skipped).
- **AMCL needs the 2D Pose Estimate**, re-done after any AMCL restart (`set_initial_pose` re-seeds to
  the map origin). Localize BEFORE pressing Start, or the goal plans from an unlocalized tree → skip.
- **Gazebo cleanup:** `pkill -9 -f "gz sim"` between runs (course GazeboTutorial) to avoid leftover
  processes / weirdness.
- **Isolate back-to-back full-stack sim tests by `ROS_DOMAIN_ID`** + a ~3 s settle; sequential
  gz+Nav2 stacks on one domain collide (duplicate nodes → SIGABRT). Run a FRESH container per full
  suite. Unit tests (pure libs) + in-process integration tests are deterministic; full-stack gz
  tests can flake under load — re-run a flaked one once.
- **GUI map via in-tree PGM parser.** Qt's PNM plugin isn't guaranteed in a minimal PyQt5; parse
  `map.pgm` (P5/P2) with `lib/pgm.py` → `Format_Grayscale8` QImage. Headless GUI:
  `QT_QPA_PLATFORM=offscreen` (no xvfb) or under `xvfb-run`.
- **Dependency direction:** if you ever split into two packages, the asset/launch package
  `exec_depend`s on the node package — never the reverse (a cycle makes colcon build nothing →
  `ModuleNotFoundError`). We keep ONE package to sidestep this entirely.
- **`set -u` + sourcing ROS/colcon setup = silent shell exit.** Wrap `source …/setup.bash` in
  `set +u … set -u` in any script.

## TA-familiar fallback: the manual multi-terminal launch
If a combined `bringup.launch.py` misbehaves in the lab, fall back to the course's per-component
multi-terminal style (one `ros2 launch`/`ros2 run` per terminal), which the TAs know:
```
T1: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py        # or our algae_arena world
T2: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=.../map.yaml
T3: ros2 run algae_dt twin_mediator
T4: ros2 run algae_dt mission_runner   (or turtlebot3_teleop teleop_keyboard -r /cmd_vel:=/dt/cmd_vel_raw)
T5: ros2 run algae_dt operator_gui
```
Keep both paths working; the combined launch is for speed, the manual path is for debugging with a TA.
