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

## Library-doc verification (Context7 — Nav2 & TurtleBot3 official docs)
Confirmed before implementation:
- **cmd_vel:** Nav2 `enable_stamped_cmd_vel` defaults **false on Jazzy** (TwistStamped became default
  in Kilted); real `turtlebot3_node` runs it **true** (burger.yaml) so the robot + teleop use
  `TwistStamped`. → bus is `TwistStamped`; set the flag true on Nav2 via `params_file`. (See §cmd_vel.)
- **BasicNavigator** (`nav2_simple_commander`): `goToPose(pose, behavior_tree='')`, `isTaskComplete()`,
  `getFeedback().navigation_time`, `getResult()`→`TaskResult.{SUCCEEDED,CANCELED,FAILED}`,
  `cancelTask()`, `waitUntilNav2Active()`, `setInitialPose(PoseStamped)`, constructor takes `namespace`
  (and `node_name`). All used by our `mission_runner` — verified real.
- **sim_only AMCL seed:** use `nav.setInitialPose(spawn_pose)` from the commander (cleanest) OR
  `set_initial_pose` in the Nav2 `params_file`.
- **`turtlebot3_navigation2 navigation2.launch.py`** accepts `map`, `use_sim_time`, `params_file` —
  our launch/plan use exactly these.
- Nav2 has a built-in **`spin(spin_dist, time_allowance)`** behavior; we still spray via a direct
  timed publish to `/dt/cmd_vel_raw` (gives us the exact N-revolution spin + abort + fan-out-to-both
  control), but `spin()` is a viable alternative if we want Nav2 to own the rotation.

## Lessons & gotchas (APPEND as you learn)
- **`tf2_echo` lies under sim time.** The CLI uses wall clock; verify TF via `ros2 topic echo /tf |
  grep frame_id` or a node with `use_sim_time:=true`.
- **`BasicNavigator(node_name='…')`** is accepted on Jazzy (`__init__(self, node_name='basic_navigator',
  namespace='')`). Keep `namespace=''` so action names resolve globally. **Do NOT put `name=` on the
  mission_runner `Node(...)` in the launch** — a `launch_ros` `name=` injects a process-wide
  `__node:=` remap that renames EVERY node the process spawns (incl. BasicNavigator's helper) →
  duplicate `/mission_runner`, no helper. Let the node self-name; params bind via the `/**` wildcard.
- **cmd_vel is `TwistStamped` on Jazzy (verified against Nav2 + TurtleBot3 docs).** Real
  `turtlebot3_node` runs `enable_stamped_cmd_vel:true` (subscribes TwistStamped) and `turtlebot3_teleop`
  publishes TwistStamped → the whole `/dt/cmd_vel_raw` bus is TwistStamped. **Nav2 on Jazzy defaults to
  plain `Twist`** (TwistStamped became default only in Kilted), so you MUST set
  `enable_stamped_cmd_vel:true` on Nav2's controller/behavior/velocity_smoother via a `params_file`,
  else autonomous commands never reach the bus and **the robot activates but never moves** (the #1
  silent trap). Still **verify `ros2 topic type /sim/cmd_vel`** empirically before wiring the sim
  fan-out (gz bridge side may differ). (A plain-`Twist` bus is WRONG here — it breaks stock teleop.)
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
  `set +u … set -u` in any script. (Hit in `docker/ci.sh`: `AMENT_TRACE_SETUP_FILES: unbound`.)
- **Nav2 cmd_vel topology (Jazzy turtlebot3):** controller_server→`cmd_vel`→velocity_smoother→
  `cmd_vel_smoothed`→**collision_monitor→`cmd_vel`** (final to the robot). The stock `burger.yaml`
  already sets `enable_stamped_cmd_vel: true` everywhere, so the whole chain is TwistStamped. To make
  the mediator the single chokepoint, **rewrite collision_monitor's `cmd_vel_out_topic` →
  `/dt/cmd_vel_raw`** via `RewrittenYaml` (a blanket `SetRemap('/cmd_vel',…)` is WRONG — controller
  *and* collision_monitor both use `cmd_vel`, so it would double-publish the bus). Also flip
  `set_initial_pose: True` for headless sim_only AMCL auto-seed. (Verified in `bringup.launch.py`.)
- **Headless Gazebo render rate throttles autonomy (verified, host limitation — NOT a code bug).**
  `gz sim -s --headless-rendering` renders the LiDAR via **software rasterization (swrast)** when no
  GPU is exposed to Docker → `/scan` runs ~2 Hz instead of 5 Hz. Nav2's `collision_monitor` then
  rejects the stale scans ("invalid source / impossible to transform to base frame") and stop-and-go
  throttles the robot: in `docker/mission_probe.py` (run by `docker/mission_smoke.sh`) the robot navigates (moves ~0.26 m via the full
  mission→Nav2→bus→mediator→`/cmd_vel`→gz chain — integration PROVEN) but rarely *arrives*. The fix
  is GPU-rate LiDAR: run the full navigate-and-spray demo on the **lab laptop / a GPU host** (5 Hz
  scan → Nav2 completes normally). Mission *completion* logic (arrive→spray→treated / fail→skipped /
  stop→pending) is proven hardware-free by the fake-navigator unit tests (`test_mission_runner.py`).
  `docker/sim_smoke.sh` (topic/type/flow check) is the headless acceptance gate; `mission_smoke.sh`
  is the GPU-host end-to-end check.
- **Spray is CLOSED-LOOP on odometry, never a timer.** A fixed-duration spin UNDER-rotates: under
  `use_sim_time` the duration is sim-seconds, and a throttled real-time-factor (or any tracking
  slack) leaves the robot short of N full turns (we measured 1.2 of 3). `mission_runner._spray`
  accumulates wrap-safe `|Δyaw|` from `/dt/odom_active` and spins until the robot has TRULY turned
  `spray_revolutions*360°` — backstop `spray_time_margin ×` nominal so a stalled odom can't loop
  forever. Verified live: measured ~3.00 full revolutions (closed-loop from `/dt/odom_active`), and
  the map-frame view turns ≈3 revs too — the GUI/RViz view shows it.
- **Spin near the hardware max so it's VISIBLE under a throttled sim.** The spray omega is in SIM
  time; the wall-clock rate is `omega × real_time_factor`. On a GPU-less host RTF ≈ 0.5, so
  `spray_omega_radps = 1.0` is only ~0.5 rad/s real — a slow drift that reads as "not rotating" even
  though odom AND the map-frame view both turn the full N revs (the COUNT was right, the RATE wasn't).
  Set `spray_omega_radps ≈ 2.8` with `max_angular_radps = 2.84` (Burger max) → ~1.4 rad/s real, a
  brisk visible spin; Nav2 still plans at `max_vel_theta = 1.0` so nav is unaffected. Diagnose spin
  problems by measuring `/odom` (physical), `/dt/sim_pose` (the map-frame VIEW) and `/joint_states`
  (wheels) separately — they isolate robot-vs-view-vs-throttle.
- **Throttled sim aborts nav with "Failed to make progress".** The ~3.5 Hz software-render LiDAR
  makes the controller sluggish; the stock progress checker (move 0.5 m / 10 s) then aborts before
  the robot settles into the goal → blooms skipped, never sprayed. Loosen it for SIM ONLY via the
  launch `RewrittenYaml` (`movement_time_allowance: 30`, `required_movement_radius: 0.1`); our
  `nav_goal_timeout_s` still bounds a genuinely stuck goal. real_only/both keep the stock values.
- **Goal projection keeps Nav2 off the walls.** A bloom dropped near a wall sits in the costmap
  inflation/lethal zone → Nav2 recovery-churns then aborts ("starts then does nothing"). The mission
  projects each goal to the nearest obstacle-clear cell (`lib/occupancy.reachable_goal`, static map,
  `goal_clearance_m`), capped at `center_tol_m`; un-projectable → skipped immediately (no 60 s hang).
- **Restart-after-complete must NOT depend on `waitUntilNav2Active()`.** The reused `BasicNavigator`
  runs it only on the 2nd+ mission (the 1st creates the navigator lazily and skips it); there it can
  block in `_waitForInitialPose`, republish an all-zero `/initialpose` (wrecking AMCL), or throw under
  executor contention → the mission silently does nothing. Drop the call (Nav2 is already autostarted +
  AMCL seeded), check `goToPose()`'s return (rejected → skip, no stale SUCCEEDED reuse), and spin
  `mission_runner` on its OWN executor so the worker's `BasicNavigator` (which spins the global
  executor) doesn't fight `rclpy.spin`. `/dt/cmd_vel_raw` has 2 publishers (mission + collision_monitor)
  but the latter is SILENT when idle (probe: 0 msgs), so the spray is not diluted after a goal completes.
- **One observer clock for ALL cross-world timing.** Header stamps cross clock domains: the real
  Burger stamps with the Pi's wall clock (un-NTP'd over lab Wi-Fi), the laptop has its own, gz
  stamps with sim time near 0. Differencing them gave 10^12 ms "stop skews" and offset-polluted
  latencies. Measure latency + stop-skew from the supervisor's OWN clock at message ARRIVAL.
  A test that fabricates matching header stamps will happily pin the broken behaviour — write the
  regression test with DELIBERATELY mismatched clock bases instead.
- **Mode-aware required streams.** `real_only` has no sim world: any check that waits for
  `/dt/sim_pose` there (stream-ok, CSV gating) alert-spams at the tick rate forever and starves the
  evidence CSV. Make 'which worlds must report' a function of the mode.
- **RViz default must be mode-aware.** AMCL in `real_only`/`both` can only be seeded via RViz's
  2D Pose Estimate (nothing else publishes `/initialpose`), so `use_rviz` defaults to `auto` = ON
  in those modes. Docs that say "RViz opens with the stack" must be made TRUE by the launch, not
  assumed.
- **The gz 3D client must be non-fatal.** Its big OGRE2 VBO crashes weak GL stacks; with
  `on_exit_shutdown:true` that tore down the whole graded demo. Run the `-g` client with
  `on_exit_shutdown:false` (server stays critical) and thread `gz_gui` through `both` too.
- **Anti-stall ≠ time cap.** The spray's total-time backstop (margin x nominal) fires on a merely
  SLOW sim (zero headroom at RTF 0.5 with margin 2) and skips healthy blooms. The right guard is a
  PROGRESS watchdog (abort when odom yaw hasn't advanced for N wall-seconds); keep a generous cap
  only as belt-and-braces.
- **Check arrival against the goal you SENT.** With goal projection, the projected goal sits up to
  `center_tol_m` from the raw bloom centre; gross-arrival measured to the raw centre falsely
  skipped exactly the near-wall blooms projection exists to save.
- **Latched things must LATCH.** `_estop_battery = crit` re-evaluated per sample silently
  un-latches when a sagging LiPo bounces back over the threshold. Set-on-trip, clear-only-on-RESUME
  (and a still-critical battery re-trips on the next sample, so RESUME can't bypass it).
- **In-process integration tests can starve callbacks.** All harness+node traffic shares one
  test-pumped executor; adding one more 30 Hz publisher made the spray loop's odom view lag and
  over-rotate. Keep harness side-traffic at the lowest realistic rate (and remember `spin_once`
  executes ONE callback — the GUI drains a bounded batch per Qt tick for the same reason).

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
