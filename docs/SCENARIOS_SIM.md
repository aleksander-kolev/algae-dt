# SCENARIOS_SIM.md — running in `sim_only` (home, the dev + fallback demo)

No robot needed. `sim_only` is the develop/test target AND the guaranteed demo fallback. Setup →
`docs/SETUP.md` §1. Each scenario: **steps → expected → if-not.**

## S1 · Bring-up
- In the container: `ros2 launch algae_dt bringup.launch.py mode:=sim_only` (add `headless:=true` for
  no display; verify headless with `docker/sim_smoke.sh`).
- **Expected:** Gazebo (`algae_arena.world`) + Nav2 (AMCL auto-seeded, `set_initial_pose:true`) +
  mediator + sync_supervisor + mission_runner + GUI up. **sim_only uses BARE topics** (`/sim/*` is
  `both`-only): `/clock`, `/scan`, `/odom`, `/cmd_vel`(TwistStamped) flow; Nav2 reports "Managed nodes
  are active"; `/amcl_pose` in `map`.
- **If-not:** `pkill -9 -f "gz sim"` and relaunch; check `ros2 pkg list | grep turtlebot3`.

## S2 · Teleop (command bus)
- `ros2 run turtlebot3_teleop teleop_keyboard -r /cmd_vel:=/dt/cmd_vel_raw`; drive.
- **Expected:** sim robot moves; obstacle <0.25 m on `/scan` → forward zeroed (`/dt/safety` True), turn OK.

## S3 · Autonomous bloom + spray (pillar ③)
- GUI → place blooms (open space, ≥0.3 m from walls) → **Start**.
- **Expected:** Nav2 → centre (±0.15 m) → 5 s spin → bloom **green** → next. Unreachable → **grey**.
  Nav-failed/aborted-spray → NOT treated (honest).
- **Note:** run on a **GPU host** so the LiDAR is 5 Hz; on a GPU-less Docker host the ~2 Hz software
  render throttles Nav2 (the robot navigates but rarely arrives — see `docs/VERIFICATION.md`).

## S4 · Dynamic obstacle / live environment change (pillar ③)
- Spawn a NEW obstacle into the running world (live change):
  `ros2 run ros_gz_sim create -world default -name algae_obstacle -x 0.6 -y 0.0 -z 0.25 -file \
   $(ros2 pkg prefix algae_dt)/share/algae_dt/worlds/obstacle_box.sdf` — or move it continuously:
  `ros2 run algae_dt dynamic_obstacle --ros-args -p amplitude:=0.6 -p period:=12.0`.
- **Expected:** the LiDAR sees it → Nav2 reroutes around it; if it blocks <0.25 m, forward stops
  (`/dt/safety` True) then resumes when clear.

## S5 · Sync + tolerances + alerts (pillar ②)
- `sim_only` sync source = commanded-vs-achieved sim pose (no real robot).
- Induce a discrepancy (lag/perturb) → **expected:** `/dt/sync_error`/`/dt/latency_ms` published,
  `/dt/sync_ok` flips, `/dt/alerts` fires, `sync_metrics_*.csv` row logged with `stop_skew_ms`.

## S6 · Bidirectional proof without hardware (pillar ①)
- `ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true` — `fake_robot` publishes
  the bare real topics (`/scan /odom /battery_state /tf`) and integrates the gated `/cmd_vel`; the
  Gazebo mirror runs on `/sim/*`.
- **Expected:** `rqt_graph` shows fan-in/fan-out through the mediator; the bare real topics and the
  `/sim/*` mirror coexist (collision-free); commanding the bus moves the fake robot and its odom
  returns on `/dt/real_pose` (real↔digital), mirrored to `/sim/cmd_vel` (digital→sim). Proven by
  `test_fake_robot.py` (run anywhere, no hardware).

## S7 · E-STOP
- GUI E-STOP (Space/Esc) → **expected:** robot halts, Nav2 goal cancelled, `/dt/estop` latched;
  RESUME clears. Spray interrupted by E-STOP → bloom stays pending.

## S8 · Record the baseline video (T6.3)
- Run S1→S3→S4→S5→S7 in one clean take → the guaranteed backup video before any lab session.

## Edge cases
- **No motion:** Nav2 publishing `Twist` → set `enable_stamped_cmd_vel:true` (Nav2 params_file).
- **AMCL not localized headless:** ensure `setInitialPose`/`set_initial_pose` matches the spawn pose.
- **Headless GUI:** `QT_QPA_PLATFORM=offscreen`. **Flaky full-stack test:** unique `ROS_DOMAIN_ID` + settle.
