# SCENARIOS_SIM.md — running in `sim_only` (home, the dev + fallback demo)

No robot needed. `sim_only` is the develop/test target AND the guaranteed demo fallback. Setup →
`docs/SETUP.md` §1. Each scenario: **steps → expected → if-not.**

## S1 · Bring-up
- In the container: `ros2 launch algae_dt bringup.launch.py mode:=sim_only`.
- **Expected:** Gazebo (`algae_arena.world`) + Nav2 (AMCL auto-seeded via `setInitialPose`/params) +
  mediator + sync_supervisor + mission_runner + GUI up; `/clock`, `/sim/scan`, `/sim/odom` flow.
- **If-not:** `pkill -9 -f "gz sim"` and relaunch; check `ros2 pkg list | grep turtlebot3`.

## S2 · Teleop (command bus)
- `turtlebot3_teleop teleop_keyboard -r /cmd_vel:=/dt/cmd_vel_raw`; drive.
- **Expected:** sim robot moves; obstacle <0.25 m on `/sim/scan` → forward zeroed, turn OK.

## S3 · Autonomous bloom + spray (pillar ③)
- GUI → place blooms (open space) → **Start**.
- **Expected:** Nav2 → centre (±0.15 m) → 5 s spin → bloom **green** → next. Unreachable → **grey**.
  Nav-failed/aborted-spray → NOT treated (honest).

## S4 · Dynamic obstacle (pillar ③)
- Add the scripted/teleoped moving obstacle to the arena mid-mission.
- **Expected:** Nav2 reroutes around it; if it blocks <0.25 m, forward stops then resumes when clear.

## S5 · Sync + tolerances + alerts (pillar ②)
- `sim_only` sync source = commanded-vs-achieved sim pose (no real robot).
- Induce a discrepancy (lag/perturb) → **expected:** `/dt/sync_error`/`/dt/latency_ms` published,
  `/dt/sync_ok` flips, `/dt/alerts` fires, `sync_metrics_*.csv` row logged with `stop_skew_ms`.

## S6 · Bidirectional proof without hardware (pillar ①)
- Run the ported `fake_robot.py` (publishes bare `/scan /odom /cmd_vel /battery_state`) alongside.
- **Expected:** `rqt_graph` shows fan-in/fan-out through the mediator; `ros2 topic echo` confirms
  one topic each direction (real→digital, digital→real, digital→sim, sim→digital).

## S7 · E-STOP
- GUI E-STOP (Space/Esc) → **expected:** robot halts, Nav2 goal cancelled, `/dt/estop` latched;
  RESUME clears. Spray interrupted by E-STOP → bloom stays pending.

## S8 · Record the baseline video (T6.3)
- Run S1→S3→S4→S5→S7 in one clean take → the guaranteed backup video before any lab session.

## Edge cases
- **No motion:** Nav2 publishing `Twist` → set `enable_stamped_cmd_vel:true` (Nav2 params_file).
- **AMCL not localized headless:** ensure `setInitialPose`/`set_initial_pose` matches the spawn pose.
- **Headless GUI:** `QT_QPA_PLATFORM=offscreen`. **Flaky full-stack test:** unique `ROS_DOMAIN_ID` + settle.
