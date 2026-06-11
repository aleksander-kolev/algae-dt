# SCENARIOS_LAB.md — running on the lab laptop + real robot

Concise run scenarios. Setup commands → `docs/SETUP.md`. Each scenario: **steps → expected → if-not.**

## S0 · Pre-flight (every session, ~5 min)
1. Lab laptop + robot on Wi-Fi `AP2IRR10`; `export ROS_DOMAIN_ID=<robot#>` everywhere.
2. The §0 dep checklist from `docs/RUN_ON_LAB_PC.md` → no `MISSING:` lines (`turtlebot3_*`,
   `nav2_bringup/_common/_simple_commander`, `ros_gz_sim/_bridge`, `rviz2`, `PyQt5`).
3. `ssh turtlebot@<ip>` → `ros2 launch turtlebot3_bringup robot.launch.py`.
4. On laptop: `ros2 topic hz /scan` (~5 Hz), `ros2 topic type /cmd_vel` → `TwistStamped`.
- **Expected:** topics flow. **If-not:** wrong domain/Wi-Fi, or robot Pi not up — fix before launching.

## S1 · Teleop through the mediator (real)
- Run mediator + teleop: `ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/dt/cmd_vel_raw`; drive.
- **Expected:** robot moves; `/cmd_vel` is `TwistStamped`; obstacle <0.25 m → forward zeroed, turn OK.
- **If-not:** see S-EDGE-A (no motion).

## S2 · Safety stop (real, pillar ③)
- Drive toward a box.
- **Expected:** forward motion stops at ~0.25 m on `/scan`; `/dt/safety=true (blocked)`; rotate/back-up still work.
- **If-not:** check `/dt/scan_active` populated; `stop_distance_m`; front-sector convention.

## S3 · `real_only` autonomous bloom (Nav2 + spray)
1. `ros2 launch algae_dt bringup.launch.py mode:=real_only` (RViz opens automatically in this mode);
   in RViz set **2D Pose Estimate** on the robot's real spot; wait for AMCL lock.
2. GUI → place 1 bloom in open space (≥0.3 m from walls) → **Start**.
- **Expected:** Nav2 drives to centre (±0.15 m), spins 3 full turns, bloom→green, mission complete.
- **If-not:** bloom **grey** = unreachable (move from wall) / not localized (redo 2D Pose Estimate
  BEFORE Start) / Nav2 timeout. Half-spray interrupted → stays **pending** (honest).

## S4 · `both` mode — twin mirror (pillars ①②③)
- `./scripts/lab_run.sh` (full `both` demo); real leads, sim mirrors. Place blooms → Start.
- **Start-alignment is automatic:** the sim always spawns at the map origin while the real Burger
  stands wherever it stands — after the 2D Pose Estimate the pose error exceeds tolerance, and
  twin_resync **auto-snaps the sim onto the real pose** within ~`resync_sustain_s` (or press
  **RESYNC TWIN** immediately). Verify `/dt/sync_ok` goes green before Start.
- **Expected:** both robots move 1:1; `/dt/sync_ok` green; obstacle in EITHER world stops BOTH;
  `sync_metrics_*.csv` logs Δxy/latency/stop_skew/resync; sim on `/sim/*` only (no collision).
- **If-not:** topic collision (sim not namespaced) / domain mismatch / sim cmd_vel type unverified.

## S5 · State sync, alerts & resync (pillar ②)
- During S4, nudge the sim out of tolerance (or block one scan). Let drift accumulate (the spray
  spins are the biggest generator) or carry the real robot half a metre.
- **Expected:** `/dt/sync_ok` flips, GUI banner amber/red, `/dt/alerts` fires, CSV row logged;
  after `resync_sustain_s` of sustained breach the twin **auto-resyncs** (sim teleports onto the
  real pose, error collapses on the live banner, the CSV row carries `auto` in its `resync`
  column). **RESYNC TWIN** does the same on demand — both are demo beats, not failures.
- **If-not:** thresholds in `twin.yaml`; supervisor + twin_resync running; both poses present;
  a `RESYNC FAILED` alert = the gz set_pose call (sim up? world name `default`?).

## S6 · E-STOP (safety headline)
- Hit E-STOP (button / Space / Esc) mid-mission.
- **Expected:** both robots halt instantly, Nav2 goal cancelled, `/dt/estop=true` latched; RESUME recovers.
- **If-not:** check `/dt/estop` latched publisher + mission/GUI subscribe.

## Edge cases
- **S-EDGE-A no motion:** Nav2 publishing `Twist` not `TwistStamped` → set `enable_stamped_cmd_vel:true`
  on Nav2 (params_file). `ros2 topic type /dt/cmd_vel_raw /cmd_vel` to diagnose.
- **S-EDGE-B battery sag:** real V ≤ 10.5 → auto-E-STOP latches mid-mission. Start >12 V; RESUME after.
- **S-EDGE-C AMCL re-seed:** any AMCL/Nav2 restart re-seeds to map origin → redo 2D Pose Estimate.
- **S-EDGE-D Wi-Fi drop / scan stale:** safety gate fails SAFE (stale considered-scan → blocked).
- **S-EDGE-E robot unavailable:** fall back to `sim_only` (S in SCENARIOS_SIM.md) — valid demo.
- **Shutdown:** `ssh turtlebot@<ip> 'sudo shutdown now'` BEFORE the power switch.
