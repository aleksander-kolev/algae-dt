# SCENARIOS_LAB.md — running on the lab laptop + real robot

Concise run scenarios. Setup commands → `docs/SETUP.md`. Each scenario: **steps → expected → if-not.**

## S0 · Pre-flight (every session, ~5 min)
1. Lab laptop + robot on Wi-Fi `AP2IRR10`; `export ROS_DOMAIN_ID=<robot#>` everywhere.
2. `ros2 pkg list | grep -E "turtlebot3|nav2_simple_commander"` → all present.
3. `ssh turtlebot@<ip>` → `ros2 launch turtlebot3_bringup robot.launch.py`.
4. On laptop: `ros2 topic hz /scan` (~5 Hz), `ros2 topic type /cmd_vel` → `TwistStamped`.
- **Expected:** topics flow. **If-not:** wrong domain/Wi-Fi, or robot Pi not up — fix before launching.

## S1 · Teleop through the mediator (real)
- Run mediator + `turtlebot3_teleop` remapped `-r /cmd_vel:=/dt/cmd_vel_raw`; drive.
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
- **Expected:** both robots move 1:1; `/dt/sync_ok` green; obstacle in EITHER world stops BOTH;
  `sync_metrics_*.csv` logs Δxy/latency/stop_skew; sim on `/sim/*` only (no collision).
- **If-not:** topic collision (sim not namespaced) / domain mismatch / sim cmd_vel type unverified.

## S5 · State sync & alerts (pillar ②)
- During S4, nudge the sim out of tolerance (or block one scan).
- **Expected:** `/dt/sync_ok` flips, GUI banner amber/red, `/dt/alerts` fires, CSV row logged.
- **If-not:** thresholds in `twin.yaml`; supervisor running; both poses present.

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
