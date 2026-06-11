# VERIFICATION.md — what is implemented, how it's proven, and where

Every deliverable maps to a rubric pillar and to concrete, re-runnable evidence. **The full test
suite passes** (`bash /ci/ci.sh` prints the authoritative count) and
`colcon build --packages-select algae_dt` is clean in the reproducible `algae-dt:dev` image.
These claims describe the **committed HEAD** — re-run the gate after any local change.

## How to reproduce the evidence
```bash
docker build -t algae-dt:dev docker
# 1) clean build + full test suite (unit + in-process rclpy integration), headless:
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/ci.sh
# 2) headless sim_only end-to-end (topics/types/flows, Nav2 active, AMCL localized):
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/sim_smoke.sh
# 3) headless `both` end-to-end (hardware-free): /sim/* mirror collision-free, GROUND-TRUTH sim
#    pose path (PosePublisher -> bridge -> /dt/sim_pose), and a LIVE resync round-trip
#    (/dt/resync_cmd -> gz set_pose on the running server -> /dt/resync_event):
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/both_smoke.sh
# 4) full navigate-and-spray mission (best on a GPU host — see the LiDAR-rate note below):
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/mission_smoke.sh
```

## Pure libraries (no ROS, unit-tested)
| Lib | What | Tests |
|---|---|---|
| `lib/safety.py` | 25 cm dual-LiDAR fail-safe gate (full-width front sector, NaN/inf, wrap-around, stale→blocked, startup→unblocked, OR-block, per-world staleness budgets, bounds-clamped scan wrapper, command shaping) | `test_safety.py` (31) |
| `lib/blooms.py` | immutable Bloom/BloomField, nearest_untreated (active not skipped), terminal states | `test_blooms.py` (13) |
| `lib/sync.py` | pose/sensor error + tolerances + commanded-shadow unicycle | `test_sync.py` (13) |
| `lib/metrics.py` | command→motion latency, inf-safe CSV row/header (+ `resync` column) | `test_metrics.py` (9) |
| `lib/resync.py` | bounded-drift resync policy: manual-now / auto-after-SUSTAINED-breach, cooldown-spaced attempts, stale/NaN refusal, immutable state | `test_resync.py` (12) |
| `lib/gzcli.py` | shared `gz service` CLI helpers (request composition, Boolean-reply parsing, pose proto-text with finite/name validation) | `test_gzcli.py` (9) |
| `lib/geometry.py` | world↔pixel (floor semantics: off-map stays off-map), yaw↔quaternion, angle wrap, shared pose/MapInfo helpers | `test_geometry.py` (14) |
| `lib/pgm.py` | P5/P2 parser incl. the real 86×110 course map | `test_pgm.py` (9) |
| `lib/hud.py` | battery colour thresholds, scan projection, status text | `test_hud.py` (5) |
| `lib/occupancy.py` | static-map goal projection / `reachable_goal` (off-wall + edge-aware clearance, off-map rejection) + `raycast_scan` (the fake robot's map-true synthetic LiDAR: LDS-02 blind-spot 0.0, no-return inf, range_max, yaw geometry — incl. on the real course map) | `test_occupancy.py` (21) |
| `lib/trajectory.py` | dynamic-obstacle sinusoidal sweep + spawn keep-out clamp | `test_trajectory.py` (10) |

## Pillar ① — Bidirectional (fan-in / fan-out)
- **`twin_mediator`** is the single command chokepoint: subscribes `/dt/cmd_vel_raw` (TwistStamped)
  + scans, gates once, fans out to `/cmd_vel` (active) **and** `/sim/cmd_vel` (mirror, `both`).
  Mirrors `/odom`→`/dt/{real,sim}_pose`, republishes `/dt/scan_active` + `/dt/odom_active`.
- **Hardware-free proof:** `test_fake_robot.py::test_bidirectional_loop_through_mediator` — commanding
  the bus moves the (fake) real robot and its odom returns on `/dt/real_pose` (digital→real→digital).
- **`both` collision-free** (real bare vs sim `/sim/*`): verified live by the both-mode topic check
  (`/scan`+`/sim/scan`, `/cmd_vel`+`/sim/cmd_vel` both TwistStamped, distinct), now AUTOMATED by
  `docker/both_smoke.sh` (node liveness, `/sim/ground_truth` type + flow, `/dt/sim_pose` /
  `/dt/real_pose` / `/dt/sync_error` flowing, live resync round-trip) — re-run it after touching
  `_sim_mirror` (the sim RSP remaps `/tf` `/tf_static` `/robot_description` `/joint_states` onto
  `/sim/*`).
- Mediator integration: `test_mediator.py` (10).

## Pillar ② — Synchronization of states (the differentiator)
- **`sync_supervisor`** publishes the *measured* `/dt/sync_error` (Δxy, Δyaw, sensor), `/dt/latency_ms`
  (command→motion), latched `/dt/sync_ok`, and `/dt/alerts` when out of the documented `twin.yaml`
  tolerances; appends a flushed `sync_metrics_*.csv` with `stop_skew_ms`.
- **Drift is BOUNDED, not just measured (`both`):** `twin_resync` snaps the sim entity onto
  `/dt/real_pose` via gz `set_pose` — GUI **RESYNC TWIN** button or auto after a sustained
  out-of-tolerance (policy pure + unit-tested, `lib/resync.py`); every execution publishes
  `/dt/resync_event`, stamped into the CSV `resync` column; a failed gz call raises `/dt/alerts`.
  Genuine because `/dt/sim_pose` in `both` is **gz GROUND TRUTH** (`worlds/burger_sim_gt.sdf`
  PosePublisher → `/sim/ground_truth`): the teleport moves pose + LiDAR view + measured error
  together. Live round-trip proven by `docker/both_smoke.sh`.
- **Proof:** `test_sync_supervisor.py` (12) — in-tolerance vs out-of-tolerance flip + alert, latency
  measured, CSV written with the documented columns incl. the one-shot `resync` stamp;
  `test_twin_resync.py` (6) — manual/auto/transient/failure/mode-guard/stale-input wiring.
  sim_only sync source = commanded-shadow vs achieved sim pose (mediator integrates the gated
  command into `/dt/real_pose`).

## Pillar ③ — Environmental (safety + autonomy + live change)
- **25 cm dual-LiDAR stop on BOTH worlds, fail-safe** — `test_safety.py` (gate logic) +
  `test_mediator.py` (10) (live zeroing of forward + `/dt/safety`) +
  `test_fake_robot.py::test_front_obstacle_triggers_safety_stop_through_mediator` (hardware-free).
- **Navigate-and-spray with honest accounting** — `test_mission_runner.py` (20): SUCCEEDED→spray+treated,
  nav-fail→skipped (no spray), Stop/E-STOP mid-nav→pending.
- **Live environment change / dynamic obstacle** — `worlds/obstacle_box.sdf` + `dynamic_obstacle`
  (sweep via gz set_pose; path unit-tested in `test_trajectory.py`). Spawn/move it mid-run → the
  LiDAR sees it → gate stops / Nav2 reroutes.

## Integration verified headless (`docker/sim_smoke.sh`)
`sim_only` launches the full stock turtlebot3 stack + Nav2 + our DT layer: Nav2 lifecycle **active**,
AMCL **auto-localized** at the spawn (`set_initial_pose` rewrite), `/cmd_vel` **and** `/dt/cmd_vel_raw`
both **TwistStamped** (the chokepoint), `/scan` flowing, the mediator publishing gated `/cmd_vel`,
`/dt/mode`, `/dt/safety`, `/dt/sim_pose`, synthetic `/dt/health` battery.

## Known environmental limit (honest)
On a **GPU-less Docker host** the Gazebo LiDAR renders via software rasterization (~2 Hz instead of
5 Hz), so Nav2's `collision_monitor` rejects stale scans and stop-and-go throttles autonomy: the
robot **navigates** (moves via the full mission→Nav2→bus→mediator→`/cmd_vel`→gz chain — integration
proven by `mission_probe.py` (run by `docker/mission_smoke.sh`)) but rarely *arrives* headless. Run the full navigate-and-spray demo on
a **GPU host / the lab laptop** (5 Hz scan → Nav2 completes normally). Mission *completion* logic is
proven hardware-free by the fake-navigator unit tests. See `docs/BEST_APPROACHES.md` §Lessons.
