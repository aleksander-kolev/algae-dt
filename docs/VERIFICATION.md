# VERIFICATION.md — what is implemented, how it's proven, and where

Every deliverable maps to a rubric pillar and to concrete, re-runnable evidence. **145 tests pass**
and `colcon build --packages-select algae_dt` is clean in the reproducible `algae-dt:dev` image.

## How to reproduce the evidence
```bash
docker build -t algae-dt:dev docker
# 1) clean build + full test suite (unit + in-process rclpy integration), headless:
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/ci.sh
# 2) headless sim_only end-to-end (topics/types/flows, Nav2 active, AMCL localized):
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/sim_smoke.sh
# 3) full navigate-and-spray mission (best on a GPU host — see the LiDAR-rate note below):
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/mission_smoke.sh
```

## Pure libraries (no ROS, unit-tested)
| Lib | What | Tests |
|---|---|---|
| `lib/safety.py` | 25 cm dual-LiDAR fail-safe gate (full-width front sector, NaN/inf, wrap-around, stale→blocked, startup→unblocked, OR-block, command shaping) | `test_safety.py` (25) |
| `lib/blooms.py` | immutable Bloom/BloomField, nearest_untreated (active not skipped), terminal states | `test_blooms.py` (13) |
| `lib/sync.py` | pose/sensor error + tolerances + classify + commanded-shadow unicycle | `test_sync.py` (13) |
| `lib/metrics.py` | command→motion latency, data age, inf-safe CSV row/header | `test_metrics.py` (7) |
| `lib/geometry.py` | world↔pixel, yaw↔quaternion, angle wrap | `test_geometry.py` (11) |
| `lib/pgm.py` | P5/P2 parser incl. the real 86×110 course map | `test_pgm.py` (9) |
| `lib/hud.py` | battery colour thresholds, scan projection, status text | `test_hud.py` (5) |
| `lib/occupancy.py` | static-map goal projection / `reachable_goal` (off-wall goal clearance) | `test_occupancy.py` (11) |
| `lib/trajectory.py` | dynamic-obstacle sinusoidal sweep | `test_trajectory.py` (4) |

## Pillar ① — Bidirectional (fan-in / fan-out)
- **`twin_mediator`** is the single command chokepoint: subscribes `/dt/cmd_vel_raw` (TwistStamped)
  + scans, gates once, fans out to `/cmd_vel` (active) **and** `/sim/cmd_vel` (mirror, `both`).
  Mirrors `/odom`→`/dt/{real,sim}_pose`, republishes `/dt/scan_active` + `/dt/odom_active`.
- **Hardware-free proof:** `test_fake_robot.py::test_bidirectional_loop_through_mediator` — commanding
  the bus moves the (fake) real robot and its odom returns on `/dt/real_pose` (digital→real→digital).
- **`both` collision-free** (real bare vs sim `/sim/*`): verified live by the both-mode topic check
  (`/scan`+`/sim/scan`, `/cmd_vel`+`/sim/cmd_vel` both TwistStamped, distinct).
- Mediator integration: `test_mediator.py` (8).

## Pillar ② — Synchronization of states (the differentiator)
- **`sync_supervisor`** publishes the *measured* `/dt/sync_error` (Δxy, Δyaw, sensor), `/dt/latency_ms`
  (command→motion), latched `/dt/sync_ok`, and `/dt/alerts` when out of the documented `twin.yaml`
  tolerances; appends a flushed `sync_metrics_*.csv` with `stop_skew_ms`.
- **Proof:** `test_sync_supervisor.py` (8) — in-tolerance vs out-of-tolerance flip + alert, latency
  measured, CSV written with the documented columns. sim_only sync source = commanded-shadow vs
  achieved sim pose (mediator integrates the gated command into `/dt/real_pose`).

## Pillar ③ — Environmental (safety + autonomy + live change)
- **25 cm dual-LiDAR stop on BOTH worlds, fail-safe** — `test_safety.py` (gate logic) +
  `test_mediator.py` (8) (live zeroing of forward + `/dt/safety`) +
  `test_fake_robot.py::test_front_obstacle_triggers_safety_stop_through_mediator` (hardware-free).
- **Navigate-and-spray with honest accounting** — `test_mission_runner.py` (16): SUCCEEDED→spray+treated,
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
