# PLAN.md — phased implementation to 12/12

Built the course way: **develop at home in `sim_only`; the lab is for testing only.** 7 lab
sessions (Weeks 2–9), **technical reviews Week 4 & Week 8**, **video Week 9**. Everything is TDD on
pure libs, integration-tested in `sim_only` before it ever goes to the lab. Task IDs `T#`.

> Migration note: port reusable logic from the old `algae-twin` repo's pure libs
> (`geometry/safety/blooms/sync/pgm`) — they're ROS-free and already unit-tested — and DROP all the
> robot-description scaffolding (SDF/URDF/`ground_truth_localizer`/custom teleop/Nav2 params), which
> the stock turtlebot3 packages now provide.

---

## Phase 0 — Environment & skeleton  (before Lab 1; home)
- **T0.1** WSL+Docker + `turtlebot3_ws` image per Canvas PDFs; confirm
  `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` runs. *(done by following SETUP.md)*
- **T0.2** Custom package `algae_dt` created at `~/turtlebot3_ws/src/algae_dt` (this repo's skeleton:
  `package.xml`, `setup.py`, entry points, `config/twin.yaml`, `worlds/algae_arena.world`,
  `maps/map.{pgm,yaml}`, `launch/bringup.launch.py`, `lib/`, `test/`). **Builds clean.** ✅ scaffolded
- **T0.3** `colcon build --packages-select algae_dt` + `pytest` green on the stubs/ported libs.
- **Exit:** package builds + installs in the container; stubs run and log "not yet implemented".

## Phase 1 — Simulation Setup + command bus  (Lab-1 ready; home)  → Rubric ③ start
- **T1.1** `bringup.launch.py mode:=sim_only` brings up `turtlebot3_gazebo` with our
  `algae_arena.world` + `turtlebot3_navigation2` (`use_sim_time:=true`, our `map.yaml`).
- **T1.2** Stock `turtlebot3_teleop` remapped `-r /cmd_vel:=/dt/cmd_vel_raw`; confirm the sim robot
  moves via the command bus.
- **T1.3** `lib/safety.py` (porting old) — front-sector min-range, fail-safe staleness. **TDD first.**
- **T1.4** `twin_mediator` v1: subscribe `/dt/cmd_vel_raw` + `/sim/scan`; gate (25 cm); publish
  `/sim/cmd_vel`. Verify sim cmd_vel TYPE empirically (TwistStamped vs Twist) before wiring.
- **Exit / Lab-1 test goal:** teleop → sim moves; box in sim → forward motion blocked, rotation ok.

## Phase 2 — Fan-in/fan-out mediator + bloom mission  (home)  → Rubric ①, ③
- **T2.1** `lib/geometry.py`, `lib/blooms.py` (immutable) ported + TDD (world↔pixel, nearest-untreated).
- **T2.2** `mission_runner`: `BasicNavigator` to each bloom centre → 5 s spin-spray on
  `/dt/cmd_vel_raw`; `/dt/markers`, `/dt/mission_state`; honest accounting (no-spray-on-failure,
  aborted→pending). Remap Nav2 controller `cmd_vel`→`/dt/cmd_vel_raw`; `enable_stamped_cmd_vel:=true`.
- **T2.3** `twin_mediator` v2: full fan-out `/dt/cmd_vel_raw`→`/cmd_vel`+`/sim/cmd_vel`; mirrors
  `/odom`→`/dt/real_pose`, sim pose→`/dt/sim_pose`, battery→`/dt/health`; `/dt/scan_active`,
  `/dt/odom_active`; latched `/dt/estop` + auto-E-STOP on critical battery.
- **Exit:** in `sim_only`, place blooms → robot navigates, sprays, marks green; nav-failed→grey;
  E-STOP halts + latches. Integration test with unique `ROS_DOMAIN_ID`.

## Phase 3 — State sync + tolerances + alerts  (home)  → Rubric ② (the differentiator)
- **T3.1** `lib/sync.py` + `lib/metrics.py` (TDD): pose discrepancy (Δxy, Δyaw), sensor delta,
  latency (command→motion, scan age), tolerance comparison, CSV row formatting.
- **T3.2** `sync_supervisor`: publish `/dt/sync_error`, `/dt/latency_ms`, `/dt/sync_ok`; **append CSV**
  `sync_metrics_<run>.csv`; **publish `/dt/alerts` when out of tolerance**; thresholds from
  `config/twin.yaml` (documented with rationale).
- **Exit:** induce a discrepancy → `/dt/sync_ok` flips, `/dt/alerts` fires, CSV logs it.

## Phase 4 — Operator GUI  (home)
- **T4.1** `lib/pgm.py` ported + TDD (P5/P2 → grayscale). **T4.2** `operator_gui` (PyQt5): map canvas,
  real+sim pose overlay, live `/dt/scan_active`, bloom markers (pending/active/done/grey),
  click-to-place→`/dt/blooms`, Start/Stop/Clear/E-STOP, banners (mode/sync/latency/battery/safety/
  mission). Subscribes `/dt/*` only; QTimer `spin_once`. Headless test with `QT_QPA_PLATFORM=offscreen`.
- **Exit:** full `sim_only` demo drivable from the GUI alone. **← target state for Week-4 review.**

## Phase 5 — `both` mode + real-robot integration  (home build, LAB test)  → all pillars on hardware
- **T5.1** Namespace `turtlebot3_gazebo` to `/sim/*` (push namespace + bridge remaps; sim TF→`/sim/tf`).
  Enforce the topic-collision rule. Hardest integration task — budget a full home session.
- **T5.2** `mode:=both`: real leads (wall time), sim mirrors 1:1 via the mediator fan-out.
- **T5.3** `mode:=real_only`: robot bringup (Pi) + `turtlebot3_navigation2` (AMCL) + DT layer; 2D Pose
  Estimate workflow; gross arrival check disabled (odom-frame).
- **LAB tests:** connect (SETUP §3); teleop→real moves; 25 cm stop on real `/scan`; AMCL localize;
  one bloom navigate+spray; `both` mirrors. Tune inflation at runtime (no rebuild). Re-do 2D Pose
  Estimate after restarts. **← target state for Week-8 review.**

## Phase 6 — Scenario testing, hardening, demo  (home + Lab)
- **T6.1** Scenarios: sensor noise, dynamic obstacle (person walks in), multi-bloom, battery sag.
  Fix desync/latency issues; record metrics.
- **T6.2** `docs/demo_script.md`: the algae-cleaning story, one clean run, **backup plan** (sim-only
  fallback if the robot/Wi-Fi misbehaves).
- **T6.3** Week-9 video: clean run + every RUBRIC_MAP evidence clip.

---

## Test plan (what "tested" means)
- **Unit (pure libs, ≥80%):** geometry, safety (incl. fail-safe + dual-scan), blooms (immutability,
  nearest-untreated, no-skip-half-treated), sync/metrics (tolerance + latency + CSV), pgm parser.
- **Integration (`sim_only`, in-process + full-stack):** mediator fan-out types; mission
  success/failure/abort marker colours; E-STOP latch; sync alert on induced discrepancy. Unique
  `ROS_DOMAIN_ID` per full-stack test + settle; fresh container per full suite.
- **Lab (hardware):** the SETUP §3 smoke tests + each RUBRIC_MAP clip. Lab time is testing only.

## Risk assessment (course explicitly requires this — ⚠️ on Lab Advice)
| Risk | Likelihood | Mitigation |
|---|---|---|
| Limited/again-unavailable robot time | High | Everything works in `sim_only`; `sim_only` is a valid demo fallback. Lab only validates real/both. |
| Lab laptop factory-reset / wiped | Med | Commit to git + OneDrive every session; full-package copy rebuild is one command (SETUP §2). |
| Wi-Fi/`ROS_DOMAIN_ID` mismatch → no topics | Med | SETUP §3 smoke test first; never change network settings. |
| Sim `/cmd_vel` type mismatch (no motion) | Med | Verify `ros2 topic type` before wiring fan-out; `enable_stamped_cmd_vel:=true`. |
| `both`-mode namespace collision | Med | Topic-collision rule; sim strictly `/sim/*`; sim TF off global `/tf`. |
| Key member absent in lab | Med | 2–3 rotating members; everyone can build+run from SETUP.md; no single owner. |
| Battery sag mid-mission → auto-E-STOP | Low | Start > 12 V; RESUME workflow documented; report swelling to TA. |
| Sunk-cost on a hard feature | Low | Backlog prioritised; `sim_only` + ① already secures a strong baseline before risking hardware. |

## Definition of done (per task)
Failing test written → minimal impl → `colcon build` green → `pytest` green → runs in `sim_only` →
`docs/PLAN.md` ticked → committed. No "done" without a green build + a run.
