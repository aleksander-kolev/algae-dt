# PLAN.md — phased implementation to 12/12

Built the course way: **develop at home in `sim_only`; the lab is for testing only.** 7 lab
sessions (Weeks 2–9), **technical reviews Week 4 & Week 8**, **video Week 9**. Everything is TDD on
pure libs, integration-tested in `sim_only` before it ever goes to the lab. Task IDs `T#`.

> **Rubric (confirmed official):** "Implementation of PoC" = 3 criteria × 4 = **12** (Bidirectional,
> Sync, Environmental); aim for the **Redlining (4)** band on each — see `docs/RUBRIC_MAP.md` for the
> exact triggers. **Option A is mandatory** (Option B caps at 7/12). Submission package + demo script:
> `docs/SUBMISSION.md`, `docs/DEMO_SCRIPT.md`. Due **Mon 22 Jun 2026 21:00**.

> **IMPLEMENTATION STATUS — Phases 1→7 DONE, TDD, integration-verified.** All 11 pure libs + 5 nodes
> (+ `fake_robot`, `dynamic_obstacle`) implemented; full `bringup.launch.py` (sim_only|real_only|both
> + headless/use_fake_robot); reproducible `algae-dt:dev` image. **239 tests pass** + clean
> `colcon build`; `sim_only` runs end-to-end headless (Nav2 active, AMCL localized, TwistStamped
> chokepoint); `both` collision-free with **ground-truth sim pose + bounded-drift resync** (live
> round-trip gated by `docker/both_smoke.sh`). Evidence per deliverable → `docs/VERIFICATION.md`.
> Remaining is NON-code: the Week-9 demo video + lab-hardware validation, and the full
> navigate-and-spray demo on a GPU host (headless GPU-less Docker throttles Nav2 at ~2 Hz
> software-render LiDAR — see VERIFICATION).

> Migration note: port reusable logic from the old `algae-twin` repo's pure libs
> (`geometry/safety/blooms/sync/pgm`) — they're ROS-free and already unit-tested — and DROP all the
> robot-description scaffolding (SDF/URDF/`ground_truth_localizer`/custom teleop/Nav2 params), which
> the stock turtlebot3 packages now provide.
>
> Blueprint: this project IS the course **"DT Example with Gazebo and Physical Robot.pdf"** (Lidar
> DT, Mini-Projects 1–3) generalised to autonomy. The provided `tb3_safety_stop` is the seed of our
> `twin_mediator`. Coverage of every course file → `docs/COURSE_COVERAGE.md`.

## Proof of Concept framing (course "Intro to Technical Solutions and the PoC")
This deliverable IS our **Proof of Concept**. After **Phase 3 — Scope of PoC** approval, the feature
list below is the **backlog** (course 2IP90-style Todo). For each backlog item we record: technical
knowledge needed, whether hardware is actually required (most is `sim_only`), and the return on time
(drop a feature rather than sink the project — sunk-cost warning). **Take Option A** (full lab
participation → full rubric potential); Option B caps the score and would be declared in writing.

**Team knowledge prerequisites (acquire before Lab 1):** Linux + CLI, SSH, the **publish/subscribe
pattern** (course **"Simple Publisher & Subscriber.pdf"** + `subscriber_node`/`publisher_node`
downloads — our nodes are exactly pub/sub built on this primitive), ROS 2 topics/nodes, basic Gazebo.

---

## Phase 0 — Environment & skeleton  (before Lab 1; home)
- **T0.1** WSL+Docker + `turtlebot3_ws` image per Canvas PDFs; confirm
  `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` runs. *(done by following SETUP.md)*
- **T0.2** Custom package `algae_dt` created at `~/turtlebot3_ws/src/algae_dt` (this repo's skeleton:
  `package.xml`, `setup.py`, entry points, `config/twin.yaml`, `worlds/algae_arena.world`,
  `maps/map.{pgm,yaml}`, `launch/bringup.launch.py`, `lib/`, `test/`). **Builds clean.** ✅ scaffolded
- **T0.3** `colcon build --packages-select algae_dt` + `pytest` green on the stubs/ported libs.
- **T0.4** **Context diagram** (course "Introduction to Context Diagrams") → `docs/CONTEXT_DIAGRAM.md`:
  the DT system boundary, external entities (operator, real robot, sim, online source), and the
  topic flows in/out of the mediator. Bring it to the Week-4 review as evidence.
- **T0.5** **Per-member readiness gate:** EACH team member independently (a) completes the SETUP §0/§1
  environment, (b) runs `sim_only` from a fresh clone, and (c) does the SETUP §3 connect flow against
  a sim stand-in. Tick per person — this is the real mitigation for "key member absent", not a claim.
- **Exit:** package builds + installs in the container; stubs run and log "not yet implemented";
  context diagram drafted; every member has run `sim_only` solo.

## Phase 1 — Simulation Setup + command bus  (Lab-1 ready; home)  → Rubric ③ start
- **T1.1** `bringup.launch.py mode:=sim_only` brings up `turtlebot3_gazebo` with our
  `algae_arena.world` (gz_sim + spawn + robot_state_publisher + ros_gz bridge; confirm `/clock` and
  `/sim/scan` flow) + `turtlebot3_navigation2` (`use_sim_time:=true`, our `map.yaml`). **Auto-seed
  AMCL** for sim_only: set `set_initial_pose: true` + `initial_pose.{x,y,yaw}` to the Gazebo spawn
  pose in a Nav2 `params_file` (no human 2D-Pose-Estimate at home), else goals plan from an
  unlocalized tree and blooms get skipped. Verify stock launch arg names (`use_sim_time`, `map`,
  `params_file`) empirically in the container.
- **T1.2** Stock `turtlebot3_teleop` remapped `-r /cmd_vel:=/dt/cmd_vel_raw`; confirm the sim robot
  moves via the command bus.
- **T1.3** `lib/safety.py` (porting old) — front-sector min-range, fail-safe staleness. **TDD first.**
- **T1.4** `twin_mediator` v1: subscribe `/dt/cmd_vel_raw` + `/sim/scan`; gate (25 cm); publish
  `/sim/cmd_vel`. Verify sim cmd_vel TYPE empirically (TwistStamped vs Twist) before wiring.
- **Exit / Lab-1 test goal:** teleop → sim moves; box in sim → forward motion blocked, rotation ok.

## Phase 2 — Fan-in/fan-out mediator + bloom mission  (home)  → Rubric ①, ③
- **T2.1** `lib/geometry.py`, `lib/blooms.py` (immutable) ported + TDD (world↔pixel, nearest-untreated).
- **T2.2** `mission_runner`: `BasicNavigator` to each bloom centre (goal projected clear of walls) → 3-spin spray on
  `/dt/cmd_vel_raw`; `/dt/markers`, `/dt/mission_state`; honest accounting (no-spray-on-failure,
  aborted→pending). **Nav2's FINAL velocity is routed to `/dt/cmd_vel_raw` by rewriting
  `collision_monitor.cmd_vel_out_topic` (RewrittenYaml, NOT a SetRemap), NOT on `mission_runner`** (which
  publishes no cmd_vel). **Pass Nav2 a `params_file` setting `enable_stamped_cmd_vel: true`** on
  controller/behavior/velocity_smoother (Nav2 Jazzy defaults to plain `Twist`; the bus is
  `TwistStamped`) — build it from turtlebot3's bundled params via `nav2_common` RewrittenYaml so the
  tuned params are kept (RULES §B-1/B-2).
- **T2.3** `twin_mediator` v2: full fan-out `/dt/cmd_vel_raw`→`/cmd_vel`+`/sim/cmd_vel`; mirrors
  `/odom`→`/dt/real_pose`, sim pose→`/dt/sim_pose`, battery→`/dt/health`; `/dt/scan_active`,
  `/dt/odom_active`; latched `/dt/estop` + auto-E-STOP on critical battery.
- **Exit:** in `sim_only`, place blooms → robot navigates, sprays, marks green; nav-failed→grey;
  E-STOP halts + latches. Integration test with unique `ROS_DOMAIN_ID`.

## Phase 3 — State sync + tolerances + alerts  (home)  → Rubric ② (the differentiator)
- **T3.1** `lib/sync.py` + `lib/metrics.py` (TDD). Define the measurements concretely (don't
  hand-wave the latency number — it's the highest-value graded item):
  - **pose discrepancy** Δxy, Δyaw between `/dt/real_pose` and `/dt/sim_pose`; **sensor delta** front-range.
  - **command→motion latency:** the mediator stamps each `/dt/cmd_vel_raw` it forwards; the supervisor
    marks motion onset when `|odom.twist.linear|>motion_eps_mps` OR `|angular|>motion_eps_radps`
    (params in twin.yaml) and reports `t_motion − t_cmd` (clock = sim time in `sim_only`). Unit-tested.
  - **sim_only sync source (so pillar ② is demonstrable in the fallback env):** with no real robot,
    the "shadow real pose" is the COMMANDED pose integrated from `/dt/cmd_vel_raw`; sync error =
    commanded-vs-achieved sim pose (`sim_only_sync_source: commanded` in twin.yaml) — same trick as
    the synthetic battery.
- **T3.2** `sync_supervisor`: publish `/dt/sync_error`, `/dt/latency_ms`, `/dt/sync_ok`; **append CSV**
  `sync_metrics_<run>.csv`; **publish `/dt/alerts` when out of tolerance**; thresholds (+ rationale)
  from `config/twin.yaml`. Also capture `stop_skew_ms` (real-stop vs sim-stop time delta) on safety
  events → CSV, so pillar ③ "synchronous" is MEASURED, not just asserted.
- **Exit:** induce a discrepancy → `/dt/sync_ok` flips, `/dt/alerts` fires, CSV logs it (in `sim_only`
  via the commanded shadow pose).

## Phase 4 — Operator GUI  (home)
- **T4.1** `lib/pgm.py` ported + TDD (P5/P2 → grayscale). **T4.2** `operator_gui` (PyQt5): map canvas,
  real+sim pose overlay, live `/dt/scan_active`, bloom markers (pending/active/done/grey),
  click-to-place→`/dt/blooms`, Start/Stop/Clear/E-STOP, banners (mode/sync/latency/battery/safety/
  mission — the **battery banner turns amber below `battery_low_v`, red below `battery_critical_v`**).
  Subscribes `/dt/*` only; QTimer `spin_once`. Headless test with `QT_QPA_PLATFORM=offscreen`.
- **Exit:** full `sim_only` demo drivable from the GUI alone. **← Week-4 review bar = Phases 1–4
  complete: bidirectional fan-out + Simulation Setup + a measured sim_only sync number (Phase 3) +
  GUI.** (Phase 3 precedes Phase 4, so finishing Phase 4 guarantees the "basic sync number" the
  Week-4 checkpoint asks for.)

## Phase 5 — `both` mode + real-robot integration  (home build, LAB test)  → all pillars on hardware
- **T5.1** Namespace `turtlebot3_gazebo` to `/sim/*`. This is MORE than a namespace push: the sim
  topics come from the **ros_gz bridge**, so you must **remap each bridged topic** (a `PushRosNamespace`
  won't rename gz-side topics), and decide TF handling. Simplest correct approach for `both`: the sim
  is a **visual mirror only (no sim Nav2)** — drive it via `/sim/cmd_vel` + ground-truth `/sim/odom`,
  keep sim TF on `/sim/tf` OFF the global `/tf`, and avoid frame-name collisions entirely (don't put
  sim `base_link`/`odom`/`map` on the global tree). Hardest integration task — budget a full home session.
- **T5.1b** **Validate the topic-collision rule entirely in sim BEFORE the lab:** port the old repo's
  `fake_robot.py` as a stand-in that publishes the real robot's BARE topics (`/scan /odom /cmd_vel
  /battery_state`), run it alongside the `/sim/*` sim, and confirm no collision + the fan-out hits
  both. This de-risks `both` (and exercises pillar ①'s real→digital/digital→real arrows) without
  spending scarce lab time — and gives a hardware-independent ① demo for the reviews.
- **T5.2** `mode:=both`: real leads (wall time), sim mirrors 1:1 via the mediator fan-out.
- **T5.3** `mode:=real_only`: robot bringup (Pi) + `turtlebot3_navigation2` (AMCL) + DT layer; 2D Pose
  Estimate workflow; gross arrival check disabled (odom-frame).
- **LAB tests:** connect (SETUP §3); teleop→real moves; 25 cm stop on real `/scan`; AMCL localize;
  one bloom navigate+spray; `both` mirrors. Tune inflation at runtime (no rebuild). Re-do 2D Pose
  Estimate after restarts. **← target state for Week-8 review.**

## Phase 6 — Scenario testing, hardening, demo  (home + Lab)
- **T6.1** Scenarios: sensor noise, **dynamic obstacle**, multi-bloom, battery sag. Fix desync/latency
  issues; record metrics. **Make the dynamic obstacle reproducible in `sim_only`** (don't improvise it
  in the lab): either a scripted moving model added to `algae_arena.world`, or a second teleoped box
  entity — so the Nav2 reroute (pillar ③) is demonstrable at home and recorded.
- **T6.1b (OPTIONAL stretch — context-aware DT)** Following the course **"DT Example With Only
  Gazebo.pdf"** (Weather DT) / `tb3_weather_dt`: a `context_adapter` node pulls an online source
  (e.g. Open-Meteo weather, no API key) and adjusts behaviour in BOTH worlds — e.g. bad weather →
  reduce `max_linear_mps` + increase `stop_distance_m` ("cautious spray mode"), shown on a GUI
  banner. Fail-safe: keep last value, then revert to conservative defaults. Only if Phase 1–5 are
  solid. **Constraint: HOME / `sim_only` only — never on lab hardware** (the lab network may block
  outbound calls and an HTTP lib isn't a stock package; per RULES §A-2/§A-3 don't add network deps on
  the lab laptop). This is the FIRST thing cut if a lab session is lost.
- **T6.2** `docs/DEMO_SCRIPT.md` (done): timed 2–3 min script hitting all three Redlining triggers,
  one clean run, **backup plan** (pre-recorded `sim_only` clip if the robot/Wi-Fi misbehaves).
- **T6.3** Week-9 video: clean run + every RUBRIC_MAP evidence clip. **Pre-record the full `sim_only`
  run at home as the guaranteed baseline video BEFORE the hardware phase**, so a submittable video
  exists even if late lab sessions fail; swap in `both`/real footage if it's clean.

## Phase 7 — Bounded-drift resync ("predict with the model, correct with the data")  → Rubric ② DONE
- [x] **T7.1** `twin_resync` + `lib/resync.py` (pure policy: manual-fires-now, auto needs a
  SUSTAINED out-of-tolerance, cooldown spaces attempts, stale inputs refuse) + `lib/gzcli.py`
  (shared `gz service` helpers, extracted from `dynamic_obstacle`). GUI **RESYNC TWIN** button
  (`/dt/resync_cmd`) + `/dt/resync_event` → SYNC banner note + CSV `resync` column.
  `/dt/sim_pose` in `both` switched to **gz GROUND TRUTH** (`worlds/burger_sim_gt.sdf` PosePublisher
  → `/sim/ground_truth`; the scene broadcaster's `dynamic_pose/info` bridges with empty frame names
  — see BEST_APPROACHES §Lessons), which is what makes the teleport genuinely correct pose + LiDAR
  view + measured error together, and solves the `both` start-alignment gap (the first 2D Pose
  Estimate puts the error out of tolerance → the twin auto-snaps to the real start).
  TDD: `test_resync.py` (12) + `test_gzcli.py` (9) + `test_twin_resync.py` (6) + extended
  metrics/supervisor/GUI/mediator tests; end-to-end `docker/both_smoke.sh` (live resync round-trip).

---

## Lab-session schedule (7 sessions, Weeks 2–9) — phase ↔ session, entry criteria
Phases 0–4 are **home/sim_only**; the lab is only needed from Phase 5. Each session: 2–3 members,
roles assigned, a written 30–60 min test plan, code committed + on USB. **Entry criterion** = what
must already work at home before the session is worth spending.

The course grants **7 lab sessions across Weeks 2–9** (8 calendar weeks → one week has no session;
the Week-4 + Week-8 reviews and the Week-9 video are themselves lab slots). **Confirm the exact
week→session calendar on Canvas** and pin the contingency slot to whichever week is free.

| Session | Target week | Lab objective (TESTING) | Entry criterion (home-built FIRST) |
|---|---|---|---|
| 1 | Wk 2 | Familiarisation; SETUP §3 connect; stock teleop → real moves; `ros2 topic hz /scan` | Phase 1 (teleop→sim works); SETUP done by all (T0.5) |
| 2 | Wk 3 | Real 25 cm safety stop on `/scan`; confirm `/cmd_vel` is TwistStamped on real | Phase 2 (mediator fan-out + safety gate in sim) |
| 3 | Wk 4 *(review)* | **Week-4 technical review** evidence: bidirectional + sim + sync number + GUI; light real check | Phases 1–4 complete in `sim_only` |
| 4 | Wk 5 | `real_only`: AMCL **2D-Pose-Estimate (first done here, in the lab)** + one-bloom navigate+spray; inflation tune | `real_only` launch graph + AMCL params verified in **sim**; `both` validated at home with `fake_robot` (T5.1b) |
| 5 | Wk 6 | `both` mode: real leads, sim mirrors 1:1; sync metrics + `/dt/alerts` on hardware | Phase 5 `both` working at home with `fake_robot` |
| 6 | Wk 7 *(contingency)* | **Buffer/hardening** — re-run anything that flaked; dynamic-obstacle + E-STOP on real | Phase 6 scenarios pass in sim |
| 7 | Wk 8 *(review)* → Wk 9 video | **Week-8 review** evidence (full twin, all 3 interactions, CSV+alerts, E-STOP); then the **Week-9 video** clean run | everything green; `sim_only` baseline video already recorded (T6.3) |

> Note: `real_only` is hardware-dependent and cannot be fully built at home — only its launch graph +
> AMCL params are verified in sim; the real AMCL 2D-Pose-Estimate is **first exercised in Session 4**,
> not pre-built. If the course gives a separate Wk-9 slot, that's the 7th physical session and Wk-8 is
> the 6th-plus-review — adjust the table to the Canvas calendar.

**Slack & cut-order (no buffer = high risk with "High-likelihood" robot-unavailability):** Session 6
(Wk 7) is a deliberate **contingency slot** with no new objectives. If a session is lost, cut in this
order: (1) T6.1b online-source stretch, (2) `real_only` polish (keep `both`), (3) extra scenarios —
**never** cut the sync metrics/alerts (pillar ②) or the dual-LiDAR stop (pillar ③). The `sim_only`
demo is always a valid fallback for any review or the video. Take **Option A** (full participation).

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
| cmd_vel type mismatch (no motion) | Med | Bus is `TwistStamped` (Jazzy norm). Set `enable_stamped_cmd_vel:true` on Nav2 via params_file (Nav2 Jazzy defaults to Twist); verify `ros2 topic type /sim/cmd_vel` before wiring the sim fan-out (RULES §B-1/B-2). |
| `both`-mode namespace collision | Med | Topic-collision rule; sim strictly `/sim/*`; sim TF off global `/tf`. |
| Key member absent in lab | Med | 2–3 rotating members; everyone can build+run from SETUP.md; no single owner. |
| Battery sag mid-mission → auto-E-STOP | Low | Start > 12 V; RESUME workflow documented; report swelling to TA. |
| Sunk-cost on a hard feature | Low | Backlog prioritised; `sim_only` + ① already secures a strong baseline before risking hardware. |

## Definition of done (per task)
Failing test written → minimal impl → `colcon build` green → `pytest` green → runs in `sim_only` →
`docs/PLAN.md` ticked → committed. No "done" without a green build + a run.
