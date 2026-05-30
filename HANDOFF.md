# HANDOFF.md — start here (resuming AI)

You are taking over **`algae-dt`**, a TU/e 2IRR10 TurtleBot3 **digital-twin** PoC. This file is your
entry point. **Read in this order:** `HANDOFF.md` (this) → `CLAUDE.md` → `docs/DECISIONS.md` →
`docs/PLAN.md` → `docs/RULES.md`. Then implement, TDD, verified in `sim_only`.

## Mission (what 100% looks like)
Earn **Redlining 4/4 on all three rubric criteria = 12/12** on "Implementation of PoC", under
**Option A**, and produce a Redlining-quality **2–3 min demo video** (the assignment is 30 pts total;
the implementation rubric is the 12, the rest is video/presentation — see `docs/RUBRIC_MAP.md` +
`docs/SUBMISSION.md`). Due **Mon 22 Jun 2026 21:00**.

The three criteria → our deliverables:
- **① Bidirectional** — `twin_mediator` fan-in/fan-out: `/scan`→twin, `/cmd_vel`→robot,
  `/sim/cmd_vel`→sim, **+ internal status topic** (`/dt/sync_ok`,`/dt/mode`).
- **② Sync of states** — `sync_supervisor`: pose + **internal state that affects behavior**
  (battery→auto-E-STOP, mode) mirrored near-real-time, measured error/latency + tolerances + alerts + CSV.
- **③ Environmental** — 25 cm dual-LiDAR stop mirrored on BOTH robots + Nav2 dynamic-obstacle avoidance
  + navigate-and-spray; **introduce a live environment change** in the demo.

## State: DONE — Phases 1→6 implemented, TDD, and integration-verified (see `docs/VERIFICATION.md`)
**DONE:** 8 pure libs (safety/blooms/sync/metrics/geometry/pgm/hud/trajectory) + 4 nodes
(twin_mediator, sync_supervisor, mission_runner, operator_gui) + `fake_robot` + `dynamic_obstacle`;
full `bringup.launch.py` (sim_only | real_only | both, with `headless`/`use_rviz`/`use_fake_robot`);
`/sim/*` bridge for `both`; reproducible **`algae-dt:dev`** Docker image (`docker/`, `docs/DOCKER.md`).
**103 tests pass** + clean `colcon build`. The five old open items are RESOLVED & verified in-container:
(1) `mode:=sim_only` smoke-tested (Nav2 active + AMCL localized, `/scan` flows) — `docker/sim_smoke.sh`;
(2) gz↔ROS bridge confirmed (the stock spawn bundles `parameter_bridge`; `/scan /odom /cmd_vel /clock`
reach ROS); (3) `both` `/sim/*` namespacing done (custom bridge + namespaced RSP, collision-free);
(4) Nav2 rewrite keys (`collision_monitor.cmd_vel_out_topic`, `set_initial_pose`) verified present in
`burger.yaml`; (5) the cmd_vel chokepoint is TwistStamped end-to-end.

**REMAINING (environment/process — not code):**
- Full **navigate-and-spray COMPLETION is throttled on a GPU-less Docker host** (~2 Hz software-render
  LiDAR → Nav2 `collision_monitor` rejects stale scans). The robot navigates via the full chain
  (proven); run the full demo on a **GPU host / the lab laptop** (5 Hz scan). Completion logic is
  unit-proven (`test_mission_runner.py`). Details: `docs/VERIFICATION.md` + BEST_APPROACHES §Lessons.
- **Record the Week-9 demo video** (`docs/DEMO_SCRIPT.md`) on a GPU host, and **validate on lab
  hardware** (`real_only`/`both` on the Burger; `scripts/lab_run.sh`, `docs/RUN_ON_LAB_PC.md`).

> ⚠️ **Shared branch:** this branch (`feat/poc-implementation`) also received `feat(lab)` commits
> (lab_run.sh / RUN_ON_LAB_PC.md) from a parallel effort. If two sessions edit at once, coordinate to
> avoid clobbering the working tree.

## What's next (implementation is done — these are lab/demo steps)
1. **GPU host:** `bash docker/run.sh` → inside, `ros2 launch algae_dt bringup.launch.py mode:=sim_only`
   (display). Drive the GUI, place blooms, Start; record the baseline video per `docs/DEMO_SCRIPT.md`.
   Hardware-free `both`: `mode:=both use_fake_robot:=true`.
2. **Lab:** connect to the Burger (SETUP §3), then `mode:=real_only` / `mode:=both`
   (`scripts/lab_run.sh`, `docs/RUN_ON_LAB_PC.md`). Validate the 25 cm stop on real `/scan`, AMCL
   2D-Pose-Estimate, one bloom navigate+spray, the `both` mirror.
3. **Verify anytime:** `docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash
   /ci/ci.sh` (103 tests + build). Evidence map: `docs/VERIFICATION.md`.

> Re-implementing? The TDD recipe still holds: failing test (pure libs) → minimal impl → `colcon
> build --packages-select algae_dt` → `pytest src/algae_dt/test` → run in `sim_only` → commit.

## HARD INVARIANTS (read `docs/RULES.md` — breaking these fails silently)
1. **Bus = `TwistStamped`**; mediator forwards to real `/cmd_vel` (TwistStamped); set Nav2
   `enable_stamped_cmd_vel: true` via params_file. (Do NOT switch to plain Twist — see D3.)
2. **Topic-collision rule:** real = bare topics, sim = `/sim/*`, sim TF off global `/tf` in `both`.
3. **`use_sim_time: true` only in `sim_only`** (bool to node params, lowercase string to includes).
4. **Safety gate fails SAFE** (stale considered-scan → blocked; startup no-data → unblocked); EITHER
   scan <0.25 m zeroes forward on BOTH.
5. **Nav failure / aborted spray must NOT mark a bloom treated** (honest accounting).
6. **No sudo / no settings changes on lab kit**; build only `--packages-select algae_dt`.

## Do NOT regress (settled — see `docs/DECISIONS.md`)
- Don't hand-build SDF/URDF/`robot_state_publisher`/teleop/Nav2 params — use stock packages (D2).
- Don't make the bus plain Twist (D3). Don't split into 2 packages (D4). Don't put `name=` on
  `mission_runner`'s launch Node (process-wide remap trap). Don't remap cmd_vel on `mission_runner`
  (it publishes none — remap Nav2 via SetRemap in the launch).

## Definition of done (per task) + verify
Failing test → minimal impl → `colcon build --packages-select algae_dt` clean → `pytest src/algae_dt/test`
green → runs in `sim_only` → tick the task in `docs/PLAN.md` → commit. **Never claim done without a
green build + a run.** Build/test recipe: `docs/SETUP.md` §1 (container) or §2 (lab).

## Doc index (everything is in md — nothing lives only in chat)
| File | Purpose |
|---|---|
| `CLAUDE.md` | What it is, course method, architecture, topic contract, lab facts. |
| `docs/DECISIONS.md` | Why — settled decisions + forensics (turtlebot3, cmd_vel, restart). |
| `docs/PLAN.md` | Phased TDD plan (T-IDs), 7-session schedule, risk, definition-of-done. |
| `docs/RULES.md` | Hard constraints: golden rules + technical invariants + coding style. |
| `docs/SETUP.md` | WSL+Docker / lab-laptop / robot-connection commands. |
| `docs/BEST_APPROACHES.md` | Patterns, lessons, gotchas, + Context7 API verification. |
| `docs/RUBRIC_MAP.md` | Official rubric → deliverable → evidence → demo (Redlining targets). |
| `docs/SCENARIOS_SIM.md` / `docs/SCENARIOS_LAB.md` | Run scenarios (flow + edge cases). |
| `docs/RUN_ON_LAB_PC.md` | Pull-from-GitHub → build → run on the lab laptop (+ readiness verdict). |
| `docs/CONTEXT_DIAGRAM.md` | System boundary + topic flows. |
| `docs/SUBMISSION.md` / `docs/DEMO_SCRIPT.md` | Zip/checklist/credentials + 2–3 min video script. |
| `docs/COURSE_COVERAGE.md` | Every course artifact → where used. |

## Environment quick ref
Everything runs in the course `turtlebot3_ws` Docker container (turtlebot3 is in the image, NOT the
bare host — D1). **One command, from a fresh clone:** `./scripts/lab_run.sh sim_only` (home) or
`ROS_DOMAIN_ID=<robot#> ./scripts/lab_run.sh real_only` (lab) — it recreates `~/turtlebot3_ws`, copies
the package, builds, and launches. Full flow + provenance: `docs/RUN_ON_LAB_PC.md`. Robot: Wi-Fi
`AP2IRR10`, `ssh turtlebot@<ip>` → `turtlebot3_bringup robot.launch.py`.
