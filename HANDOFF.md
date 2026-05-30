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

## State: DONE vs NOT DONE  (as of commit on `main`; repo: github.com/aleksander-kolev/algae-dt)
**DONE:** all docs; **7 pure libs implemented + TDD** (safety/blooms/sync/metrics/geometry/pgm/hud —
**82 tests pass**); **4 nodes implemented** (twin_mediator, sync_supervisor, mission_runner,
operator_gui); **`bringup.launch.py` fully wired** (gz sim into our world + Nav2 via `RewrittenYaml`
+ DT nodes); Docker dev image (`docker/Dockerfile`); submission package + demo script.
**NOT DONE / AT RISK (your job) — the integrated launch has NEVER run in ROS/Gazebo; only unit tests
pass.** Before trusting it (see `docs/RUN_ON_LAB_PC.md` §0 readiness):
1. **Smoke-test `mode:=sim_only` in the turtlebot3_ws container** (`colcon build` + launch + `ros2 topic hz /scan`).
2. **gz↔ROS bridge:** the launch starts `gz_sim`+spawn but no explicit `ros_gz parameter_bridge` —
   confirm `/scan`,`/odom`,`/cmd_vel`,`/clock` reach ROS; if not, add a bridge (or use turtlebot3_gazebo's
   world launch which bundles it).
3. **`both`-mode `/sim/*` namespacing (PLAN T5.1):** the mediator expects the mirror sim on
   `/sim/scan`,`/sim/odom` and publishes `/sim/cmd_vel`, but the launch spawns the sim on BARE topics →
   collides with the real robot. Namespace/remap the sim before `both` works.
4. Verify the Nav2 `RewrittenYaml` keys exist in turtlebot3_navigation2 `param/burger.yaml`
   (`collision_monitor.cmd_vel_out_topic`, `set_initial_pose`).
5. Record the demo video (`docs/DEMO_SCRIPT.md`).
Then finish any remaining `docs/PLAN.md` items. `real_only` and `sim_only` are closest to working;
`both` needs item 3.

## Implementation order (do NOT skip TDD)
Follow `docs/PLAN.md`. **Start with Phase 1:**
1. **T1.3 `lib/safety.py`** — write failing tests first (front-sector min range; fail-safe staleness;
   dual-scan OR-block; NaN/inf; wrap-around). Then implement. `front_sector_rad` is FULL width.
2. **T1.4 `twin_mediator` v1** — sub `/dt/cmd_vel_raw`(TwistStamped) + `/sim/scan`; 25 cm gate; pub
   `/sim/cmd_vel` (verify type with `ros2 topic type`).
3. **T1.1 sim launch** — add the `turtlebot3_gazebo` bring-up into our `algae_arena.world` (gz_sim +
   spawn + RSP + ros_gz bridge), and a Nav2 `params_file` (RewrittenYaml) with
   `enable_stamped_cmd_vel: true` + `set_initial_pose` to the spawn pose.
Then Phase 2 (mediator full + mission_runner), Phase 3 (sync_supervisor), Phase 4 (GUI), Phase 5
(`both` + hardware), Phase 6 (scenarios + video).

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
