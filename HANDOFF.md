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
  (battery→auto-E-STOP, mode) mirrored near-real-time, measured error/latency + tolerances + alerts + CSV
  — **and bounded**: `twin_resync` corrects sustained out-of-tolerance pose drift (auto + GUI
  RESYNC button; predict-with-the-model, correct-with-the-data), logged in the CSV `resync` column.
- **③ Environmental** — 25 cm dual-LiDAR stop mirrored on BOTH robots + Nav2 dynamic-obstacle avoidance
  + navigate-and-spray; **introduce a live environment change** in the demo.

## State: DONE — Phases 1→7 implemented, TDD, and integration-verified (see `docs/VERIFICATION.md`)
**DONE:** 11 pure libs (safety/blooms/sync/metrics/geometry/pgm/hud/trajectory/occupancy/resync/gzcli)
+ 5 nodes (twin_mediator, sync_supervisor, mission_runner, operator_gui, twin_resync) + `fake_robot`
+ `dynamic_obstacle`;
full `bringup.launch.py` (sim_only | real_only | both, with `headless`/`use_rviz`/`use_fake_robot`);
`/sim/*` bridge for `both` incl. the **ground-truth sim pose** (`/sim/ground_truth` ← PosePublisher
in `worlds/burger_sim_gt.sdf`) and **bounded-drift resync** (`twin_resync`: GUI RESYNC button +
auto-correct on sustained out-of-tolerance, CSV-stamped; live round-trip gated by
`docker/both_smoke.sh`); reproducible **`algae-dt:dev`** Docker image (`docker/`, `docs/DOCKER.md`).
**252 tests pass** + clean `colcon build`. The five old open items are RESOLVED & verified in-container:
(1) `mode:=sim_only` smoke-tested (Nav2 active + AMCL localized, `/scan` flows) — `docker/sim_smoke.sh`;
(2) gz↔ROS bridge confirmed (the stock spawn bundles `parameter_bridge`; `/scan /odom /cmd_vel /clock`
reach ROS); (3) `both` `/sim/*` namespacing done (custom bridge + namespaced RSP, collision-free);
(4) Nav2 rewrite keys (`collision_monitor.cmd_vel_out_topic`, `set_initial_pose`) verified present in
`burger.yaml`; (5) the cmd_vel chokepoint is TwistStamped end-to-end.

**2026-06-11 lab-bug fix wave** (real-robot session forensics): the Burger's ~0.22 m/s wheel ceiling
saturated DWB's 0.3 m/s plans, bending every fast arc — the launch now caps Nav2
`max_vel_x`/`max_speed_xy` at 0.22 and `lib/safety.limit_command` clamps (vx,wz) with ONE shared
scale factor (curvature-preserving), so real arcs match RViz; the 25 cm stop is STICKY
(`lib/safety.BlockLatch`: once blocked, forward stays cut until the front range exceeds
`stop_release_m` 0.35 m for `stop_release_hold_s` 0.3 s — no more rotate-and-lurch past the box);
the mediator scales ONLY the sim fan-out by clamp(1/RTF) (`lib/sync.rtf_estimate`/`rtf_compensation`)
so the mirror no longer under-travels at real-time-factor < 1; virtual obstacles now reach REAL nav —
the mediator publishes `/dt/scan_nav` (real scan + TRUSTED mirror returns overlaid by angle,
`lib/scanmerge.py`) and the launch points the 4 costmap scan sources + collision_monitor at it while
AMCL stays on the bare `/scan`; `lib/nav2check.py` CHECKS AND REPAIRS the stock Nav2 params
(replace-or-ADD every override, patched file to a temp path — RewrittenYaml's silent no-op on
missing keys is out of the safety path; the lab's no-`use_sim_time` burger.yaml, 2026-06-11, is
auto-fixed) — the launch REFUSES real modes only when no `collision_monitor`/`amcl` section can
host the chokepoint, and `lab_run.sh` aborts (exit 6); and `lab_run.sh` records evidence by default into
`~/turtlebot3_ws/lab_logs/<stamp>_<gitref>/` (bag + console.log + per-node ROS logs + sync CSV;
`--no-log` opts out). **Meta-finding:** the 2026-06 session demoed the stale DEFAULT branch (docs
said plain `git clone`) — **push the current branch before any lab session**; `lab_run.sh` prints
the demoed commit.

**REMAINING (environment/process — not code):**
- **navigate-and-spray now COMPLETES in throttled sim_only** (verified live on WSLg: `idle →
  navigating:0 → spraying:0 → complete`, measured ~3.00 full revolutions (closed-loop from
  `/dt/odom_active`)). The launch loosens the
  sim_only `collision_monitor.source_timeout` AND the controller progress checker so the ~3.5 Hz
  software-render LiDAR no longer aborts nav ("Failed to make progress"); the spray is closed-loop on
  odom (3 full turns guaranteed); near-wall goals are projected clear of the inflation zone. Still
  prefer a **GPU host / the lab laptop** (5 Hz scan) for a crisp demo. Details: BEST_APPROACHES §Lessons.
- **Record the Week-9 demo video** (`docs/DEMO_SCRIPT.md`) on a GPU host, and **validate on lab
  hardware** (`both` on the Burger via `scripts/lab_run.sh` (or `real_only` via a manual `ros2 launch`); `docs/RUN_ON_LAB_PC.md`).

> ⚠️ **Shared branch:** this branch (`feat/poc-implementation`) also received `feat(lab)` commits
> (lab_run.sh / RUN_ON_LAB_PC.md) from a parallel effort. If two sessions edit at once, coordinate to
> avoid clobbering the working tree.

## What's next (implementation is done — these are lab/demo steps)
1. **GPU host:** `bash docker/run.sh` → inside, `ros2 launch algae_dt bringup.launch.py mode:=sim_only`
   (display). Drive the GUI, place blooms, Start; record the baseline video per `docs/DEMO_SCRIPT.md`.
   Hardware-free `both`: `mode:=both use_fake_robot:=true` (AMCL auto-seeds at the origin there; the
   fake robot's LiDAR is raycast from the course map, so AMCL/sync/safety all behave — no RViz
   pose-estimate race). One-command GUI variants from Windows/WSL: `docker/open_sim.sh` /
   `docker/open_both.sh`.
2. **Lab:** connect to the Burger (SETUP §3), then `mode:=both` via `scripts/lab_run.sh` (or
   `mode:=real_only` via a manual `ros2 launch`) (`docs/RUN_ON_LAB_PC.md`). Validate the 25 cm stop on real `/scan`, AMCL
   2D-Pose-Estimate, one bloom navigate+spray, the `both` mirror. Evidence (bag/console/CSV) is
   auto-recorded under `~/turtlebot3_ws/lab_logs/` — copy it off before leaving.
3. **Verify anytime:** `docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash
   /ci/ci.sh` (252 tests + build); `bash /ci/sim_smoke.sh` (sim_only stack) and `bash
   /ci/both_smoke.sh` (live `both` mirror: ground-truth pose path + a real resync round-trip).
   Evidence map: `docs/VERIFICATION.md`.

> Re-implementing? The TDD recipe still holds: failing test (pure libs) → minimal impl → `colcon
> build --packages-select algae_dt` → `pytest src/algae_dt/test` → run in `sim_only` → commit.

## HARD INVARIANTS (read `docs/RULES.md` — breaking these fails silently)
1. **Bus = `TwistStamped`**; mediator forwards to real `/cmd_vel` (TwistStamped); set Nav2
   `enable_stamped_cmd_vel: true` via params_file. (Do NOT switch to plain Twist — see D3.)
2. **Topic-collision rule:** real = bare topics, sim = `/sim/*`, sim TF off global `/tf` in `both`.
3. **`use_sim_time: true` only in `sim_only`** (bool to node params, lowercase string to includes).
4. **Safety gate fails SAFE** (stale considered-scan → blocked; startup no-data → unblocked); EITHER
   scan <0.25 m zeroes forward on BOTH; the block is STICKY (releases only at `stop_release_m`
   sustained `stop_release_hold_s`).
5. **Nav failure / aborted spray must NOT mark a bloom treated** (honest accounting).
6. **No sudo / no settings changes on lab kit**; build only `--packages-select algae_dt`.

## Do NOT regress (settled — see `docs/DECISIONS.md`)
- Don't hand-build SDF/URDF/`robot_state_publisher`/teleop/Nav2 params — use stock packages (D2).
- Don't make the bus plain Twist (D3). Don't split into 2 packages (D4). Don't put `name=` on
  `mission_runner`'s launch Node (process-wide remap trap). Don't remap cmd_vel on `mission_runner`
  (it publishes none — route Nav2 via the `collision_monitor.cmd_vel_out_topic` RewrittenYaml rewrite).

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
**Home** = the Docker container (`docker/run.sh`, image `algae-dt:dev`). **Lab PC = NATIVE, NO
Docker (verified 2026-06):** ROS Jazzy at `/opt/ros/jazzy` + the turtlebot3 stack built FROM SOURCE
in `~/turtlebot3_ws/src` — D1. **One command, from a fresh clone:** `./scripts/lab_run.sh --sim`
(hardware-free sim demo) or `ROS_DOMAIN_ID=<robot#> ./scripts/lab_run.sh` (lab; full `both` real+sim
demo) — it auto-detects the runtime (native on the lab PC; `--native` forces), copies the package,
builds, and launches. Workspace broken/copied-from-another-PC → `./scripts/lab_fix_workspace.sh`
(no sudo; rebuilds from the fail-safe zip in `~/Downloads`). Full flow + provenance:
`docs/RUN_ON_LAB_PC.md`. Robot: Wi-Fi `AP2IRR10`, `ssh turtlebot@<ip>` →
`turtlebot3_bringup robot.launch.py`.
