# algae-dt — TurtleBot3 Burger Digital Twin (TU/e 2IRR10)

A bidirectional **digital twin**: a real TurtleBot3 Burger and a Gazebo Harmonic twin run in
parallel with bidirectional comms + state synchronization. An operator places algae blooms; the
active robot Nav2-navigates to each, sprays (3 full spins), and a 25 cm dual-LiDAR gate stops forward
motion in either world. Built the **course-recommended way**: WSL+Docker / lab laptop + the
**default turtlebot3 packages** + one **custom `algae_dt` package** (the thin DT layer).

## Quickstart (home, Docker)
Build the reproducible course-faithful dev image (ROS 2 Jazzy + the full turtlebot3 stack + Nav2 +
Gazebo Harmonic + PyQt5 — our equivalent of the course `turtlebot3_ws` image; see `docs/DOCKER.md`):
```bash
docker build -t algae-dt:dev docker          # Windows: docker/build.ps1
```
**Verify** (clean build of our package + the full test suite, headless):
```bash
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/ci.sh
```
**Run `sim_only` with a display** (WSL/WSLg renders Gazebo/RViz/the GUI):
```bash
bash docker/run.sh                           # then, inside the container:
cd /ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```
**Headless end-to-end smoke** (no display): `… bash /ci/sim_smoke.sh`. The course `turtlebot3_ws`
image and the lab laptop also work — put `ros2_ws/src/algae_dt` at `~/turtlebot3_ws/src/algae_dt`
(see `docs/SETUP.md`).

## Modes
`ros2 launch algae_dt bringup.launch.py mode:=<sim_only|real_only|both>` with optional
`headless:=true` (CI/no display), `use_rviz:=true`, `use_fake_robot:=true` (drive real_only/both at
home with a kinematic stand-in, no hardware).

## Docs
| File | What |
|---|---|
| `HANDOFF.md` | **Resuming AI starts here** — state, what's done/verified, invariants. |
| `CLAUDE.md` | What this is + the course method + architecture + topic contract. |
| `docs/DECISIONS.md` | Why — settled decisions + forensics (preserve this context). |
| `docs/DOCKER.md` | The reproducible `algae-dt:dev` image + build/run/verify helpers. |
| `docs/SETUP.md` | WSL+Docker, lab laptop, robot connection, file transfer — exact commands. |
| `docs/RULES.md` | Hard constraints (course golden rules + technical invariants + style). |
| `docs/BEST_APPROACHES.md` | Patterns, lessons & gotchas (incl. Nav2 cmd_vel topology, headless render). |
| `docs/PLAN.md` | Phased 12/12 implementation plan (7 lab sessions, Week-4/8 reviews, Week-9 video). |
| `docs/VERIFICATION.md` | **Deliverable → rubric pillar → test/script evidence** (what's proven + how). |
| `docs/RUBRIC_MAP.md` | Deliverable → rubric pillar → evidence → demo step. |
| `docs/SCENARIOS_SIM.md` | `sim_only` run scenarios (dev + fallback demo) — flow + edge cases. |
| `docs/SCENARIOS_LAB.md` | Lab-laptop + real-robot run scenarios — flow + edge cases. |
| `docs/RUN_ON_LAB_PC.md` | Pull repo on the lab laptop → build → run (`scripts/lab_run.sh`). |
| `docs/CONTEXT_DIAGRAM.md` | System context diagram (boundary + topic flows). |
| `docs/SUBMISSION.md` / `docs/DEMO_SCRIPT.md` | PoC submission package + timed 2–3 min demo script. |

## Status
**Implemented and tested.** All 11 pure libs + 5 nodes (+ `fake_robot`, `dynamic_obstacle`) are
done with **239 passing tests** (unit + in-process rclpy integration) and a clean
`colcon build --packages-select algae_dt`. `sim_only` launches end-to-end headless (Nav2 active +
AMCL auto-localized, the TwistStamped chokepoint, all `/dt/*` flowing); `both` keeps the real (bare)
and sim (`/sim/*`) topics collision-free with the mediator fanning out to both, publishes the sim's
**ground-truth pose**, and **bounds the real-vs-sim drift** (`twin_resync`: GUI RESYNC button +
auto-correct on sustained out-of-tolerance, every correction CSV-logged; live round-trip gated by
`docker/both_smoke.sh`). The full navigate-and-spray demo runs at GPU-rate LiDAR (lab laptop / GPU
host) — see `docs/VERIFICATION.md`.
