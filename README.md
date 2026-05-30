# algae-dt — TurtleBot3 Burger Digital Twin (TU/e 2IRR10)

A bidirectional **digital twin**: a real TurtleBot3 Burger and a Gazebo Harmonic twin run in
parallel with bidirectional comms + state synchronization. An operator places algae blooms; the
active robot Nav2-navigates to each, sprays (5 s spin), and a 25 cm dual-LiDAR gate stops forward
motion in either world. Built the **course-recommended way**: WSL+Docker / lab laptop + the
**default turtlebot3 packages** + one **custom `algae_dt` package**.

## Quickstart (home, WSL + Docker)
```bash
# 1) start the course container (shares host network so it talks to the robot directly)
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/turtlebot3_ws:/ws \
  --user $(id -u):$(id -g) turtlebot3_ws bash
# 2) inside the container
cd /ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```
Put this repo's `ros2_ws/src/algae_dt` at `~/turtlebot3_ws/src/algae_dt` first. Full instructions
(lab laptop, real robot, GUI, transfer) → **`docs/SETUP.md`**.

## Docs
| File | What |
|---|---|
| `HANDOFF.md` | **Resuming AI starts here** — state, implementation order, invariants, first task. |
| `CLAUDE.md` | What this is + the course method + architecture. |
| `docs/DECISIONS.md` | Why — settled decisions + forensics (preserve this context). |
| `docs/SETUP.md` | WSL+Docker, lab laptop, robot connection, file transfer — exact commands. |
| `docs/RULES.md` | Hard constraints (course golden rules + technical invariants + style). |
| `docs/BEST_APPROACHES.md` | Patterns, lessons & gotchas. |
| `docs/PLAN.md` | Phased 12/12 implementation plan (7 lab sessions, Week-4/8 reviews, Week-9 video). |
| `docs/RUBRIC_MAP.md` | Deliverable → rubric pillar → evidence → demo step. |
| `docs/SCENARIOS_SIM.md` | `sim_only` run scenarios (dev + fallback demo) — flow + edge cases. |
| `docs/SCENARIOS_LAB.md` | Lab-laptop + real-robot run scenarios — flow + edge cases. |
| `docs/RUN_ON_LAB_PC.md` | Pull repo on the lab laptop → build → run (with honest readiness verdict). |
| `docs/CONTEXT_DIAGRAM.md` | System context diagram (boundary + topic flows). |
| `docs/SUBMISSION.md` | PoC submission package, checklist, due date, Option A (official rubric). |
| `docs/DEMO_SCRIPT.md` | Timed 2–3 min demo video script mapped to the rubric. |

## Status
Scaffold + docs + plan complete. Node logic is built per `docs/PLAN.md` (TDD, tested in `sim_only`
at home before any lab session).
