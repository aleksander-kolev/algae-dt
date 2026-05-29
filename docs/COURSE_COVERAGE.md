# COURSE_COVERAGE.md — every 2IRR10 course artifact → where it lives in this project

Verification that **all course pages, PDFs, tutorials, downloads, and process requirements** are
accounted for in the plan + structure. Status key: **USED** (drives a file/deliverable),
**READ** (required reading / knowledge, no artifact), **ASSUMED** (depends on Canvas confirmation),
**OPTIONAL** (stretch/enhancer), **N/A**.

> **Disclaimer:** this list is best-effort from the materials we have, **not** a confirmed-exhaustive
> Canvas index. The **rubric is now confirmed** (official "Implementation of PoC": 3 criteria × 4 = 12,
> Option B caps at 7/12) — see `docs/RUBRIC_MAP.md` + `docs/SUBMISSION.md`.

## Setup & environment pages/PDFs
| Course artifact | Status | Where in this project |
|---|---|---|
| Software Setup and Usage (page) | READ/USED | `SETUP.md` §0 reading order |
| WSL+Docker Setup and Installation.pdf | USED | `SETUP.md` §0/§1 (primary env) |
| Setting up your workspace in WSL.pdf | USED | `SETUP.md` §1 (`~/turtlebot3_ws/src/algae_dt`) |
| VirtualBox+VM Installation.pdf | USED | `SETUP.md` §1b (VM alternative) |
| Running the VM.pdf | USED | `SETUP.md` §1b (VM alternative) |
| MAC OS users.pdf | READ (team on Windows) | `SETUP.md` §1b note (kept for completeness) |
| Connecting lab laptop to lab robot.pdf | USED | `SETUP.md` §3 (robot connection, verbatim flow) |
| Transferring files.pdf | USED | `SETUP.md` §2 (full-package copy to lab laptop) |
| GazeboTutorial.pdf | USED | `BEST_APPROACHES.md` (`pkill -9 -f "gz sim"`) + `SETUP.md` §1c (world/`<size>` editing) |
| Changing Robot Inflation.pdf | USED | `BEST_APPROACHES.md` (runtime inflation, no sudo) |
| How to create Packages.pdf (Sections 1/2/3) | USED | `SETUP.md` §2 (Section 3 = copy full package) |

## Concept / Solution-Space pages (required reading → shape design & deliverables)
| Course artifact | Status | Where |
|---|---|---|
| Lab session overview / Intro presentation | READ | `RULES.md` §A, `PLAN.md` (schedule, Option A/B, roles) |
| Lab Advice | USED | `RULES.md` §A, `PLAN.md` §Risk + PoC/backlog framing |
| Lab Rules and Regulations (18 Golden Rules) | USED | `RULES.md` §A (all 18 distilled) |
| Lab Preparation (step table) | USED | `RUBRIC_MAP.md` step-table coverage; `PLAN.md` phases |
| Intro to Technical Solutions and the Proof of Concept | READ/USED | `PLAN.md` §PoC scope & backlog |
| Development and Implementation of Proof of Concept | READ/USED | `PLAN.md` phases (PoC = the build) |
| Introduction to Context Diagrams | USED | `PLAN.md` T0.4 → `docs/CONTEXT_DIAGRAM.md` deliverable |
| Digital Twin (concept) | READ | `CLAUDE.md` §1, `RUBRIC_MAP.md` (DT triad) |
| Mobile Robot and Digital Twin | READ | `CLAUDE.md` §1/§4 |
| Simulation and Robot Operating System (ROS) | READ | `RULES.md` §C, pub/sub design |
| Final Submission and Rubric / Submit PoC | USED (confirmed) | `RUBRIC_MAP.md` (official 3×4=12, Option A/B caps) + `SUBMISSION.md` (zip/credentials/video, due 22 Jun 2026) |
| Technical Review document | USED | `RUBRIC_MAP.md` review checkpoints (Week 4 & 8) |

## Tutorials & provided code (the canonical templates we build on)
| Course artifact | Status | Where |
|---|---|---|
| Simple Publisher & Subscriber.pdf + subscriber_node / publisher_node downloads | USED | `PLAN.md` T0.3 knowledge note; our nodes ARE pub/sub built on this primitive |
| DT Example With Only Gazebo.pdf (Weather / online-source context-aware DT) + tb3_weather_dt.zip | OPTIONAL | `PLAN.md` Phase 6 stretch: online-source-driven cautious mode (maps to "DT connects to online source") |
| DT Example with Gazebo and Physical Robot.pdf (Lidar DT, 3 mini-projects) + tb3_safety_stop | USED (blueprint) | This IS our blueprint — mediator + 25 cm dual-LiDAR stop + `/cmd_vel`+`/sim/cmd_vel` (Mini-Project 3) |
| Simulation Files.zip (new_world.world, new_world.launch.py, setup.py) | USED | `worlds/algae_arena.world` is the course `new_world.world`; our `setup.py`/launch follow the provided pattern |
| mapFiles.zip (map.pgm, map.yaml) | USED | `maps/map.{pgm,yaml}` (res 0.05, origin [-2.051,-4.194]) |

## Process / logistics requirements
| Requirement | Status | Where |
|---|---|---|
| 7 lab sessions, Weeks 2–9; reviews Week 4 & 8; video Week 9 | USED | `PLAN.md` phase→session mapping; `RUBRIC_MAP.md` checkpoints |
| Option A (full participation, full rubric) vs Option B | USED | `RUBRIC_MAP.md` (take Option A); `PLAN.md` note |
| 2–3 rotating students, defined roles, 30–60 min test plan | USED | `RULES.md` §A-8, `PLAN.md` per-session test goals |
| Equipment handover / sign sheet; print Lab Advice; back up (factory reset) | USED | `RULES.md` §A-6, `SETUP.md` §2 backup, lab-day checklist below |
| No sudo / no settings change / robot handling / shutdown | USED | `RULES.md` §A-1..5 |
| Risk assessment (hardware availability/reliability/usability) | USED | `PLAN.md` §Risk |

## Gaps closed by this audit (edits made)
- Added **Context Diagram** as an explicit deliverable (`PLAN.md` T0.4 → `docs/CONTEXT_DIAGRAM.md`).
- Added **PoC scope + backlog** framing (`PLAN.md`) per "Phase 3 — Scope of PoC".
- Added **pub/sub primitive** knowledge note + the **VM** and **Mac** setup alternatives (`SETUP.md`).
- Recorded **asset provenance** (Simulation Files.zip / mapFiles.zip) in `SETUP.md` + here.
- Added **optional online-source context-aware DT** stretch (Weather-DT pattern) to `PLAN.md`.

## Lab-day checklist (from Lab Preparation/Advice — print & bring)
- [ ] 2–3 members assigned; roles (driver/spotter/logger/presenter).
- [ ] Latest code pulled + builds in `sim_only`; 30–60 min test plan written.
- [ ] Repo committed + backed up to OneDrive/USB (laptop may be wiped).
- [ ] Printed Lab Advice + sign-out/sign-in sheet brought.
- [ ] Robot number / IP / ROS_DOMAIN_ID noted from the stickers.
