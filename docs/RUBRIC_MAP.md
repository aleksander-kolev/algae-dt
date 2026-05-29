# RUBRIC_MAP.md — how we earn 12/12 (deliverable → pillar → evidence → demo)

> **Assumption & how to use this:** the official 2IRR10 rubric lives on Canvas ("Final Submission and
> Rubric" + the Technical Review document). This maps our work onto the **Digital-Twin triad the
> course grades — ① Bidirectional communication, ② State synchronization, ③ Environmental
> interaction — scored 3 × 4 = 12**, AND onto **every row of the course "Lab Preparation" step
> table** so that whatever the exact weighting, each named deliverable has concrete evidence.
> Cross-check against the Canvas rubric before each review; adjust here if weights differ.
> Take **Option A** (full lab participation) for full rubric potential.

---

## The 12 points

### ① Bidirectional communication — 4 pts  (course row: "DT Integration Node (fan-in/fan-out)")
**Deliverable:** `twin_mediator` reads both sides and writes both sides.
- Real → Digital: `/odom`→`/dt/real_pose`, `/scan`→safety+GUI, `/battery_state`→`/dt/health`.
- Digital → Real: `/dt/cmd_vel_raw` → `/cmd_vel` (TwistStamped).
- Digital → Sim: `/dt/cmd_vel_raw` → `/sim/cmd_vel`.
- Sim → Digital: `/sim/scan`, `/sim/odom`, sim pose → `/dt/sim_pose`.

**Evidence:** `ros2 topic list` + `rqt_graph`; `ros2 topic echo` showing live traffic each direction;
a command typed in teleop/GUI moves BOTH robots; an obstacle in either world is reflected in the DT.
**Demo step:** drive via GUI → both robots move; show the topic graph fanning out.
**Full marks bar:** ≥ 1 topic each direction, live, through a single mediator (not direct wiring).

### ② State synchronization & tolerances — 4 pts  (course row: "Real-Time Synchronization and Tolerances")
**Deliverable:** `sync_supervisor` — the part most teams skip; we make it first-class.
- Real-time pose sync real↔sim (mirror real pose into sim; both follow the DT command bus).
- **Measured** `/dt/sync_error` (Δxy, Δyaw, sensor delta) and `/dt/latency_ms` (command→motion, scan
  age) — *numbers, published & logged to CSV*.
- **Documented tolerance thresholds** in `config/twin.yaml` (`tol_pose_xy_m`, latency budget…).
- **Alerts/logging when out of tolerance:** `/dt/alerts` (String) + CSV row + GUI banner amber/red.

**Evidence:** the generated `sync_metrics_<run>.csv`; a screenshot/recording of the GUI sync banner
going amber when you nudge the sim out of tolerance; `ros2 topic echo /dt/alerts` firing on cue.
**Demo step:** show pose locked in tolerance, then induce a discrepancy → alert + log appear.
**Full marks bar:** sync achieved + latency/error measured & reported + thresholds documented + alerts.

### ③ Environmental interaction — 4 pts  (course rows: "Obstacle Detection & Avoidance" + "Object Manipulation/Transport")
**Deliverable:** three distinct interactions (bar is "pick one" — we do three):
- **25 cm dual-LiDAR safety stop** — obstacle in `/scan` OR `/sim/scan` < 0.25 m zeroes forward
  motion on BOTH robots (course Lidar-DT Mini-Project 3, verbatim).
- **Nav2 dynamic-obstacle avoidance** — the active robot replans around obstacles en route to a bloom.
- **Task interaction (the algae "spray")** — navigate to the bloom centre and spin-in-place 5 s; the
  sim mirrors the interaction. (Optional stretch: push/transport an object to satisfy the course's
  "Object Manipulation/Transport" row literally.)

**Evidence:** put a box in front in either world → both stop; add a dynamic obstacle → Nav2 reroutes;
bloom turns green only after a full spray. **Demo step:** all three on one run.
**Full marks bar:** at least one robust interaction synchronous across both robots; we show three.

### Safety headline (cross-cutting, strengthens all three)
Latched `/dt/estop` halts both robots + cancels the Nav2 goal + latches until RESUME; auto-E-STOP on
critical battery. Demoed explicitly.

---

## Course "Lab Preparation" step-table coverage (every row → our artifact)
| Course step | Our deliverable | Evidence |
|---|---|---|
| Setup & Familiarization | `docs/SETUP.md`, container builds, basic teleop | build log; teleop moves sim |
| Simulation Setup | `turtlebot3_gazebo` + `algae_arena.world` + RViz + `/sim/scan` | sim runs; scan in RViz |
| DT Integration Node (fan-in/fan-out) | `twin_mediator` | topic graph; both-direction echo |
| Real-Time Sync & Tolerances | `sync_supervisor` + CSV + `/dt/alerts` | metrics CSV; alert on cue |
| Obstacle Detection & Avoidance | dual-scan gate + Nav2 | both stop; reroute |
| Object Manipulation/Transport | bloom navigate + spray (mirrored) | green-on-full-spray; sim mirrors |
| Scenario Testing & Debugging | test plan + sensor-noise/dynamic-obstacle scenarios | `docs/PLAN.md` §Test plan; logs |
| Final Demonstration & Video | demo script + one clean run + backup | Week-9 video |

## Review checkpoints (be ready with EVIDENCE, not plans — course requirement)
- **Week 4 review:** ① bidirectional + Simulation Setup working in `sim_only`; mediator fan-out;
  basic sync number. Evidence: topic list, graph, a short clip.
- **Week 8 review:** ②+③ complete; real robot integrated (`both` mode); sync metrics + alerts; all
  three environmental interactions; E-STOP. Evidence: CSV, logs, live run.
- **Week 9 video:** the algae-cleaning story end-to-end, one clean run, backup plan ready.

## 12/12 evidence checklist (tick before each review)
- [ ] `ros2 topic list` + `rqt_graph` screenshot showing fan-in/fan-out.
- [ ] `ros2 topic echo` captures: one real→digital, one digital→real, one digital→sim, one sim→digital.
- [ ] `sync_metrics_<run>.csv` with Δxy/Δyaw/latency columns; a row where alert fired.
- [ ] Clip: GUI sync banner amber/red + `/dt/alerts` when out of tolerance.
- [ ] Clip: obstacle in real OR sim → both robots stop (25 cm).
- [ ] Clip: Nav2 reroute around a dynamic obstacle.
- [ ] Clip: bloom green only after full 5 s spray; nav-failed bloom grey.
- [ ] Clip: E-STOP halts both + latches; RESUME recovers.
- [ ] Tolerance thresholds documented in `config/twin.yaml` (with rationale comments).
- [ ] Risk assessment (`docs/PLAN.md` §Risk) covering hardware availability/reliability/usability.
