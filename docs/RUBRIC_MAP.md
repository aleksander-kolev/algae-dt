# RUBRIC_MAP.md — how we earn 12/12 (deliverable → pillar → evidence → demo)

> **OFFICIAL rubric (confirmed — "Implementation of PoC", 3 criteria × 4 = 12):** ① Bidirectional
> Communication, ② Synchronization of States, ③ Environmental & Object Interaction. The assignment is
> 30 pts total — the extra points come from the video/presentation (per Canvas). **Take Option A** —
> **Option B caps each criterion** (Bidirectional ≤3, Sync ≤2, Environmental ≤2 → max 7/12).
> Full-mark band per criterion is **"Redlining" (4)**; our deliverables below target exactly its
> wording. We also cover every "Lab Preparation" step-table row.
>
> **Redlining triggers (must all be true in the demo):** (①) robust 2-way pub/sub, consistent rate,
> no dropouts, ≥1 topic each direction **+ an internal status topic**; (②) near-real-time mirroring of
> **multiple** states incl. **≥1 internal state (battery/health/fault/mode) that affects behavior or
> display**; (③) obstacle/object interaction **mirrored across entities** with **≥1 environment change
> introduced live** during the demo.

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
**Hardware-independent demo (de-risks the High robot-unavailability risk):** the ported `fake_robot.py`
publishes the real robot's bare `/scan /odom /cmd_vel /battery_state`, so the real→digital and
digital→real arrows are demonstrable at home without a robot.
**Demo step:** drive via GUI → both robots move; show the topic graph fanning out.
**Redlining (4):** robust 2-way pub/sub at a consistent rate, no dropouts; ≥1 topic each direction
(e.g. `/scan`→twin, `/cmd_vel`→robot) **plus an internal status topic** (`/dt/sync_ok`/`/dt/mode`).

### ② State synchronization & tolerances — 4 pts  (course row: "Real-Time Synchronization and Tolerances")
**Deliverable:** `sync_supervisor` — the part most teams skip; we make it first-class.
- Real-time pose sync real↔sim (mirror real pose into sim; both follow the DT command bus).
- **Measured** `/dt/sync_error` (Δxy, Δyaw, sensor delta) and `/dt/latency_ms` (command→motion, scan
  age) — *numbers, published & logged to CSV*.
- **Documented tolerance thresholds + rationale** in `config/twin.yaml` (`tol_pose_xy_m`,
  `latency_budget_ms`, `stop_skew_ms`…).
- **Alerts/logging when out of tolerance:** `/dt/alerts` (String) + CSV row + GUI banner amber/red.
- **Demonstrable in `sim_only` (the fallback env):** with no real robot, sync error = COMMANDED
  (integrated from `/dt/cmd_vel_raw`) vs ACHIEVED sim pose (`sim_only_sync_source: commanded`) — so
  pillar ② is shown even without hardware. Latency = mediator command stamp → sim motion onset.

**Evidence:** the generated `sync_metrics_<run>.csv`; a screenshot/recording of the GUI sync banner
going amber when you nudge the sim out of tolerance; `ros2 topic echo /dt/alerts` firing on cue.
**Demo step:** show pose locked in tolerance, then induce a discrepancy → alert + log appear.
**Redlining (4):** near-real-time mirroring of **multiple** states incl. **≥1 internal state that
affects behavior/display** — battery≤critical → **auto-E-STOP** (behavior) + red banner, `/dt/mode`,
plus measured error/latency + documented tolerances + alerts; mirror stays consistent throughout.

### ③ Environmental interaction — 4 pts  (course rows: "Obstacle Detection & Avoidance" + "Object Manipulation/Transport")
**Deliverable:** three distinct interactions (bar is "pick one" — we do three):
- **25 cm dual-LiDAR safety stop** — obstacle in `/scan` OR `/sim/scan` < 0.25 m zeroes forward
  motion on BOTH robots (course Lidar-DT Mini-Project 3, verbatim).
- **Nav2 dynamic-obstacle avoidance** — the active robot replans around obstacles en route to a bloom.
- **Task interaction (the algae "spray")** — navigate to the bloom centre and spin-in-place 5 s; the
  sim mirrors the interaction. (Optional stretch: push/transport an object to satisfy the course's
  "Object Manipulation/Transport" row literally.)

**Evidence:** put a box in front in either world → both stop (and the **measured `stop_skew_ms`** real
vs sim is logged — synchrony is quantified, not just claimed); add a dynamic obstacle (scripted/teleoped
in sim) → Nav2 reroutes; bloom turns green only after a full spray. **Demo step:** all three on one run.
**Redlining (4):** obstacle/object interaction functional and **mirrored across both entities**
near-real-time, with **≥1 environment change introduced LIVE during the demo** (drop a box) →
consistent mirrored stop (skew within `stop_skew_ms`); we show three interactions.

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
- [ ] `sync_metrics_<run>.csv` with Δxy/Δyaw/latency + `stop_skew_ms` columns; a row where alert fired.
- [ ] Clip: GUI sync banner amber/red + `/dt/alerts` when out of tolerance.
- [ ] Clip: obstacle in real OR sim → both robots stop (25 cm); measured stop-skew within `stop_skew_ms`.
- [ ] Clip: Nav2 reroute around a dynamic obstacle.
- [ ] Clip: bloom green only after full 5 s spray; nav-failed bloom grey.
- [ ] Clip: E-STOP halts both + latches; RESUME recovers.
- [ ] Tolerance thresholds documented in `config/twin.yaml` (with rationale comments).
- [ ] Context diagram (`docs/CONTEXT_DIAGRAM.md`) rendered for the Week-4 review.
- [ ] `sim_only` baseline video pre-recorded as the guaranteed backup (T6.3).
- [ ] Risk assessment (`docs/PLAN.md` §Risk) covering hardware availability/reliability/usability.
