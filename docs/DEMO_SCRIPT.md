# DEMO_SCRIPT.md — 2–3 min video (hits all 3 rubric criteria at Redlining)

Run `mode:=both` (real leads, sim mirrors). Keep the GUI + both robots in frame. Each beat names the
rubric line it satisfies. Total ≈ 2:40. Pre-record the `sim_only` version as backup (T6.3).

## 0:00–0:20 · Setup shot
- Show both entities: real Burger in the arena + Gazebo sim + operator GUI (mode/sync/battery banners).
- Say: "One DT — real robot leads, sim mirrors, all state on `/dt/*`."

## 0:20–1:00 · Bidirectional communication (→ Redlining 4)
- Drive via GUI/teleop → **both robots move** (digital→real `/cmd_vel`, digital→sim `/sim/cmd_vel`).
- Cut to a terminal: `rqt_graph` + `ros2 topic echo` showing **each direction**: `/scan`→twin,
  `/cmd_vel`→robot, **plus the internal status topic `/dt/sync_ok`** (the "preferably more" + internal).
- Say the update rate is steady, no dropouts.

## 1:00–1:45 · State synchronization (→ Redlining 4)
- Show pose mirrored real↔sim (GUI overlay) — **multiple states**.
- Trigger an **internal state that affects behavior**: drop battery below critical (or force it) →
  **auto-E-STOP latches**, both robots halt, GUI battery banner red + mode/`/dt/estop` flip. RESUME.
- Cut to `/dt/sync_error` + `/dt/latency_ms` numbers + the amber `/dt/alerts` when nudged out of tol.

## 1:45–2:30 · Environmental & object interaction (→ Redlining 4)
- **Introduce an environment change LIVE:** place a box in front of the real robot →
  **both robots stop** at 25 cm (mirrored, near-real-time); show `stop_skew_ms` is small.
- Remove it → motion resumes; then a bloom mission: navigate around a dynamic obstacle → spray
  (3 full spins) → bloom turns green. Sim mirrors the whole interaction.

## 2:30–2:40 · Close
- "Bidirectional, synchronized, environment-driven — all mirrored across the twin." End.

## Must-show (rubric Redlining triggers)
- [ ] ≥1 topic EACH direction **+ an internal status topic** (bidirectional).
- [ ] ≥1 **internal state that affects behavior/display** mirrored (battery→E-STOP, mode) (sync).
- [ ] a **live environment change** with consistent mirrored response (environmental).
