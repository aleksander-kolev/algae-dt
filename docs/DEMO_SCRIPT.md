# DEMO_SCRIPT.md — 2–3 min video (hits all 3 rubric criteria at Redlining)

Run `mode:=both` (real leads, sim mirrors). Keep the GUI + both robots in frame. Each beat names the
rubric line it satisfies. Total ≈ 2:40. Pre-record the `sim_only` version as backup (T6.3).
If the Gazebo 3D window crashes/flickers on the lab PC's GL stack, run `lab_run.sh --no-gz-gui`
and frame **RViz** for the sim side instead — the gz server still renders the LiDAR and the launch
keeps running (a 3D-client crash is non-fatal by design).

> **Where to shoot:** the comms / safety-stop / E-STOP beats record fine anywhere. The full
> navigate-and-spray completion beat (bloom turns green) needs the **lab laptop or a GPU host** —
> a CPU-only Docker host renders the sim LiDAR too slowly for Nav2 to reliably *arrive*
> (`docs/VERIFICATION.md` §Known environmental limit).

## 0:00–0:20 · Setup shot
- Show both entities: real Burger in the arena + Gazebo sim + operator GUI (mode/sync/battery banners).
- Say: "One DT — real robot leads, sim mirrors, all state on `/dt/*`."

## 0:20–1:00 · Bidirectional communication (→ Redlining 4)
- Drive via **teleop** (separate terminal — `lab_run.sh` prints the exact teleop line for the
  runtime it picked: a native `bash -lc '… teleop_keyboard …'` line on the lab PC, a `docker exec …`
  line only on a Docker host; the GUI itself does not drive) → **both robots move**
  (digital→real `/cmd_vel`, digital→sim `/sim/cmd_vel`).
- Cut to a terminal: `rqt_graph` + `ros2 topic echo` showing **each direction**: `/scan`→twin,
  `/cmd_vel`→robot, **plus the internal status topic `/dt/sync_ok`** (the "preferably more" + internal).
- Say the update rate is steady, no dropouts.

## 1:00–1:45 · State synchronization (→ Redlining 4)
- Show pose mirrored real↔sim (GUI overlay) — **multiple states**.
- Trigger an **internal state that affects behavior** — battery → auto-E-STOP:
  - `both` (real robot): a genuinely low battery trips it; otherwise use the manual E-STOP button.
  - `sim_only` (backup video): force the synthetic battery below critical —
    `ros2 topic pub --once /dt/battery_override_v std_msgs/msg/Float64 "{data: 10.0}"`
  → **auto-E-STOP latches**, both robots halt, GUI battery banner red + `/dt/estop` flips. The
  latch holds even if the voltage recovers; press **Resume** (clears it; publish `{data: 0.0}` to
  drop the override).
- Cut to `/dt/sync_error` + `/dt/latency_ms` numbers + the GUI **ALERT banner** / `/dt/alerts`
  when nudged out of tolerance.
- **The resync beat (`both`):** let the drift exceed tolerance (carry the robot half a metre, or
  just keep driving) → alert fires → say *"drift exceeded our documented 15 cm tolerance — the
  twin now corrects it"* → it **auto-resyncs** (or press **RESYNC TWIN**): the sim snaps onto the
  real pose, `dxy` collapses to ~0 live on the SYNC banner ("resynced (auto) 2s ago"), and the
  CSV row carries `auto`. Measured → alerted → **corrected**, in one shot.

## 1:45–2:30 · Environmental & object interaction (→ Redlining 4)
- **Introduce an environment change LIVE:**
  - `both` (real robot): place a physical box in front of the real Burger.
  - `sim_only` (backup video): launch with `use_dynamic_obstacle:=true` (or start it mid-demo:
    `ros2 run algae_dt dynamic_obstacle --ros-args -p use_sim_time:=true`) — the box sweeps
    across the robot's path.
  → **both robots stop** at 25 cm (mirrored, near-real-time); show `stop_skew_ms` is small.
- Remove it → motion resumes; then a bloom mission: navigate around the obstacle → spray
  (3 full spins) → bloom turns green. Sim mirrors the whole interaction.

## 2:30–2:40 · Close
- "Bidirectional, synchronized, environment-driven — all mirrored across the twin." End.

## Must-show (rubric Redlining triggers)
- [ ] ≥1 topic EACH direction **+ an internal status topic** (bidirectional).
- [ ] ≥1 **internal state that affects behavior/display** mirrored (battery→E-STOP, mode) (sync).
- [ ] a **live environment change** with consistent mirrored response (environmental).
