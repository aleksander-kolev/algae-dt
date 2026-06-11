# CODE_EXPLAINED.md — the whole project, explained simply

This document explains **everything** in this repository — the ideas, the architecture, every
node, every library, every config file — in plain language. It assumes you know basic Python but
nothing about ROS or robots. It is self-contained: you can read it top to bottom without opening
any other document.

---

## 1. What is this project?

Imagine a small robot vacuum-cleaner-sized robot (a **TurtleBot3 Burger**) whose job is to clean
algae blooms from a pond. An operator looks at a map on a screen, clicks where the algae is, and
presses **Start**. The robot drives to each spot, spins around three full times (pretend it is
spraying chemicals), and moves on to the next one. If anything gets within 25 cm in front of it,
it stops. If the battery gets critically low, it emergency-stops.

The twist: there are **two** robots. A **real** one driving around a physical arena, and a
**simulated** one driving around a virtual copy of that arena (in the Gazebo simulator). They are
connected so that they move together and constantly compare themselves to each other. That pair —
a real thing plus a live virtual copy that mirrors it — is called a **digital twin**.

Why bother with a twin? Because you can:
- **watch** the real robot through its virtual copy (even from far away),
- **test** dangerous situations virtually (put a virtual wall in front of the virtual robot — the
  real robot stops too, because either world can trigger the shared safety brake),
- **measure** how well your simulation matches reality (if the virtual robot drifts away from the
  real one, your model of reality is wrong, and the system tells you by how much).

This is a university project (TU/e course 2IRR10) graded on three things, called **pillars**:

1. **Bidirectional communication** — information must flow both ways between the real robot and
   the digital side (sensors go in, commands come out, plus internal status topics).
2. **Synchronization of states** — the two worlds must mirror each other's state (position,
   battery, mode) nearly in real-time, with **measured** error, **documented** tolerances, and
   **alerts** when out of tolerance.
3. **Environmental interaction** — the robot must react to its environment (obstacles, the
   spraying task), and a live change in the environment must produce a consistent, mirrored
   response in both worlds.

---

## 2. The concepts you need, in plain words

### ROS 2 — the robot's nervous system
**ROS 2** (Robot Operating System 2) is not an operating system — it is a messaging framework.
A robot program is split into small independent programs called **nodes**. Nodes talk by
**publishing** messages onto named channels called **topics**, and **subscribing** to the topics
they care about. For example, the LiDAR driver publishes distance measurements on the topic
`/scan`; anyone who wants those measurements subscribes to `/scan`. Nobody calls anybody
directly — everything is "shout into a channel / listen to a channel". This makes it trivial to
swap parts in and out (a simulated LiDAR publishes the same `/scan` topic as a real one).

Key vocabulary used everywhere below:
- **node** — one running program (we have six of our own).
- **topic** — a named message channel, e.g. `/cmd_vel` ("command velocity" = drive commands).
- **message type** — the data structure on a topic, e.g. `LaserScan` (360 distances) or
  `TwistStamped` (a velocity command with a timestamp).
- **parameter** — a named setting a node reads at startup (we keep ours in `config/twin.yaml`).
- **launch file** — a Python script that starts many nodes at once with the right wiring.
- **QoS** (Quality of Service) — per-topic delivery rules. Two matter here:
  *sensor data* (lossy but fast — fine for high-rate scans) and *latched* ("transient local":
  a late subscriber immediately receives the last value — used for things like "is the
  emergency stop on?", which you must know even if you subscribed after it was pressed).

### Frames and TF — "where is everything?"
Positions are always *relative to something*. ROS names these reference points **frames**:
- `base_link` / `base_footprint` — the robot itself (its body / its point on the floor),
- `odom` — where the robot *thinks* it started, with position integrated from wheel rotations
  (**odometry**). Smooth but drifts over time, like dead reckoning,
- `map` — the fixed map of the arena. Positions in the map frame don't drift.

**TF** is the ROS subsystem that stores the live tree of transformations between frames.
**AMCL** (Adaptive Monte Carlo Localization) is the standard algorithm that compares laser scans
against the known map to figure out where the robot really is; it continuously publishes the
correction between `map` and `odom`. That is why, in the code, turning an odometry position into
a map position is one multiplication: `map_pose = (map←odom correction) ∘ (odom pose)` —
implemented in `lib/geometry.py: compose_pose_2d`.

One practical consequence you'll see in the code: **never compare a map-frame position with an
odom-frame position** — they have different origins, and the difference would be a meaningless
offset. Several of our fixes exist precisely to enforce this rule.

### Nav2 — the autopilot
**Nav2** is ROS 2's standard navigation stack. You give it a goal pose on the map; it plans a
path around obstacles (using costmaps built from the map + live laser data), and streams velocity
commands until the robot arrives. We use it through a small helper class (`BasicNavigator`):
`goToPose(goal)`, then poll `isTaskComplete()`, then check `getResult()`. AMCL is part of the
Nav2 bring-up. Critically, we **reroute Nav2's output** so that its velocity commands do not go
to the robot directly — they go onto our command bus and through our safety gate first
(explained below).

### Gazebo — the virtual world
**Gazebo** (version "Harmonic") is a physics simulator: a 3D world with a simulated Burger whose
simulated LiDAR produces real `LaserScan` messages. A **bridge** (`ros_gz_bridge`) converts
between Gazebo's internal messages and ROS topics. The simulator has its own clock (**sim time**)
which can run slower than reality on a weak computer — the ratio is called the **real-time
factor** (RTF). RTF 0.5 means one simulated second takes two wall-clock seconds. Several bugs we
fixed came from mixing sim time and wall time.

### The TwistStamped contract
Drive commands are `TwistStamped` messages (a linear velocity, an angular velocity, and a header
with a timestamp). On this ROS version (Jazzy) the real TurtleBot3 expects **stamped** commands,
while Nav2 by default publishes plain unstamped `Twist`. The launch file flips a Nav2 setting
(`enable_stamped_cmd_vel`) so the whole chain is `TwistStamped` end-to-end. Get this wrong and
nothing moves while nothing errors — the #1 silent trap of this stack.

---

## 3. The big picture

```
                       OPERATOR
                          │ clicks blooms / Start / Stop / E-STOP
                          v
                 ┌─────────────────┐  /dt/* state (poses, sync, battery, alerts…)
                 │  operator_gui   │<──────────────────────────────────────────┐
                 └─────────────────┘                                           │
                          │ /dt/blooms, /dt/mission_cmd, /dt/estop_cmd         │
                          v                                                    │
                 ┌─────────────────┐   Nav2 goal    ┌──────────┐               │
                 │ mission_runner  │───────────────>│   Nav2   │               │
                 └─────────────────┘                └──────────┘               │
                          │ spray spin                    │ planned velocity   │
                          v                               v                    │
                ════════ /dt/cmd_vel_raw  (the COMMAND BUS, TwistStamped) ═════════
                          │                  ▲                                 │
                          │                  │ teleop (keyboard) also          │
                          v                  │ publishes here                  │
                 ┌─────────────────┐         │                                 │
   /scan ───────>│  twin_mediator  │ 25 cm dual-LiDAR safety gate + E-STOP     │
   /sim/scan ───>│  (the choke-    │ + speed limits — applied ONCE, here       │
   /odom ───────>│   point)        │──────> /cmd_vel      (real robot)         │
   /sim/odom ───>│                 │──────> /sim/cmd_vel  (sim mirror, `both`) │
   /battery ────>│                 │──────> /dt/*  mirrors ─────────────────────┘
                 └─────────────────┘
                          │ /dt/real_pose, /dt/sim_pose, /cmd_vel, /dt/odom_active…
                          v
                 ┌─────────────────┐
                 │ sync_supervisor │ measures: pose error, latency, stop-skew
                 └─────────────────┘ publishes /dt/sync_error /dt/sync_ok /dt/alerts + CSV
```

The design rule behind the whole thing: **every command goes through one chokepoint**
(`twin_mediator`), where safety is applied exactly once; and **everything the operator sees**
comes from `/dt/*` topics, never from raw robot topics. The hard logic lives in small **pure
Python libraries** (no ROS imports), each unit-tested; the nodes are thin wiring around them.

### Topic naming — who owns which channel
A course-critical rule: **the real robot and the simulated robot must never publish the same
topic name** (else messages mix and chaos follows). Convention:
- **bare names** (`/scan`, `/odom`, `/cmd_vel`, `/battery_state`) = the **active** robot —
  the Gazebo robot in `sim_only`, the real Burger in `real_only`/`both`;
- **`/sim/...`** = the mirroring simulation in `both` mode (`/sim/scan`, `/sim/odom`,
  `/sim/cmd_vel`, `/sim/tf`, even `/sim/robot_description` and `/sim/joint_states`);
- **`/dt/...`** ("digital twin") = our own layer: the command bus, mirrored state, status,
  alerts, mission control.

### The three modes
The same launch file starts three different configurations
(`ros2 launch algae_dt bringup.launch.py mode:=...`):

| mode | what runs | used for |
|---|---|---|
| `sim_only` (default) | Gazebo + Nav2 + our layer. The sim robot IS the robot. | development at home; backup demo |
| `real_only` | Nav2 + our layer on the laptop; the real Burger runs its own bring-up on its Raspberry Pi. | first lab validation |
| `both` | real robot leads on bare topics **and** Gazebo mirrors it on `/sim/*`. | the full digital-twin demo |

Two mode-dependent rules to remember:
- **`use_sim_time` is true only in `sim_only`** (only there does a Gazebo clock drive everything).
- **RViz opens automatically in `real_only`/`both`** because that is where a human must click
  "2D Pose Estimate" to tell AMCL where the robot really starts. In `sim_only` AMCL is auto-seeded
  at the spawn point, so no click and no RViz are needed.

---

## 4. The seven nodes, one by one

### 4.1 `twin_mediator.py` — the heart (the fan-in / fan-out chokepoint)

**What it does, simply:** it is the only thing allowed to talk to the robots' motors. Everything
that wants to drive (Nav2, the keyboard teleop, the mission's spray spin) publishes onto one
shared bus, `/dt/cmd_vel_raw`. Twenty times per second the mediator takes the latest bus command,
asks "is it safe?", clips it to the robot's physical limits, and forwards the result to the real
robot (`/cmd_vel`) — and in `both` mode the identical command to the sim (`/sim/cmd_vel`). That's
the "fan-in" (many sources, one bus) and "fan-out" (one decision, both worlds).

**The 25 cm dual-LiDAR safety gate.** Each cycle it looks at the latest laser scan from each
world and finds the nearest valid reading in a ±20° cone dead ahead (`lib/safety.py`). If
**either** world sees something closer than 0.25 m, forward motion is zeroed **for both robots**
(turning and reversing stay allowed, so the robot can still escape). The gate **fails safe**:
- a scan that previously existed but stopped arriving (stale) counts as *blocked* — a dead sensor
  must stop the robot, not blind it;
- a world that never produced data (e.g. there is no real robot in `sim_only`) is simply ignored —
  otherwise the absent world would permanently block everything;
- the mirror sim gets a slightly looser staleness budget (2 s vs 1 s) because a slow computer
  renders the simulated LiDAR at only ~3 Hz, and one late sim frame must not brake the real robot.

**The E-STOP.** The mediator owns the emergency stop. It is **latched**: once tripped (by the
operator's button or automatically by a critically low battery) it forces full stop and *stays*
tripped until the operator explicitly presses Resume. The battery trip latching matters: a tired
battery's voltage sags under load and recovers at rest — without the latch, the E-STOP would
silently release itself every time the voltage bounced back. Resume clears every cause at once,
and a still-critical battery immediately re-trips on the next reading.

**State mirroring.** The mediator also translates raw robot state into operator-friendly `/dt/*`
topics: it lifts odometry into the **map frame** (using AMCL's correction) and publishes it as
`/dt/real_pose` / `/dt/sim_pose`; it republishes the active scan as `/dt/scan_active` and raw
odometry as `/dt/odom_active`; it forwards battery state to `/dt/health`; and it publishes the
latched `/dt/estop`, `/dt/mode` and `/dt/safety` flags.

**The commanded shadow (sim_only's clever trick).** In `sim_only` there is no real robot, so what
do you compare the sim against, to demonstrate pillar ② (sync)? Answer: against *what was
commanded*. The mediator integrates the safe commands mathematically (a "unicycle model" — see
`lib/sync.py`) into an ideal pose, the **shadow**, and publishes it as `/dt/real_pose`. The sync
supervisor then measures "how far is the *achieved* sim pose from the *commanded* pose" — a real,
meaningful tracking error, with no hardware. The GUI labels this pose "commanded shadow" so
nobody mistakes it for a real robot.

**The synthetic battery.** Same idea: in `sim_only` a fake battery drains slowly so the battery
banner does something. It floors *above* the critical threshold on purpose (an ordinary demo run
should never E-STOP itself). For the demo's "battery → auto-E-STOP" moment you force it:
`ros2 topic pub --once /dt/battery_override_v std_msgs/msg/Float64 "{data: 10.0}"`.

### 4.2 `sync_supervisor.py` — the measuring instrument (pillar ②)

**What it does, simply:** it constantly answers "how well are the two worlds in sync?", with
numbers, and complains loudly when the answer is "not well".

Five times a second it computes and publishes:
- **pose discrepancy** (`/dt/sync_error`): distance and heading difference between
  `/dt/real_pose` and `/dt/sim_pose` (both map-frame, so the comparison is meaningful).
  In `real_only` there is no sim, so this degenerates to zero by design;
- **sensor discrepancy**: difference between the two worlds' front laser distances;
- **command→motion latency** (`/dt/latency_ms`): when a drive command appears on `/cmd_vel`,
  how long until the odometry actually shows movement;
- **stop skew**: when a safety stop fires in `both` mode, how far apart in time did the real and
  sim robots actually halt — the "they stop together" evidence;
- **in tolerance?** (`/dt/sync_ok`): all of the above compared to documented thresholds from
  `twin.yaml`, plus `/dt/alerts` text messages when something exceeds them;
- **a CSV file** (`sync_metrics_*.csv`): one row per tick — the trustworthy evidence file for
  grading. Row count is capped so a forgotten node can't fill the disk.

**The one rule that makes the numbers honest — one clock.** All latency/skew timing uses the
supervisor's **own clock at message arrival**, never the timestamps inside the messages. Why?
The real robot stamps messages with its Raspberry Pi's clock, the laptop has its own clock (the
two are *not* synchronized over lab Wi-Fi), and the simulator stamps with sim time which starts
near zero. Subtracting timestamps across those clock domains once produced "stop skews" of
50 years. One observer, one clock, real durations.

**Alert discipline.** Alerts are **edge-triggered**: they fire when a condition *becomes* true,
then repeat at most every 5 s while it stays true (instead of 5 times per second, forever).
A measured latency sample is judged exactly once. A missing **required** pose stream (which
streams are required depends on the mode) flips `sync_ok` to false and alerts — failing loud
rather than going silently dark.

### 4.3 `mission_runner.py` — the autopilot conductor (pillar ③)

**What it does, simply:** runs the cleaning mission. It keeps a list of blooms (from the GUI),
and in a loop: pick the nearest untreated bloom → ask Nav2 to drive there → on arrival, spin
three full turns ("spray") → mark it green → next. The loop runs on a worker thread so the node
stays responsive to Stop/E-STOP at all times.

The interesting mechanisms:

- **Goal projection (the near-wall fix).** If the operator clicks a bloom right next to a wall,
  the exact click point is unreachable — Nav2 refuses to park the robot *inside* the wall's
  safety margin and gives up confusingly. So before sending the goal, the mission projects it
  through the static map (`lib/occupancy.py`) to the **nearest point with ≥ 0.30 m clearance from
  every obstacle**, and sends *that*. A bloom that has no such point within half a metre is
  declared unreachable immediately (greyed out) instead of letting Nav2 churn for a minute.

- **Arrival is checked against the projected goal.** Since the robot was *sent* to the projected
  point (possibly half a metre from the raw click), the sanity check "did we roughly get there?"
  must measure to the projected point too. (Measuring to the raw click skipped exactly the
  near-wall blooms the projection was built to save.)

- **Closed-loop spray.** "Spin three full turns" is measured from the odometry's actual heading,
  accumulated turn by turn (wrap-around handled by `lib/geometry.angle_diff`), not by spinning
  for a fixed time. On a slow simulator the robot turns slower than commanded — a timed spin
  would under-rotate; the closed loop always completes the true count. Two watchdogs protect it:
  if the heading **stops changing** for 3 s (odometry frozen), abort; and an absolute time cap as
  a belt-and-braces backstop. Both watchdogs use wall-clock, not sim time, on purpose.

- **Honest accounting.** A bloom is marked *treated* (green) only after a confirmed arrival
  **and** a completed full spray. Failed navigation → *skipped* (grey). Operator Stop / E-STOP →
  back to *pending* (yellow, resumable). Even if the worker thread crashes, the in-flight bloom
  is put back to pending — never left looking "in progress" and never falsely marked done. The
  rubric (and basic honesty) demand the display never claims work that didn't happen.

- **Frames, again.** Nearest-bloom selection and the arrival check use the robot's **map-frame**
  pose (blooms live on the map). The spray loop uses **odometry** heading (it only needs *change*
  of angle, and odom is smooth and high-rate). Each data source is used for what it is good at.

### 4.4 `operator_gui.py` — the operator console

**What it does, simply:** a PyQt5 window. Left: the arena map with the robot(s), the live laser
points, and the bloom markers (yellow = pending, blue = active, green = treated, grey =
skipped); click to place a bloom. Right: status banners (mode, mission, sync, latency, battery,
safety, E-STOP, alerts) and buttons (Start / Stop / Clear / E-STOP / Resume).

Design choices worth understanding:
- It is split into a Qt-free `GuiBridge` (the ROS side — testable without a display) and the
  Qt window built around it. The GUI subscribes **only** to `/dt/*` topics — it knows nothing
  about raw robot topics, which keeps the operator's view going through the mediator's single
  source of truth.
- Qt and ROS each want to own the program's event loop; here Qt owns it, and a 25 Hz Qt timer
  *drains* all pending ROS messages each tick (up to a bounded batch — draining only one message
  per tick made the display lag behind reality).
- The map image ↔ world maths: the map is a grayscale image (PGM) where row 0 is the *top*, but
  in world coordinates y grows *up* — so there is a vertical flip in every conversion
  (`lib/geometry.world_to_pixel` / `pixel_to_world`). Clicks in the dark margins around the map
  are rejected (they used to become phantom off-map blooms).
- The laser overlay is anchored to the pose of whichever robot the scan actually came from
  (the sim robot in `sim_only`, the real one otherwise) — anchoring it to the wrong robot made
  the overlay visibly detach exactly when the twins diverged.

### 4.5 `fake_robot.py` — the hardware stand-in

**What it does, simply:** pretends to be the real Burger so the `real_only` and `both` modes can
be exercised at home with zero hardware. It subscribes the gated `/cmd_vel`, integrates the
commands into a pose (the same unicycle maths as the shadow), and publishes everything a real
Burger would: `/odom`, a synthetic 360-beam `/scan` (with a configurable fake obstacle dead
ahead — set it under 0.25 m to test the safety stop), `/battery_state`, and the **full TF chain**
(`odom → base_footprint → base_link → base_scan`) that AMCL and Nav2 need to make sense of the
laser. Like the real robot, it **stops when commands stop arriving** (0.5 s timeout) instead of
coasting forever.

### 4.6 `dynamic_obstacle.py` — the "live environment change" prop

**What it does, simply:** spawns a box into the running Gazebo world and sweeps it back and forth
across the robot's path (a sine wave, maths in `lib/trajectory.py`), by calling Gazebo's
`create`/`set_pose` services. The robot's LiDAR sees a *moving* obstacle → the safety gate stops
the robot and/or Nav2 plans around it — the scripted way to show pillar ③'s "introduce an
environment change live" in simulation. Launch it with `use_dynamic_obstacle:=true`.

Safety details: it retries the spawn until Gazebo *confirms* it (never teleports a box that
doesn't exist), re-spawns if the box vanishes (e.g. a world reset), and — because the box is
teleported with **no physics** — it clamps its sweep so it can never pass through the robot's
spawn point.

### 4.7 `twin_resync.py` — the drift corrector (pillar ②, `both` only)

**What it does, simply:** the mirror sim follows the *same commands* as the real robot, not the
real robot itself — so over time wheel slip, motor lag and physics differences make the two poses
drift apart. The supervisor *measures* that drift; this node *fixes* it: it teleports the sim
robot onto the real robot's pose (Gazebo `set_pose`, same mechanism as the obstacle box) either
when the operator presses **RESYNC TWIN** in the GUI, or automatically once the error has stayed
beyond the documented tolerance for a few seconds. Think "predict with the model, correct with
the data" — the standard state-estimation loop, applied to the whole twin.

The rules that keep it safe (all in pure, unit-tested `lib/resync.py`): a transient error spike
(e.g. during a spray spin) never triggers it — the breach must be *sustained*; attempts are spaced
by a cooldown so a stuck Gazebo can't be hammered; and it refuses to act on stale data ("never
teleport onto a target the data doesn't support"). Every executed resync is announced on
`/dt/resync_event`, shown on the GUI SYNC banner, and stamped into the evidence CSV's `resync`
column; a failed Gazebo call raises a loud `/dt/alerts` instead.

The piece that makes the teleport *honest*: in `both` mode `/dt/sim_pose` is Gazebo's
**ground-truth** model pose (`worlds/burger_sim_gt.sdf` adds gz's PosePublisher to the stock
burger; bridged as `/sim/ground_truth`), not integrated odometry. Odometry ignores teleports — a
snap under the old source would have moved the robot while the number stayed wrong. Ground truth
moves the pose, the LiDAR viewpoint and the measured error together. Bonus: this also solves
start-up alignment — the sim always spawns at the map origin, the real robot stands wherever it
stands, so the first 2D Pose Estimate puts the error out of tolerance and the twin snaps itself
onto the real start within seconds.

---

## 5. The pure libraries (`algae_dt/lib/`)

All the actual logic lives in small modules with **no ROS imports**, so each is unit-testable on
any machine with plain `pytest`. The nodes are thin shells around them.

| module | in one sentence |
|---|---|
| `safety.py` | The 25 cm gate maths: find the nearest valid front laser return, fail-safe staleness rules, combine two worlds (either one blocks both), clamp/zero the command. |
| `sync.py` | Pose/sensor discrepancy maths + tolerance check + the unicycle integrator used by the shadow pose and the fake robot. |
| `metrics.py` | Latency arithmetic and the sync CSV row/header formatting (stable columns, NaN/∞-safe). |
| `geometry.py` | World ↔ map-pixel transforms (with the y-flip), yaw ↔ quaternion, wrap-safe angle difference, 2D pose composition (the map←odom lift), shared pose/MapInfo helpers. |
| `occupancy.py` | The static map as a free/blocked grid + **goal projection**: nearest cell with enough clearance to every obstacle (the near-wall fix). Clearance accounts for obstacle cells being squares, not points. |
| `blooms.py` | The bloom list as **immutable** data: every state change returns a new list (pending → active → treated/skipped, with "aborted goes back to pending"). Immutability means the mission thread and ROS callbacks can never half-update it. |
| `pgm.py` | A tiny parser for the map image format (PGM), so the GUI doesn't depend on Qt image plugins. |
| `hud.py` | Display helpers: battery color thresholds, battery %, laser points projected to x/y, status texts. |
| `trajectory.py` | The obstacle's sine sweep + the keep-out clamp (distance from a point to the swept segment). |
| `resync.py` | The drift-correction policy: WHEN may the twin snap the sim onto the real pose (manual = now, auto = sustained breach only, cooldown between attempts, stale/NaN data refuses). Immutable state, pure function. |
| `gzcli.py` | The `gz service` CLI plumbing shared by `dynamic_obstacle` and `twin_resync`: request text composition (validated names, finite coordinates) + Boolean-reply parsing, with an injectable runner so tests never need a sim. |
| `ros_utils.py` | The only ROS-adjacent helper module: the declare-and-get parameter idiom and the installed-map loaders (used by several nodes; previously copy-pasted). |

---

## 6. Configuration — `config/twin.yaml`

One file holds every tunable, loaded by the launch file into **all** nodes (the `/**:` header
means "apply to every node"). Nothing is hardcoded in the nodes; every value in the file carries
a comment saying where it comes from and why that magnitude. Highlights:

- **Safety:** `stop_distance_m: 0.25` (the course spec), `front_sector_rad: 0.70` (the ±20° cone),
  the LDS-02 LiDAR's valid range (0.12–3.5 m), the Burger's real speed limits, staleness budgets
  (1 s real / 2 s sim).
- **Sync tolerances (the graded thresholds):** 15 cm position, ~15° heading, 20 cm sensor
  disagreement, 250 ms latency budget, 300 ms stop-skew budget — each with a rationale comment.
- **Spray:** 3.0 revolutions at 2.8 rad/s (fast enough to *look* like spinning even on a slow
  sim), the 3 s stall watchdog, the generous absolute cap.
- **Mission:** bloom radius, 0.50 m arrival bound (doubles as the max goal-projection distance),
  0.30 m goal clearance, 60 s navigation timeout.
- **Battery:** the GUI's amber/red thresholds, the 10.5 V auto-E-STOP trip, the synthetic
  battery's behaviour, and the demo override topic.
- **Map metadata:** resolution and origin of the 86×110-pixel arena map (must match
  `maps/map.yaml`).

`config/sim_bridge.yaml` is the Gazebo↔ROS bridge table for `both` mode: which Gazebo topics map
to which `/sim/*` ROS topics, with which message types, in which direction.

---

## 7. The launch file — `launch/bringup.launch.py`

One file brings up everything, in three flavours (`mode:=sim_only|real_only|both`). What it does:

1. **Environment:** sets the TurtleBot3 model name and Gazebo's resource path.
2. **Per mode:**
   - `sim_only`: starts the Gazebo server (+ optionally its 3D window), the robot model
     publisher, and spawns the Burger at the origin.
   - `both`: starts the Gazebo *mirror* — the same, but every ROS-facing channel pushed into
     `/sim/*` (custom bridge config; the robot-state-publisher's `/tf`, `/tf_static`,
     `/robot_description` and `/joint_states` all remapped) so nothing collides with the real
     robot's bare topics.
   - `real_only`: starts no simulator at all (the real robot's own computer runs its bring-up).
3. **Nav2** in every mode, with our three surgical setting rewrites:
   - reroute Nav2's final output velocity onto our command bus (`/dt/cmd_vel_raw`) — this is what
     puts the safety gate between Nav2 and the motors;
   - in `sim_only`, auto-seed AMCL at the spawn (no human click needed) and loosen two timing
     checks that a slow software-rendered LiDAR would otherwise trip.
4. **Our four nodes** (+ `fake_robot` / `dynamic_obstacle` when their flags are set).
5. **RViz** — automatically in `real_only`/`both` (the operator *must* click 2D Pose Estimate
   there), opt-in elsewhere.

Two robustness details: the Gazebo **3D window** runs as *non-critical* (if it crashes — which
weak graphics stacks are known to do — the demo keeps running without it; `gz_gui:=false` skips
it entirely), and the robot's spawn pose is a named constant tied by comment to AMCL's seed pose
so nobody changes one without the other.

---

## 8. Scripts and Docker tooling

**At home** everything runs inside a Docker image (`algae-dt:dev`) containing ROS 2 Jazzy + the
TurtleBot3 stack + Gazebo — the home machine needs nothing but Docker. **The lab PC is the
opposite (verified 2026-06): it has NO Docker** — ROS 2 Jazzy is installed natively and the
TurtleBot3 stack is built from source in `~/turtlebot3_ws`, so there the demo runs natively.

| file | purpose |
|---|---|
| `scripts/lab_run.sh` | **The lab-laptop runner.** From a fresh clone: auto-detects the runtime — Docker where it exists (rootful required for `both`: rootless Docker cannot reach the robot's network traffic), otherwise **native** (the lab PC; `--native` forces it). Recreates the package in the workspace, requires the real robot to be reachable and actually publishing `/scan`, builds, and launches the full `both` demo (RViz included). `--sim` = hardware-free fallback; `--no-gz-gui` = skip the 3D window; `--rebuild` = clean build. It *refuses to start a half-demo*: if `both` can't run, it says exactly why. |
| `scripts/lab_fix_workspace.sh` | **Lab workspace recovery (no sudo, no Docker).** A `~/turtlebot3_ws` copied from another machine/user is unsalvageable in place (absolute CMake-cache paths + foreign-owned files). This rebuilds it from the fail-safe zip in `~/Downloads`: extracts only `src/`, quarantines the broken dirs by rename (local experiments kept), fixes permissions, repairs `~/.bashrc`, clean-rebuilds in a sanitized environment, verifies. Tested by `scripts/test_lab_fix_workspace.sh` (57-assertion container suite). |
| `scripts/Dockerfile` | Fallback image build for Docker-capable machines if no turtlebot3 image exists. |
| `docker/build.sh` / `run.sh` (+ `.ps1`) | Build/enter the dev image at home. |
| `docker/ci.sh` | **The CI gate:** clean `colcon build` + the entire test suite, headless. |
| `docker/sim_smoke.sh` | Headless end-to-end `sim_only` boot check: Nav2 active, AMCL localized, topics flowing with the right types. |
| `docker/mission_smoke.sh` + `mission_probe.py` | Headless full-mission check: places a bloom, starts the mission, asserts the robot moved, sprayed, and the bloom turned treated. |
| `docker/open_sim.sh` | Open the interactive sim with GUI windows (operator console + RViz) under WSLg. |
| `docker/demo_run.sh` | Convenience demo launcher. |

---

## 9. The tests — what protects what

The suite (in `ros2_ws/src/algae_dt/test/`) runs entirely inside the container via
`bash /ci/ci.sh`. Two layers:

1. **Pure-library tests** (no ROS needed): every branch of the safety gate maths (including the
   nasty ones — beam angle wrap-around, NaN/∞ beams, the 0.0 "no return" sentinel, staleness),
   geometry round-trips (a map cell must survive pixel→world→pixel exactly), occupancy
   projection (including off-map clicks and the obstacle-edge clearance), bloom state
   transitions, CSV formatting, the obstacle sweep and its keep-out clamp.

2. **In-process integration tests** (rclpy, still no simulator): real node objects wired to small
   test harnesses over real topics. These pin the *behaviours* that earlier reviews found broken,
   so they can never silently regress — among them:
   - commands pass the gate when clear / forward is zeroed at 25 cm / E-STOP latches;
   - the battery E-STOP **stays latched when the voltage recovers** and clears on Resume;
   - stop-skew and latency are **immune to mixed clock domains** (the regression test literally
     stamps one stream with epoch wall-time and the other with small sim-times);
   - `real_only` runs healthy without a sim world (no alert storm, CSV collects rows);
   - one bad sample alerts **once**, not five times a second;
   - near-wall blooms: goal projected off the wall, **arrival measured against the projected
     goal**, boxed-in blooms skipped fast without bothering Nav2;
   - the spray spins to the **achieved** turn count on a slow robot, the stall watchdog ends a
     frozen spin, and an interrupted spray never marks the bloom treated;
   - a crashed mission worker reverts its in-flight bloom to pending;
   - the fake robot stops on command silence and broadcasts the full TF chain;
   - GUI: margin clicks rejected, alerts ingested, scan anchored to the active robot's pose.

The guiding principle: **tests assert the documented invariant, not the current behaviour** —
several earlier tests were rewritten because they accidentally pinned a bug in place (e.g. they
fabricated message timestamps in a way no real system ever would, hiding the clock-domain bug).

---

## 10. Safety design, summarized (why it fails safe)

| situation | behaviour | why |
|---|---|---|
| obstacle < 25 cm ahead, either world | forward zeroed on BOTH robots; turn/reverse still allowed | either world is evidence; escape must stay possible |
| a laser stream goes stale | treated as blocked | a dead sensor must stop the robot, not blind it |
| a world never had data (absent) | ignored by the gate | an absent world isn't a hazard; it must not freeze the present one |
| command bus goes silent > 0.5 s | output zeroed (watchdog) | a crashed commander must not leave the last command running |
| battery ≤ 10.5 V | E-STOP, **latched** | sagging voltage recovers at rest; only a human may re-enable |
| E-STOP pressed | full stop both worlds, latched until Resume | the operator is the final authority |
| odometry freezes mid-spray | spray aborts in 3 s, bloom NOT marked treated | never report work that didn't happen |
| known blind spot: objects closer than the LiDAR's 12 cm minimum read as "no return" | documented; the 25 cm stop normally prevents ever getting that close | physics of the sensor — software cannot distinguish "too close" from "nothing there" |

---

## 11. Glossary

| term | meaning |
|---|---|
| **digital twin** | a live virtual copy of a real system, synchronized both ways |
| **node / topic / message** | a ROS program / a named channel / the data on it |
| **`TwistStamped`** | a velocity command (forward m/s + turn rad/s) with a timestamp header |
| **LiDAR / `LaserScan`** | spinning laser ranger / its message: 360 distances around the robot |
| **odometry (`/odom`)** | position integrated from wheel motion — smooth but drifts |
| **AMCL** | localization: matches laser vs map to correct the drift (the map←odom transform) |
| **TF / frame** | the live tree of coordinate systems (map, odom, robot body, laser…) |
| **Nav2 / `BasicNavigator`** | the navigation stack / its simple Python interface |
| **Gazebo / RTF** | the physics simulator / how fast sim time runs vs real time (1.0 = real-time) |
| **QoS / latched** | per-topic delivery rules / "late subscribers get the last value" |
| **costmap / inflation** | Nav2's obstacle grid / the safety margin puffed around obstacles |
| **2D Pose Estimate** | the RViz click that tells AMCL the robot's true starting pose |
| **fail-safe** | when something breaks, the system moves to the SAFE state (stopped) |
| **E-STOP (latched)** | emergency stop that stays engaged until explicitly cleared |
| **the bus (`/dt/cmd_vel_raw`)** | the single pre-safety command channel everyone publishes into |
| **the gate** | the mediator's per-cycle safety decision (block forward or not) |
| **shadow pose** | sim_only's "what was commanded" reference pose for sync measurement |
| **bloom** | one algae target: pending → active → treated / skipped (honest accounting) |
| **stop skew** | time difference between the two worlds halting on a safety event |
| **pillars ①②③** | the three graded criteria: bidirectional, state sync, environmental |
