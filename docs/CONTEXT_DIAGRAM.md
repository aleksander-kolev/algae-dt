# CONTEXT_DIAGRAM.md — algae-dt Digital Twin system context

Course "Introduction to Context Diagrams": the system boundary, external entities, and the
information flows crossing it. The **`algae_dt` DT layer** is the system; the stock turtlebot3
packages, the operator, the real robot, the sim, and any online source are external to it.

```
                                  ┌──────────────────────────────────────────────┐
                                  │            algae_dt  (our system)             │
   ┌───────────┐  click-to-place │  ┌───────────────┐      ┌──────────────────┐  │
   │  OPERATOR │ ───/dt/blooms──▶ │  │ operator_gui  │      │  mission_runner  │  │
   │  (PyQt5)  │ ◀── /dt/* state─ │  │ (subs /dt/*)  │      │ (Nav2 to bloom + │  │
   └───────────┘                  │  └───────────────┘      │  3-spin spray)   │  │
                                  │        │ start/stop      └────────┬─────────┘  │
                                  │        ▼ /dt/mission_cmd           │ /dt/cmd_vel_raw
   ┌───────────────┐  /scan /odom │  ┌──────────────────────────────────────────┐ │
   │  REAL BURGER  │  /battery    │  │            twin_mediator                  │ │
   │ (turtlebot3_  │ ───────────▶ │  │  fan-IN: /scan /odom /battery /sim/scan   │ │
   │  bringup, Pi) │              │  │  safety gate (25 cm, fail-safe)           │ │
   │               │ ◀─/cmd_vel── │  │  fan-OUT: /cmd_vel(TS) + /sim/cmd_vel     │ │
   │               │(TwistStamped)│  │  /dt/scan_nav→ costmaps+collision_monitor │ │
   └───────────────┘              │  │  mirror→ /dt/real_pose /dt/sim_pose       │ │
                                  │  │  latched /dt/estop                        │ │
   ┌───────────────┐  /sim/scan   │  └───────────┬──────────────────────────────┘ │
   │  SIM BURGER   │  /sim/odom   │              │ /dt/sync_error /dt/latency_ms    │
   │ (turtlebot3_  │ /sim/ground_ │  ┌───────────▼───────────┐  ┌────────────────┐ │
   │  gazebo +     │  truth ─────▶ │  │   sync_supervisor      │  │  twin_resync   │ │
   │  PosePublisher│              │  │  pose/sensor Δ, latency│  │ (both: snap sim│ │
   │  + Nav2/AMCL) │ ◀/sim/cmd_vel│  │  vs twin.yaml tolerances│ │  onto real pose│ │
   └───────▲───────┘              │  └─────┬──────────────────┘  └───────┬────────┘ │
           │ gz set_pose (teleport)        │ /dt/alerts /dt/sync_ok ──▶ (GUI+CSV)   │
           └──────────────────────│────────│────────────────────────────┘          │
   ┌───────────────┐  weather/UV  │   (OPTIONAL T6.1b context_adapter:              │
   │ ONLINE SOURCE │ ───────────▶ │    online source → cautious mode in both worlds)│
   │ (Open-Meteo)  │              └──────────────────────────────────────────────┘
   └───────────────┘
```

**Boundary flows (the graded bidirectional contract):**
- **In:** real `/scan /odom /battery_state`; sim `/sim/scan /sim/odom` (motion/stop-skew) +
  `/sim/ground_truth` (the `both`-mode `/dt/sim_pose` source — gz PosePublisher, world==map frame);
  operator clicks; (optional) online source.
- **Out:** `/cmd_vel`(TwistStamped→real), `/sim/cmd_vel`(→sim), `/dt/scan_nav`(→Nav2 costmaps +
  collision_monitor obstacle sources: real scan + trusted-mirror overlay; AMCL keeps `/scan`),
  gz `set_pose` (twin_resync's
  bounded-drift teleport of the mirror, operator/auto, gated on `/dt/localized`), `/dt/*` state to
  the GUI + CSV.
- **Single chokepoint:** all motion commands pass through `twin_mediator` (fan-in/fan-out + safety).

**External (reused, not ours):** `turtlebot3_gazebo`, `turtlebot3_navigation2` (Nav2/AMCL),
`turtlebot3_bringup`, `turtlebot3_teleop`. Render this as a proper diagram (draw.io / Mermaid) for
the Week-4 review; this ASCII version is the source of truth for the flows.
