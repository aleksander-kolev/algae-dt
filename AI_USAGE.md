# AI usage and authorship declaration

This document declares the use of generative-AI assistance in the **algae-dt** project, for
transparency in line with the TU/e 2IRR10 guidelines on the use of generative AI and LLMs.

## Authorship and ownership

- **Code owner / author:** the algae-dt team (TU/e 2IRR10, Team 36). The repository is owned by
  Aleksandar Kolev (GitHub `aleksander-kolev`).
- **Team members:** Yannick van Dooren, Bulcsú Gyeszli, Stan Jetten, Aleksandar Kolev,
  Kremena Kroumova, Yağmur Özdemir, David Stancu.
- **How the work was done:** the team produced all of the requirements and designed and architected
  the entire system itself — every component, what it does, how it behaves, and how the pieces fit
  together. The provided course documents and code were not pasted into the AI tool; the team
  described its own design to the assistant in its own words (see the prompt log below). The
  assistant then helped turn that design into code under the team's direction, and the team
  reviewed, corrected, tested, and integrated the result. The team owns the work and can explain the
  architecture, the code, and how they align with the Proof-of-Concept requirements.
- **AI assistant (the team's coding copilot):** Claude (Opus model) via Claude Code, by Anthropic.
  It acted as a copilot that helped implement the team's design and sped up debugging, always under
  the team's direction. It did not design, architect, own, or decide the solution.

## What the AI was used for

- **Implementation help** — turning the team's designs and specifications into code, with the team
  defining each component's behaviour and reviewing and correcting the output: the pure helper
  modules (the world/map-pixel transforms, the LiDAR front-sector safety gate, bloom bookkeeping,
  the sync maths and unicycle integrator, the PGM parser, occupancy and goal projection), the node
  logic (mediator, sync supervisor, mission runner, operator GUI, fake robot, dynamic obstacle), and
  the launch wiring.
- **Domain understanding** — explaining terminology and concepts (digital twinning, ROS 2
  topics/TF/frames, Nav2 and AMCL, occupancy/SLAM maps, the LDS-02 LiDAR) so the team could specify
  the work precisely.
- **Debugging** — locating and explaining errors (the Jazzy `TwistStamped` `/cmd_vel` contract,
  headless AMCL seeding, Nav2 collision-monitor timeouts under a slow simulation, projecting
  navigation goals clear of walls).
- **Packaging and documentation** — assembling the runnable repository, tidying comments, and
  writing the `README.md` and this file.

Every result was checked against the running system and the course material, then reviewed and
integrated by the team. The architecture, the engineering trade-offs, and the alignment between the
technical solution, the PoC requirements, and the code are the team's own.

## What the team did, not the AI

- Produced all the requirements, and designed and architected the whole system and every component.
- Made the engineering trade-offs and decided how each part should behave.
- Wrote the specifications and instructions the assistant implemented from.
- Verified every result against the running system and the course material.
- Kept the understanding needed to explain and defend the design, the diagrams, and the code.

## Tool

- **Tool:** Claude Code (Anthropic), Claude Opus model.
- **Role:** the team's coding copilot / assistant — it helped implement the team's design and helped
  with debugging and packaging.

## Prompt log

Development was iterative across many sessions. The prompts below are **representative** of the
team's own design specifications and instructions to the assistant (grouped by area) — they show the
architecture and component designs the team produced and then had the assistant implement. The final
section summarises the packaging and documentation help the team asked the assistant for.

### 1. Domain and concepts
- "Explain what a digital twin is for a TurtleBot3, and what 'bidirectional' state synchronisation
  between a real robot and a Gazebo simulation means in practice."
- "In ROS 2, explain the relationship between the `map`, `odom`, and `base_link` frames, and how
  AMCL publishes the `map -> odom` transform."
- "How does Nav2's `BasicNavigator` (`nav2_simple_commander`) work — how do I send a goal pose and
  wait for the result?"
- "What does a `sensor_msgs/LaserScan` from an LDS-02 contain (angle_min, angle_increment, ranges),
  and how do I find the minimum range within a forward cone?"
- "On ROS 2 Jazzy, why is `/cmd_vel` sometimes `geometry_msgs/TwistStamped` instead of `Twist`, and
  what publishes/subscribes which?"

### 2. Architecture and scaffolding (our design, handed to the assistant to build)
- "Our design is one custom ROS 2 package on top of the stock turtlebot3 packages (gazebo,
  navigation2, bringup, teleop) without re-implementing them, with these nodes: a mediator, a sync
  supervisor, a mission runner, and an operator GUI. Set up the package layout and node skeletons to
  match."
- "Implement our topic contract: the active robot on bare topics, a mirror sim under `/sim/*`, and a
  `/dt/*` bus for everything the GUI and the sync monitor consume — the real and sim worlds must
  never publish the same topic name."
- "Wire up our command chokepoint: every velocity source (teleop, GUI, Nav2) goes to one
  `/dt/cmd_vel_raw` topic, passes one safety gate, then fans out to `/cmd_vel` and (in `both`)
  `/sim/cmd_vel`."
- "Keep all pure logic (geometry, the safety gate, bloom bookkeeping, sync math) in ROS-free modules
  so we can unit-test them; the nodes are thin wiring around them."

### 3. Pure library modules
- "Write `world_to_pixel`/`pixel_to_world` for an occupancy map (top-left image origin), such that a
  cell centre round-trips exactly. Include yaw<->quaternion helpers and a planar pose composition."
- "Implement a fail-safe forward-collision check: minimum valid range in a full-width front sector,
  treat a scan that goes stale as blocked, stay unblocked before any data arrives, and OR-combine
  two LiDARs so either world can stop both."
- "Write an immutable bloom field: pending/active/treated/skipped states, transitions that return
  new objects, a nearest-untreated query, and a retry-skipped reset."
- "Implement the real-vs-sim discrepancy maths (XY, yaw shortest-arc, front-range delta) and an
  exact constant-velocity unicycle integrator for a commanded 'shadow' pose."
- "Write a small P2/P5 PGM parser (handle `#` comments and maxval scaling) so the GUI can draw the
  map without depending on Qt's image plugins."
- "Given the static map, project a goal to the nearest cell with a given clearance to every
  obstacle, so Nav2 never gets a goal inside the costmap inflation."

### 4. Nodes
- "Implement the mediator node: subscribe `/dt/cmd_vel_raw` and both scans, apply the safety gate +
  Burger speed limits on a timer, publish `/cmd_vel` (+ `/sim/cmd_vel` in `both`), mirror
  pose/scan/battery onto `/dt/*`, and own a latched `/dt/estop` that auto-trips on low battery."
- "Implement the sync supervisor: measure pose/sensor discrepancy and command->motion latency,
  capture the real-vs-sim stop-time skew on a safety event, publish `/dt/sync_*`, append a CSV row
  each tick, and raise `/dt/alerts` when out of the documented tolerances; fail loud if a pose
  stream goes silent."
- "Implement the mission runner on a worker thread: pick the nearest untreated bloom, send a Nav2
  goal (projected off walls), and on success spin in place for exactly N revolutions measured
  closed-loop from odometry yaw. Keep bloom accounting honest on Stop/E-STOP/failure."
- "Build a PyQt5 operator console that subscribes only to `/dt/*`: a map canvas with real/sim pose
  and live scan overlay, click-to-place blooms, Start/Stop/Clear/E-STOP buttons, and status
  banners. It must also run headless (`QT_QPA_PLATFORM=offscreen`)."
- "Write a kinematic `fake_robot` that publishes the real robot's bare topics and integrates
  `/cmd_vel`, so `real_only`/`both` can be exercised at home without hardware."
- "Write a `dynamic_obstacle` node that spawns and sweeps a box via the Gazebo services for a
  live-environment-change demo; make the gz calls best-effort and never fatal."

### 5. Integration and launch
- "Write `bringup.launch.py` (OpaqueFunction) with `mode:=sim_only|real_only|both` plus `headless`,
  `use_rviz`, `use_fake_robot`, `gz_gui`. Include the stock turtlebot3 model/RSP/spawn, Nav2 + AMCL
  + the map, and our DT nodes."
- "Use Nav2's `RewrittenYaml` to set `enable_stamped_cmd_vel` and remap the controller's
  `cmd_vel_out_topic` to `/dt/cmd_vel_raw` so Nav2's final velocity passes through our safety gate."
- "For `both`, namespace the mirror sim under `/sim/*` with a custom `ros_gz` bridge and a prefixed
  robot_state_publisher, with sim TF off the global `/tf`."
- "Put every tunable in `config/twin.yaml` and bind it to all nodes via the `/**` wildcard."

### 6. Debugging and tuning
- "Stock teleop and the real bringup expect `TwistStamped` on Jazzy, but Nav2 publishes `Twist` —
  how do I make the whole bus `TwistStamped` consistently?"
- "Headless `sim_only` won't localize and the robot won't move; how do I auto-seed AMCL
  (`set_initial_pose`) and stop the collision monitor rejecting scans on a slow software-rendered
  LiDAR (`source_timeout`)?"
- "Nav2 aborts with 'failed to make progress' under a throttled sim — which progress-checker
  parameters do I loosen for sim only?"
- "A bloom placed near a wall makes the robot start then do nothing — help me project the goal to a
  reachable, obstacle-clear point and skip ones that are boxed in."
- "On restart, the mission does nothing; I suspect executor contention with `BasicNavigator` —
  review the threading and `waitUntilNav2Active` usage."
- "The GUI heading arrow points the wrong way after the map y-flip — compute the arrow tip in world
  metres and map it through the same transform."
- "Gazebo's 3D client crashes on WSL (OGRE2 on the d3d12 GL); add a `gz_gui:=false` server-only
  option so the LiDAR still renders."

### 7. Repository packaging and documentation
For the submission, the team had the assistant help assemble a clean, runnable copy of the
repository and draft its documentation, all reviewed and approved by the team:
- Select the files needed to build and run the project — the `algae_dt` package together with its
  map, config, launch files, and Gazebo worlds — so the copy builds and runs in both the simulation
  and the real-robot modes, with the code left functionally unchanged.
- Write clear, readable code comments.
- Draft the `README.md` (how to run the simulation on WSL2 + Docker and on the lab laptop, and an
  overview of the project structure and what each part does) and this AI-usage and authorship
  declaration.
