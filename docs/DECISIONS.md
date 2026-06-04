# DECISIONS.md — why the project is the way it is (preserve this context)

ADR-style log of the strategic decisions and the evidence behind them. Read this so you don't
re-litigate settled questions. Format: **Decision · Why · Status.**

## D1 — Restart as a fresh, course-faithful repo (`algae-dt`)
**Decision:** abandon the old `algae-twin` repo's from-scratch robot stack; rebuild minimal on the
course's recommended environment.
**Why (the forensics, so it's not lost):** the old repo's founding assumption was "the lab PC has no
`turtlebot3_*` and no Docker, so we must hand-build the Burger SDF/URDF, a `robot_state_publisher`
replacement, teleop, and Nav2 params." We verified **5 ways** on the bare native account
`team36@2IRR10-NB26` that turtlebot3 was absent THERE (`dpkg -l | grep ros-jazzy-turtlebot3`,
`ros2 pkg list | grep turtlebot3`, `ls ~/turtlebot3_ws/{src,install}/turtlebot3*` — all empty). BUT
the course material shows the **intended environment is WSL+Docker running the provided
`turtlebot3_ws` image (which SHIPS the turtlebot3 stack)** or the native lab laptop provisioned with
it. The turtlebot3 build log we'd once seen was the **robot's Raspberry Pi** (`turtlebot@ubuntu`), a
different machine — not the lab PC. So turtlebot3 IS available via the intended path; the old
"absent" finding was true only for the bare native account, which was never the target.
**Status:** settled — and the open check is now CONFIRMED (lab session, 2026-06): the lab PC has
**NO Docker**; the turtlebot3 stack is present natively, **built from source in
`~/turtlebot3_ws/src`** (`turtlebot3`, `turtlebot3_msgs`, `DynamixelSDK`, `turtlebot3_simulations`).
So the demo path on the lab PC is **native** (`scripts/lab_run.sh` auto-detects; `--native` forces),
and the stock-stack decision holds exactly as made — the stack comes from the workspace sources
instead of an image. Corollary learned the hard way: that from-source workspace is the machine's
single point of failure — copied from another PC it breaks (absolute CMake paths + foreign-owned
files, unfixable without sudo); `scripts/lab_fix_workspace.sh` rebuilds it from the fail-safe zip
in `~/Downloads` (no sudo).

## D2 — Reuse the stock turtlebot3 packages; our code is a thin DT layer only
**Decision:** use `turtlebot3_gazebo` / `turtlebot3_navigation2` / `turtlebot3_bringup` /
`turtlebot3_teleop` / `turtlebot3_cartographer` (+ `nav2_simple_commander`). Our custom `algae_dt`
package = mediator + sync_supervisor + mission_runner + GUI + pure libs only.
**Why:** the course defines these as the "default packages" and *mandates* a custom package on top of
them. This deletes ~60–70% of the old code (and its hardest bugs). Evidence it's the right base: a
peer group running stock turtlebot3 in Docker got a sim working trivially — but had **no bidirectional
twin**, which is exactly the graded differentiator. So: stock stack for the robot, our layer for the DT.
**Status:** settled. Do NOT re-add custom SDF/URDF/`robot_state_publisher`/teleop/Nav2 params.

## D3 — Command bus is `TwistStamped` (NOT plain Twist)
**Decision:** `/dt/cmd_vel_raw` carries `geometry_msgs/TwistStamped`; set
`enable_stamped_cmd_vel: true` on Nav2 via a `params_file`.
**Why:** verified against Nav2 + TurtleBot3 docs (Context7). On Jazzy the real `turtlebot3_node`
(`enable_stamped_cmd_vel: true` in burger.yaml) and `turtlebot3_teleop` both use `TwistStamped`; Nav2
defaults to plain `Twist` on Jazzy. A round-1 "simplification" to a plain-Twist bus was **reverted**
because it breaks stock teleop. This matches the course Mini-Project-3 bus.
**Status:** settled. Do NOT re-simplify to plain Twist. `/sim/cmd_vel` type is verified at runtime.

## D4 — One ament_python package
**Decision:** ship a single `algae_dt` package (nodes + launch + worlds + config + maps + tests).
**Why:** mirrors the course `my_tb3_world` pattern and avoids the dependency-cycle trap that made the
old two-package layout fail `colcon build`.
**Status:** settled.

## D5 — Develop in `sim_only` at home; the lab is for TESTING only
**Decision:** every feature is built + TDD-tested in `sim_only` before any lab session; the lab
validates hardware (`real_only`/`both`).
**Why:** course rule ("lab = testing, development outside"); robot time is scarce and shared.
**Status:** settled. `sim_only` is also the guaranteed demo fallback.

## D6 — Option A (full lab participation)
**Decision:** take Option A.
**Why:** the official rubric caps Option B at Bidirectional≤3, Sync≤2, Environmental≤2 = **7/12**.
12/12 requires Option A.
**Status:** settled.

## D7 — Hardware-independent demonstrability
**Decision:** make all three pillars demonstrable without the robot — sim_only sync via a
COMMANDED-shadow pose; a ported `fake_robot.py` publishing bare `/scan /odom /cmd_vel /battery_state`
to exercise the bidirectional arrows; a pre-recorded `sim_only` baseline video.
**Why:** robot availability is the top risk; this de-risks the reviews and the video.
**Status:** settled.

## D8 — `use_sim_time: true` ONLY in `sim_only`
**Decision:** sim time only in `sim_only`; `real_only`/`both` run on wall time (real robot leads).
**Why:** the real robot leads on wall time; sim-stamped topics are re-stamped and never reach Nav2/TF.
**Status:** settled.
