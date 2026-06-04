"""mission_runner — navigate to each bloom + spray (Rubric ③ task). PLAN T2.2.

Drives the autonomy: pick the nearest untreated bloom (lib.blooms), send a Nav2 goal via
nav2_simple_commander.BasicNavigator, and on a *successful* arrival spin in place for
spray_revolutions full turns ("spraying") on the pre-safety bus /dt/cmd_vel_raw so the spin still
passes the mediator's gate.
Publishes /dt/markers (state-coloured) + /dt/mission_state. Honest accounting (RULES §B-7/§B-8):
spray + mark-treated ONLY on Nav2 SUCCEEDED (+ a generous sim-only gross-arrival check); a nav
failure/timeout greys the bloom (skipped); a spray or nav cut short by Stop/E-STOP leaves the bloom
PENDING. The mission loop runs on a worker thread; the navigator is injectable for testing.

Launch note (BEST_APPROACHES): NEVER set name= on this node's launch Node (process-wide remap trap),
and it publishes no cmd_vel of its own beyond the spray spin (Nav2's controller cmd_vel is remapped
to /dt/cmd_vel_raw in the launch, not here).
"""
from __future__ import annotations

import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from algae_dt.lib import blooms as B
from algae_dt.lib import geometry, occupancy, pgm

# Marker colours per bloom state (RGBA).
_COLORS = {
    B.PENDING: (0.95, 0.85, 0.15, 0.9),   # yellow
    B.ACTIVE:  (0.20, 0.55, 1.00, 0.9),   # blue
    B.TREATED: (0.15, 0.80, 0.25, 0.9),   # green
    B.SKIPPED: (0.55, 0.55, 0.55, 0.7),   # grey
}


def _latched(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MissionRunner(Node):
    def __init__(self, navigator=None, **kwargs) -> None:
        super().__init__('mission_runner', **kwargs)
        gp = self._declare
        self.mode = gp('mode', 'sim_only')
        self.spray_revolutions = gp('spray_revolutions', 3.0)   # FULL in-place spins per bloom
        self.spray_omega = gp('spray_omega_radps', 1.0)
        self.spray_time_margin = gp('spray_time_margin', 2.0)   # backstop = margin x nominal spin time
        self.center_tol_m = gp('center_tol_m', 0.50)
        self.nav_goal_timeout_s = gp('nav_goal_timeout_s', 60.0)
        self.cmd_rate_hz = gp('cmd_rate_hz', 20.0)
        self.bloom_radius = gp('bloom_radius_m', 0.15)
        self.goal_clearance_m = gp('goal_clearance_m', 0.30)   # min obstacle clearance for a nav goal
        self._map_info = geometry.MapInfo(
            resolution=gp('map_resolution', 0.05),
            origin_x=gp('map_origin_x', -2.051),
            origin_y=gp('map_origin_y', -4.194),
            width_px=gp('map_width_px', 86),
            height_px=gp('map_height_px', 110),
        )
        self._grid = self._load_grid()        # static-map occupancy for goal projection (None=off)

        self._navigator = navigator           # injected in tests; real BasicNavigator otherwise
        self._lock = threading.Lock()
        self._field = B.BloomField()
        self._robot_xy = (0.0, 0.0)
        self._robot_yaw = 0.0                  # odom yaw (rad), for the closed-loop spray spin count
        self._estop = False
        self._running = False
        self._worker: threading.Thread | None = None

        self.pub_cmd = self.create_publisher(TwistStamped, '/dt/cmd_vel_raw', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/dt/markers', _latched())
        self.pub_state = self.create_publisher(String, '/dt/mission_state', _latched())

        self.create_subscription(MarkerArray, '/dt/blooms', self._on_blooms, _latched())
        self.create_subscription(String, '/dt/mission_cmd', self._on_cmd, 10)
        self.create_subscription(Bool, '/dt/estop', self._on_estop, _latched())
        self.create_subscription(Odometry, '/dt/odom_active', self._on_odom, 10)

        self._set_state('idle')
        self.get_logger().info(
            f"mission_runner up: mode={self.mode} spray={self.spray_revolutions} spins")

    # ------------------------------------------------------------------ utils
    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _set_state(self, s: str) -> None:
        self.pub_state.publish(String(data=s))

    def _load_grid(self):
        """Load the static map (maps/map.pgm) into an occupancy grid for goal projection. A genuinely
        absent package/map (e.g. a bare unit-test env) disables projection (goals = raw bloom centre)
        so the node still runs. A PRESENT-but-unparseable map, or a grid whose dimensions disagree
        with the map params, is a real fault surfaced LOUDLY — not silently degraded to raw near-wall
        goals (which is exactly the 'robot starts then does nothing near a wall' failure)."""
        try:
            path = os.path.join(get_package_share_directory('algae_dt'), 'maps', 'map.pgm')
            with open(path, 'rb') as f:
                grid = occupancy.from_pgm(pgm.parse(f.read()))
        except (PackageNotFoundError, FileNotFoundError) as exc:
            self.get_logger().warn(f"static map not found; goal projection OFF: {exc!r}")
            return None
        except Exception as exc:                          # a present map that won't parse is a fault
            self.get_logger().error(f"static map FAILED to load; goal projection OFF: {exc!r}")
            return None
        if grid.width != self._map_info.width_px or grid.height != self._map_info.height_px:
            self.get_logger().error(
                f"map/params mismatch: grid {grid.width}x{grid.height} vs params "
                f"{self._map_info.width_px}x{self._map_info.height_px}; goal projection OFF")
            return None
        self.get_logger().info(
            f"goal projection ON: static map {grid.width}x{grid.height}, "
            f"clearance {self.goal_clearance_m:.2f} m")
        return grid

    # -------------------------------------------------------------- callbacks
    def _on_blooms(self, msg: MarkerArray) -> None:
        """Rebuild the field from the operator's markers, PRESERVING the state of known ids."""
        with self._lock:
            old = {b.id: b for b in self._field.blooms}
            f = B.BloomField()
            for m in msg.markers:
                state = old[m.id].state if m.id in old else B.PENDING
                f = B.add(f, B.Bloom(m.id, m.pose.position.x, m.pose.position.y,
                                     self.bloom_radius, state))
            self._field = f
        self._publish_markers()

    def _on_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self._start()
        elif cmd == 'stop':
            self._running = False
            self._set_state('stopped')
        elif cmd == 'clear':
            self._running = False
            with self._lock:
                self._field = B.clear(self._field)
            self._publish_markers()
            self._set_state('idle')

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)
        if self._estop:
            self._running = False

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._robot_xy = (p.position.x, p.position.y)
        self._robot_yaw = geometry.yaw_from_quaternion(p.orientation.z, p.orientation.w)

    # ----------------------------------------------------------- mission loop
    def _start(self) -> None:
        # Check-then-act under the lock so a concurrent E-STOP/Clear can't race the launch (F2).
        with self._lock:
            if self._running or (self._worker and self._worker.is_alive()):
                return
            if self._estop:
                self.get_logger().warn("start ignored: E-STOP latched")
                return
            self._field = B.retry_skipped(self._field)   # a fresh Start retries previously-failed blooms
            self._running = True
            self._worker = threading.Thread(target=self._run_mission, daemon=True)
            self._worker.start()
        # NOTE: the worker publishes markers as soon as it picks a bloom, which reflects the
        # skipped->pending reset; an explicit publish here would race the worker's own publishes.

    def _run_mission(self) -> None:
        # NOTE: we deliberately do NOT call BasicNavigator.waitUntilNav2Active() here. It was only
        # ever reached on the 2nd+ mission (the 1st lazily creates the navigator and skips it), and on
        # the reused navigator it can block in _waitForInitialPose, republish an all-zero /initialpose
        # (wrecking AMCL), or throw under executor contention — the root cause of "restart does
        # nothing". Nav2 is autostarted by the launch and AMCL is seeded (set_initial_pose in sim_only
        # / operator 2D-Pose-Estimate on the robot); goToPose() waits for the action server itself.
        try:
            while self._running and not self._estop:
                with self._lock:
                    field, (rx, ry) = self._field, self._robot_xy
                target = B.nearest_untreated(field, rx, ry)
                if target is None:
                    self._set_state('complete')
                    break
                with self._lock:
                    if B.by_id(self._field, target.id) is None:
                        continue                           # field changed under us -> re-pick
                    self._field = B.set_active(self._field, target.id)
                self._publish_markers()
                self._set_state(f'navigating:{target.id}')

                outcome = self._navigate(target)
                if outcome == 'stopped':
                    self._apply_outcome(target.id, B.set_pending)   # operator stop -> honest: pending
                    self._publish_markers()
                    break
                if outcome == 'arrived':
                    self._set_state(f'spraying:{target.id}')
                    completed = self._spray()
                    if completed:
                        self._apply_outcome(target.id, B.mark_treated)
                    elif self._estop or not self._running:
                        self._apply_outcome(target.id, B.set_pending)   # operator Stop/E-STOP -> resumable
                    else:
                        self._apply_outcome(target.id, B.mark_skipped)  # spray stalled (backstop) -> skip
                else:  # 'failed'
                    self._apply_outcome(target.id, B.mark_skipped)
                self._publish_markers()
        except Exception as exc:                           # never let the worker die silently (F1)
            self.get_logger().error(f"mission worker aborted: {exc!r}")
            self._set_state('idle')
        finally:
            with self._lock:                               # write under the lock _start checks (F2)
                self._running = False

    def _apply_outcome(self, bloom_id: int, transition) -> None:
        """Apply a bloom state transition, tolerating a bloom removed mid-mission (operator
        Clear / re-place drops the id between target selection and the write). No-op if gone (F1)."""
        with self._lock:
            if B.by_id(self._field, bloom_id) is None:
                return
            self._field = transition(self._field, bloom_id)

    def _navigate(self, target: B.Bloom) -> str:
        """Return 'arrived' | 'failed' | 'stopped'. The goal is projected out of obstacles first; an
        unreachable bloom OR a goal Nav2 rejects fails FAST (no recovery-behaviour churn / 60 s
        timeout that looks like the robot 'doing nothing' near a wall)."""
        goal = self._goal_xy(target)
        if goal is None:
            self.get_logger().warn(
                f"bloom {target.id}: no obstacle-clear goal within {self.center_tol_m:.2f} m "
                f"(blocked-in near a wall) -> skipped")
            return 'failed'
        nav = self._ensure_navigator()
        if nav.goToPose(self._goal_pose(*goal)) is False:    # goal rejected (e.g. in collision)
            self.get_logger().warn(f"bloom {target.id}: Nav2 rejected the goal -> skipped")
            return 'failed'
        deadline = time.monotonic() + self.nav_goal_timeout_s   # wall-clock watchdog (not sim time)
        while not nav.isTaskComplete():
            if self._estop or not self._running:
                nav.cancelTask()
                return 'stopped'
            if time.monotonic() > deadline:
                nav.cancelTask()
                self.get_logger().warn(f"bloom {target.id}: nav timeout -> skipped")
                return 'failed'
            time.sleep(0.1)
        from nav2_simple_commander.robot_navigator import TaskResult
        if nav.getResult() == TaskResult.SUCCEEDED and self._gross_arrived(target):
            return 'arrived'
        return 'failed'

    def _ensure_navigator(self):
        if self._navigator is None:                          # pragma: no cover - real Nav2 only
            from nav2_simple_commander.robot_navigator import BasicNavigator
            self._navigator = BasicNavigator(node_name='mission_runner_nav', namespace='')
        return self._navigator

    def _gross_arrived(self, target: B.Bloom) -> bool:
        """Generous sim-only sanity bound on arrival; Nav2's xy_goal_tolerance is authoritative.
        In real_only/both, /dt/odom_active is odom-frame and not map-comparable, so trust Nav2."""
        if self.mode != 'sim_only':
            return True
        rx, ry = self._robot_xy
        return geometry.euclidean(rx, ry, target.x, target.y) <= self.center_tol_m

    def _spray(self) -> bool:
        """Chemical spraying = spin in place until the robot has ACTUALLY turned spray_revolutions full
        360 deg, measured CLOSED-LOOP from odometry yaw (/dt/odom_active, accumulated wrap-safe). This
        guarantees the full spin COUNT regardless of real-time-factor or sluggish tracking — a fixed
        open-loop duration under-rotates (the robot turns fewer than N revs). A wall-clock backstop of
        spray_time_margin x the nominal spin time prevents an endless spin if odom stalls.

        Returns True ONLY when the full count was actually reached. Returns False if cut short — by
        Stop/E-STOP (operator) OR by the backstop firing before the count completed (odom stalled);
        either way the caller keeps the bloom honestly un-treated (RULES §B-8)."""
        omega = self.spray_omega if self.spray_omega > 0.0 else 1.0
        target = self.spray_revolutions * 2.0 * math.pi
        # Wall-clock backstop, matching the time.sleep() below. Must NOT use the node clock: under
        # use_sim_time that is sim time, which at a real-time-factor < 1 expires in fewer wall
        # seconds and would cut the spin short before the full count (the bug this method exists for).
        backstop = time.monotonic() + (target / omega) * self.spray_time_margin
        period = 1.0 / self.cmd_rate_hz
        turned = 0.0
        last = self._robot_yaw
        while turned < target and time.monotonic() < backstop:
            if self._estop or not self._running:
                self._publish_spin(0.0)
                return False
            self._publish_spin(omega)
            time.sleep(period)
            y = self._robot_yaw
            turned += abs(geometry.angle_diff(y, last))   # wrap-safe |delta yaw| since last sample
            last = y
        self._publish_spin(0.0)
        if turned < target:    # backstop fired before the count completed -> odom stalled, NOT sprayed
            self.get_logger().warn(
                f"spray backstop hit at {turned / (2.0 * math.pi):.2f}/{self.spray_revolutions} "
                f"revs (odom stalled?) -> bloom NOT marked treated")
            return False
        return True

    def _publish_spin(self, omega: float) -> None:
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist.angular.z = omega
        self.pub_cmd.publish(ts)

    def _goal_xy(self, target: B.Bloom):
        """The (x,y) Nav2 goal for a bloom: its centre projected to the nearest point with at least
        `goal_clearance_m` to every obstacle (within `center_tol_m`), using the static map. Returns
        None when no such point exists (truly boxed-in -> skip). Projection off (no map) -> centre."""
        if self._grid is None:
            return (target.x, target.y)
        return occupancy.reachable_goal(self._grid, self._map_info, target.x, target.y,
                                        self.goal_clearance_m, self.center_tol_m)

    def _goal_pose(self, x: float, y: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        return ps

    # ------------------------------------------------------------- markers
    def _publish_markers(self) -> None:
        with self._lock:
            blooms = self._field.blooms
        arr = MarkerArray()
        for b in blooms:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'blooms'
            m.id = b.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = b.x
            m.pose.position.y = b.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = max(0.05, 2.0 * b.radius)
            m.scale.z = 0.05
            r, g, bl, a = _COLORS.get(b.state, _COLORS[B.PENDING])
            m.color = ColorRGBA(r=r, g=g, b=bl, a=a)
            arr.markers.append(m)
        self.pub_markers.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRunner()
    # Spin on a DEDICATED executor, not rclpy's process-global one. The mission worker drives the
    # BasicNavigator via rclpy.spin_until_future_complete(), which spins the GLOBAL executor on the
    # worker thread; keeping this node off the global executor prevents two threads from driving one
    # SingleThreadedExecutor concurrently (a source of dropped callbacks / stalls on mission restart).
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
