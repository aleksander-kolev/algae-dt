"""mission_runner — navigate to each bloom + spray (Rubric ③ task). PLAN T2.2.

Drives the autonomy: pick the nearest untreated bloom (lib.blooms), send a Nav2 goal via
nav2_simple_commander.BasicNavigator, and on a *successful* arrival spin in place for spray_seconds
("spraying") on the pre-safety bus /dt/cmd_vel_raw so the spin still passes the mediator's gate.
Publishes /dt/markers (state-coloured) + /dt/mission_state. Honest accounting (RULES §B-7/§B-8):
spray + mark-treated ONLY on Nav2 SUCCEEDED (+ a generous sim-only gross-arrival check); a nav
failure/timeout greys the bloom (skipped); a spray or nav cut short by Stop/E-STOP leaves the bloom
PENDING. The mission loop runs on a worker thread; the navigator is injectable for testing.

Launch note (BEST_APPROACHES): NEVER set name= on this node's launch Node (process-wide remap trap),
and it publishes no cmd_vel of its own beyond the spray spin (Nav2's controller cmd_vel is remapped
to /dt/cmd_vel_raw in the launch, not here).
"""
from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from algae_dt.lib import blooms as B
from algae_dt.lib import geometry

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
        self.spray_seconds = gp('spray_seconds', 5.0)
        self.spray_omega = gp('spray_omega_radps', 1.0)
        self.center_tol_m = gp('center_tol_m', 0.50)
        self.nav_goal_timeout_s = gp('nav_goal_timeout_s', 60.0)
        self.cmd_rate_hz = gp('cmd_rate_hz', 20.0)
        self.bloom_radius = gp('bloom_radius_m', 0.15)

        self._navigator = navigator           # injected in tests; real BasicNavigator otherwise
        self._lock = threading.Lock()
        self._field = B.BloomField()
        self._robot_xy = (0.0, 0.0)
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
        self.get_logger().info(f"mission_runner up: mode={self.mode} spray={self.spray_seconds}s")

    # ------------------------------------------------------------------ utils
    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _set_state(self, s: str) -> None:
        self.pub_state.publish(String(data=s))

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
        self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ----------------------------------------------------------- mission loop
    def _start(self) -> None:
        if self._running or (self._worker and self._worker.is_alive()):
            return
        if self._estop:
            self.get_logger().warn("start ignored: E-STOP latched")
            return
        self._running = True
        self._worker = threading.Thread(target=self._run_mission, daemon=True)
        self._worker.start()

    def _run_mission(self) -> None:
        if self._navigator is not None and hasattr(self._navigator, 'waitUntilNav2Active'):
            try:
                self._navigator.waitUntilNav2Active()
            except Exception as exc:                       # pragma: no cover - real Nav2 only
                self.get_logger().error(f"Nav2 not active: {exc}")
                self._running = False
        while self._running and not self._estop:
            with self._lock:
                field, (rx, ry) = self._field, self._robot_xy
            target = B.nearest_untreated(field, rx, ry)
            if target is None:
                self._set_state('complete')
                break
            with self._lock:
                self._field = B.set_active(self._field, target.id)
            self._publish_markers()
            self._set_state(f'navigating:{target.id}')

            outcome = self._navigate(target)
            if outcome == 'stopped':
                with self._lock:                              # operator stop -> honest: stay pending
                    self._field = B.set_pending(self._field, target.id)
                self._publish_markers()
                break
            if outcome == 'arrived':
                self._set_state(f'spraying:{target.id}')
                full = self._spray()
                with self._lock:
                    self._field = (B.mark_treated(self._field, target.id) if full
                                   else B.set_pending(self._field, target.id))
            else:  # 'failed'
                with self._lock:
                    self._field = B.mark_skipped(self._field, target.id)
            self._publish_markers()
        self._running = False

    def _navigate(self, target: B.Bloom) -> str:
        """Return 'arrived' | 'failed' | 'stopped'."""
        nav = self._ensure_navigator()
        nav.goToPose(self._goal_pose(target))
        deadline = time.monotonic() + self.nav_goal_timeout_s
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
        """Spin in place for spray_seconds; return False if cut short by Stop/E-STOP."""
        end = time.monotonic() + self.spray_seconds
        period = 1.0 / self.cmd_rate_hz
        while time.monotonic() < end:
            if self._estop or not self._running:
                self._publish_spin(0.0)
                return False
            self._publish_spin(self.spray_omega)
            time.sleep(period)
        self._publish_spin(0.0)
        return True

    def _publish_spin(self, omega: float) -> None:
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist.angular.z = omega
        self.pub_cmd.publish(ts)

    def _goal_pose(self, target: B.Bloom) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = target.x
        ps.pose.position.y = target.y
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
