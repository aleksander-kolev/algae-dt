"""In-process integration tests for twin_mediator (PLAN T1.4/T2.3).

Runs only where rclpy is available (the algae-dt:dev container / lab laptop); skipped on a bare
Python host so the pure-lib suite still runs there. Drives the command bus + a synthetic /scan
through the real node and asserts the safety gate, fan-out, latched state, and pose mirror.
"""
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped   # noqa: E402
from nav_msgs.msg import Odometry                    # noqa: E402
from rclpy.executors import SingleThreadedExecutor   # noqa: E402
from rclpy.node import Node                           # noqa: E402
from rclpy.parameter import Parameter                 # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState, LaserScan  # noqa: E402
from std_msgs.msg import Bool, String                 # noqa: E402

from algae_dt.twin_mediator import TwinMediator       # noqa: E402


def _latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _make_scan(front_m: float, n: int = 360) -> LaserScan:
    s = LaserScan()
    s.angle_min = -math.pi
    s.angle_increment = 2.0 * math.pi / n
    s.range_min = 0.12
    s.range_max = 3.5
    s.ranges = [3.0] * n
    for i in (n // 2 - 1, n // 2, n // 2 + 1):   # dead-ahead beams (~0 rad)
        s.ranges[i] = front_m
    return s


class Harness(Node):
    """Stands in for Nav2/teleop + the robot: feeds the bus + /scan, listens on /cmd_vel + /dt/*."""

    def __init__(self) -> None:
        super().__init__('mediator_test_harness')
        self.cmd_vx = 0.0
        self.front_m = 3.0
        self.emit_cmd = True            # set False to simulate the command bus going silent
        self.emit_scan = True           # set False to simulate the LiDAR stream going stale
        self.last_cmd: TwistStamped | None = None
        self.last_safety: bool | None = None
        self.last_estop: bool | None = None
        self.last_mode: str | None = None
        self.last_real_pose = None
        self.last_sim_pose = None
        self.last_health: BatteryState | None = None

        self.p_cmd = self.create_publisher(TwistStamped, '/dt/cmd_vel_raw', 10)
        self.p_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.p_odom = self.create_publisher(Odometry, '/odom', 10)
        self.p_estop = self.create_publisher(Bool, '/dt/estop_cmd', _latched())

        self.create_subscription(TwistStamped, '/cmd_vel', lambda m: setattr(self, 'last_cmd', m), 10)
        self.create_subscription(Bool, '/dt/safety', lambda m: setattr(self, 'last_safety', m.data), _latched())
        self.create_subscription(Bool, '/dt/estop', lambda m: setattr(self, 'last_estop', m.data), _latched())
        self.create_subscription(String, '/dt/mode', lambda m: setattr(self, 'last_mode', m.data), _latched())
        self.create_subscription(PoseStamped, '/dt/real_pose',
                                 lambda m: setattr(self, 'last_real_pose', m), 10)
        self.create_subscription(PoseStamped, '/dt/sim_pose',
                                 lambda m: setattr(self, 'last_sim_pose', m), 10)
        self.create_subscription(BatteryState, '/dt/health', lambda m: setattr(self, 'last_health', m), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        if self.emit_cmd:
            self.p_cmd.publish(TwistStamped(twist=_twist(self.cmd_vx)))
        if self.emit_scan:
            self.p_scan.publish(_make_scan(self.front_m))
        od = Odometry()
        od.header.frame_id = 'odom'
        od.pose.pose.position.x = 1.0
        self.p_odom.publish(od)


def _twist(vx: float) -> Twist:
    t = Twist()
    t.linear.x = vx
    return t


@pytest.fixture()
def world():
    rclpy.init()
    med = TwinMediator()
    har = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(med)
    ex.add_node(har)
    yield har, ex
    ex.shutdown()
    med.destroy_node()
    har.destroy_node()
    rclpy.shutdown()


def _spin_until(ex, pred, secs=6.0) -> bool:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def test_clear_path_passes_command_through(world):
    har, ex = world
    har.front_m = 3.0
    har.cmd_vx = 0.2
    ok = _spin_until(ex, lambda: har.last_cmd is not None
                     and abs(har.last_cmd.twist.linear.x - 0.2) < 1e-6)
    assert ok, "forward command should pass through on a clear path"
    assert har.last_safety is False


def test_obstacle_within_25cm_zeros_forward(world):
    har, ex = world
    har.cmd_vx = 0.2
    har.front_m = 0.20            # obstacle 20 cm dead ahead
    ok = _spin_until(ex, lambda: har.last_safety is True
                     and har.last_cmd is not None
                     and har.last_cmd.twist.linear.x == 0.0)
    assert ok, "forward motion must be zeroed and /dt/safety must be True"


def test_estop_latches_and_stops(world):
    har, ex = world
    har.front_m = 3.0
    har.cmd_vx = 0.2
    har.p_estop.publish(Bool(data=True))
    ok = _spin_until(ex, lambda: har.last_estop is True
                     and har.last_cmd is not None
                     and har.last_cmd.twist.linear.x == 0.0)
    assert ok, "E-STOP must latch and full-stop the output"


def test_sim_pose_published_in_map_frame_via_tf(world):
    # Root-cause fix: the mediator lifts the odom pose into the MAP frame via map<-odom (AMCL),
    # so the GUI overlays the robot where RViz shows it. With map<-odom = (1,2,90deg) and the
    # harness odom pose (1,0,0), /dt/sim_pose must be the composed map pose (1,3,90deg).
    import math

    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    from algae_dt.lib.geometry import quaternion_from_yaw

    har, ex = world
    bc = rclpy.create_node('tf_map_odom_bc')
    stb = StaticTransformBroadcaster(bc)
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'odom'
    tf.transform.translation.x = 1.0
    tf.transform.translation.y = 2.0
    tf.transform.rotation.z, tf.transform.rotation.w = quaternion_from_yaw(math.pi / 2)
    stb.sendTransform(tf)
    ex.add_node(bc)
    try:
        ok = _spin_until(ex, lambda: har.last_sim_pose is not None
                         and har.last_sim_pose.header.frame_id == 'map'
                         and har.last_sim_pose.pose.position.y > 2.5, secs=8.0)
        assert ok, "/dt/sim_pose should be in the map frame, composed via map<-odom"
        assert abs(har.last_sim_pose.pose.position.x - 1.0) < 0.15
        assert abs(har.last_sim_pose.pose.position.y - 3.0) < 0.15
    finally:
        ex.remove_node(bc)
        bc.destroy_node()


def test_sim_only_sync_source_none_suppresses_shadow_pose():
    """sim_only_sync_source is wired: 'none' suppresses the commanded-shadow /dt/real_pose while the
    sim pose still mirrors from /odom (F14)."""
    rclpy.init()
    med = TwinMediator(parameter_overrides=[
        Parameter('sim_only_sync_source', Parameter.Type.STRING, 'none')])
    har = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(med)
    ex.add_node(har)
    try:
        har.cmd_vx = 0.2
        assert _spin_until(ex, lambda: har.last_sim_pose is not None), "sim pose still mirrors /odom"
        _spin_until(ex, lambda: False, secs=1.0)   # give the shadow integrator ample time to (not) fire
        assert har.last_real_pose is None, \
            "sim_only_sync_source=none must suppress the commanded-shadow /dt/real_pose"
    finally:
        ex.shutdown()
        med.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def test_latched_mode_and_sim_pose_and_health(world):
    har, ex = world
    ok = _spin_until(ex, lambda: har.last_mode == 'sim_only'
                     and har.last_sim_pose is not None
                     and har.last_health is not None and har.last_health.voltage > 0.0)
    assert ok, "mode latched, sim pose mirrored from /odom, synthetic battery published"
    assert har.last_estop is False    # no estop at startup


def test_command_output_is_stamped_with_populated_header(world):
    """The bus is TwistStamped on Jazzy precisely so /cmd_vel carries a populated header (stock teleop
    + Nav2 enable_stamped_cmd_vel rely on it). Assert the mediator's output is a TwistStamped with a
    base_link frame and a non-zero stamp — an unstamped forward passes the type check yet silently
    breaks stamp-consuming subscribers."""
    har, ex = world
    har.front_m = 3.0
    har.cmd_vx = 0.1
    ok = _spin_until(ex, lambda: har.last_cmd is not None and har.last_cmd.twist.linear.x > 0.0)
    assert ok
    assert isinstance(har.last_cmd, TwistStamped)
    assert har.last_cmd.header.frame_id == 'base_link'
    assert har.last_cmd.header.stamp.sec != 0 or har.last_cmd.header.stamp.nanosec != 0, \
        "mediator /cmd_vel must carry a populated header stamp (the TwistStamped contract)"


def test_battery_estop_latches_until_resume(world):
    """The battery auto-E-STOP is LATCHED: a sagging LiPo that bounces back above the critical
    threshold must NOT silently un-latch and re-enable motion. Only an operator RESUME
    (estop_cmd False) clears it — and a still-critical battery immediately re-trips."""
    from std_msgs.msg import Float64
    har, ex = world
    p_batt = har.create_publisher(BatteryState, '/battery_state', 10)

    def _v(v: float) -> BatteryState:
        b = BatteryState()
        b.voltage = float(v)
        return b

    # Trip: below critical (10.5).
    assert _spin_until(ex, lambda: har.last_estop is False)        # starts clear
    p_batt.publish(_v(10.0))
    assert _spin_until(ex, lambda: har.last_estop is True), "critical battery must auto-E-STOP"
    # Recover: voltage bounces back up. The latch must HOLD (this was the silent un-latch bug).
    p_batt.publish(_v(12.0))
    _spin_until(ex, lambda: False, secs=0.8)                        # give it time to (not) clear
    assert har.last_estop is True, "battery E-STOP must stay latched when the voltage recovers"
    # Operator RESUME clears it (battery is healthy now, so it stays cleared).
    har.p_estop.publish(Bool(data=False))
    assert _spin_until(ex, lambda: har.last_estop is False), "RESUME must clear the battery latch"


def test_battery_estop_retrips_after_resume_if_still_critical(world):
    """RESUME clears the latch, but a STILL-critical pack must immediately re-trip on the next sample
    — RESUME cannot bypass a genuinely dead battery. The latch docstring promised this; it was never
    exercised (the existing test only RESUMEs once the pack is healthy)."""
    har, ex = world
    p_batt = har.create_publisher(BatteryState, '/battery_state', 10)

    def _v(v: float) -> BatteryState:
        b = BatteryState()
        b.voltage = float(v)
        return b

    assert _spin_until(ex, lambda: har.last_estop is False)
    p_batt.publish(_v(10.0))
    assert _spin_until(ex, lambda: har.last_estop is True), "critical battery must auto-E-STOP"
    har.p_estop.publish(Bool(data=False))                # RESUME while STILL critical
    p_batt.publish(_v(10.0))                             # next sample is still below critical
    assert _spin_until(ex, lambda: har.last_estop is True), \
        "a still-critical battery must re-trip after RESUME (RESUME cannot bypass a dead pack)"


def test_invalid_battery_frame_does_not_false_trip(world):
    """A real OpenCR voltage=0.0 / NaN bringup or serial-glitch frame must NOT latch a phantom
    battery E-STOP on a healthy pack."""
    har, ex = world
    p_batt = har.create_publisher(BatteryState, '/battery_state', 10)
    assert _spin_until(ex, lambda: har.last_estop is False)
    b = BatteryState()
    b.voltage = 0.0
    p_batt.publish(b)
    _spin_until(ex, lambda: False, secs=0.6)
    assert har.last_estop is False, "a 0.0 V bringup frame must not auto-E-STOP a healthy pack"


def test_stale_scan_fail_safe_blocks_at_node_level(world):
    """The single most safety-critical behavior, wired end-to-end (not just the pure-lib gate): a
    clear path passes; when the LiDAR stream goes STALE (no /scan for max_data_age_s) the mediator
    must treat it as an obstacle — /dt/safety True and forward motion zeroed — and recover when scan
    resumes. (test_safety pins the pure gate; this pins _scan_t flowing into it inside the node.)"""
    har, ex = world
    har.front_m = 3.0
    har.cmd_vx = 0.2
    assert _spin_until(ex, lambda: har.last_safety is False
                       and har.last_cmd is not None and abs(har.last_cmd.twist.linear.x - 0.2) < 1e-6), \
        "clear, fresh scan -> forward passes, not blocked"
    har.emit_scan = False                                 # the LiDAR stream stops (stale)
    assert _spin_until(ex, lambda: har.last_safety is True
                       and har.last_cmd is not None and har.last_cmd.twist.linear.x == 0.0, secs=4.0), \
        "a stale scan must fail-safe: /dt/safety True and forward motion zeroed"
    har.emit_scan = True                                  # LiDAR recovers
    assert _spin_until(ex, lambda: har.last_safety is False, secs=4.0), \
        "a fresh clear scan must clear the fail-safe block"


def test_bus_silent_command_watchdog_stops(world):
    """Safety watchdog: if the command bus goes SILENT (teleop/Nav2 died), the mediator must stop the
    robot after max_cmd_age_s even on a clear path — a stale last command must never keep driving."""
    har, ex = world
    har.front_m = 3.0
    har.cmd_vx = 0.2
    assert _spin_until(ex, lambda: har.last_cmd is not None
                       and abs(har.last_cmd.twist.linear.x - 0.2) < 1e-6), "command passes through"
    har.emit_cmd = False                                  # the bus goes silent
    assert _spin_until(ex, lambda: har.last_cmd is not None
                       and har.last_cmd.twist.linear.x == 0.0, secs=4.0), \
        "a silent command bus must zero forward motion after max_cmd_age_s (watchdog)"


def test_sim_battery_override_forces_the_auto_estop_demo_beat(world):
    """The sim_only synthetic battery floors ABOVE critical by design, so the DEMO_SCRIPT
    battery->auto-E-STOP beat is triggered via /dt/battery_override_v; <=0 clears the override."""
    from std_msgs.msg import Float64
    har, ex = world
    p_over = har.create_publisher(Float64, '/dt/battery_override_v', 10)
    assert _spin_until(ex, lambda: har.last_health is not None and har.last_health.voltage > 11.0)
    p_over.publish(Float64(data=10.0))                              # force below critical (10.5)
    ok = _spin_until(ex, lambda: har.last_estop is True
                     and har.last_health is not None
                     and abs(har.last_health.voltage - 10.0) < 1e-6, secs=5.0)
    assert ok, "the override must drive /dt/health AND trip the latched auto-E-STOP"


def test_real_mode_starts_estop_held_until_first_healthy_battery():
    """REGRESSION (mediator restart latch): a real-mode mediator must start /dt/estop HELD (True) so
    a RESTART cannot silently un-latch a prior battery E-STOP; the hold releases only when the first
    valid healthy /battery_state arrives."""
    rclpy.init()
    med = TwinMediator(parameter_overrides=[Parameter('mode', Parameter.Type.STRING, 'real_only')])
    har = Harness()
    p_batt = har.create_publisher(BatteryState, '/battery_state', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(med)
    ex.add_node(har)
    try:
        assert _spin_until(ex, lambda: har.last_estop is True), \
            "a real-mode mediator must start E-STOP HELD (fail-safe across a restart)"
        b = BatteryState()
        b.voltage = 12.0
        p_batt.publish(b)
        assert _spin_until(ex, lambda: har.last_estop is False), \
            "the first valid healthy battery sample must release the startup hold"
    finally:
        ex.shutdown()
        med.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def test_both_mode_sim_pose_lifted_to_map_frame_via_tf():
    """In `both`, /dt/sim_pose must be lifted into the MAP frame using the same map<-odom as the real
    robot, so it is directly comparable to /dt/real_pose — not published raw in the sim's odom frame
    (which would make the sync discrepancy carry the whole map<-odom offset). With map<-odom =
    (1, 2, 90deg) and a /sim/odom pose (1, 0, 0), /dt/sim_pose must be the composed map pose
    (1, 3, 90deg)."""
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    from algae_dt.lib.geometry import quaternion_from_yaw

    rclpy.init()
    med = TwinMediator(parameter_overrides=[Parameter('mode', Parameter.Type.STRING, 'both')])
    har = rclpy.create_node('both_sim_harness')
    stb = StaticTransformBroadcaster(har)
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'odom'
    tf.transform.translation.x = 1.0
    tf.transform.translation.y = 2.0
    tf.transform.rotation.z, tf.transform.rotation.w = quaternion_from_yaw(math.pi / 2)
    stb.sendTransform(tf)
    p_sim_odom = har.create_publisher(Odometry, '/sim/odom', 10)
    got = {'p': None}
    har.create_subscription(PoseStamped, '/dt/sim_pose', lambda m: got.update(p=m), 10)

    def _emit() -> None:
        od = Odometry()
        od.header.frame_id = 'odom'
        od.pose.pose.position.x = 1.0
        od.pose.pose.orientation.w = 1.0
        p_sim_odom.publish(od)
    har.create_timer(1.0 / 30.0, _emit)

    ex = SingleThreadedExecutor()
    ex.add_node(med)
    ex.add_node(har)
    try:
        ok = _spin_until(ex, lambda: got['p'] is not None
                         and got['p'].header.frame_id == 'map'
                         and got['p'].pose.position.y > 2.5, secs=8.0)
        assert ok, "/dt/sim_pose in `both` must be composed into the map frame via map<-odom"
        assert abs(got['p'].pose.position.x - 1.0) < 0.15
        assert abs(got['p'].pose.position.y - 3.0) < 0.15
    finally:
        ex.shutdown()
        med.destroy_node()
        har.destroy_node()
        rclpy.shutdown()
