"""In-process integration tests for sync_supervisor (PLAN T3.2, Rubric ②).

rclpy-only (skipped on a bare host). Feeds /dt/real_pose + /dt/sim_pose (+ /cmd_vel, /dt/odom_active)
and asserts the measured sync error, the /dt/sync_ok flip + /dt/alerts on out-of-tolerance, the
command->motion latency, and that the CSV evidence file is written with the documented columns.
"""
import glob
import math
import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3   # noqa: E402
from nav_msgs.msg import Odometry                                          # noqa: E402
from rclpy.executors import SingleThreadedExecutor                         # noqa: E402
from rclpy.node import Node                                                 # noqa: E402
from rclpy.parameter import Parameter                                       # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,                        # noqa: E402
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Float64, String                              # noqa: E402

from algae_dt.lib import geometry, metrics                                  # noqa: E402
from algae_dt.sync_supervisor import SyncSupervisor                         # noqa: E402


def _pose(x, y, yaw=0.0) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'odom'
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z, ps.pose.orientation.w = geometry.quaternion_from_yaw(yaw)
    return ps


class Harness(Node):
    def __init__(self) -> None:
        super().__init__('sync_test_harness')
        self.real = (0.0, 0.0, 0.0)
        self.sim = (0.0, 0.0, 0.0)
        self.emit_real = True
        self.emit_sim = True
        self.stamp_cmd = True
        self.cmd_vx = 0.0
        self.odom_vx = 0.0
        self.last_err: Vector3 | None = None
        self.last_ok: bool | None = None
        self.last_alert: str | None = None
        self.last_latency: float | None = None

        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_sim = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.p_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.create_subscription(Vector3, '/dt/sync_error', lambda m: setattr(self, 'last_err', m), 10)
        self.create_subscription(Bool, '/dt/sync_ok', lambda m: setattr(self, 'last_ok', m.data), 10)
        self.create_subscription(String, '/dt/alerts', lambda m: setattr(self, 'last_alert', m.data), 10)
        self.create_subscription(Float64, '/dt/latency_ms', lambda m: setattr(self, 'last_latency', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        if self.emit_real:
            self.p_real.publish(_pose(*self.real))
        if self.emit_sim:
            self.p_sim.publish(_pose(*self.sim))
        now = self.get_clock().now().to_msg()
        c = TwistStamped()
        if self.stamp_cmd:
            c.header.stamp = now
        c.twist = Twist()
        c.twist.linear.x = self.cmd_vx
        self.p_cmd.publish(c)
        od = Odometry()
        od.header.stamp = now
        od.twist.twist.linear.x = self.odom_vx
        self.p_odom.publish(od)


@pytest.fixture()
def world(tmp_path):
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'sim_only'),
        Parameter('sync_log_dir', Parameter.Type.STRING, str(tmp_path)),
    ])
    har = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    yield har, ex, tmp_path
    ex.shutdown()
    sup.destroy_node()
    har.destroy_node()
    rclpy.shutdown()


def _spin_until(ex, pred, secs=6.0) -> bool:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def test_in_tolerance_sets_sync_ok_true(world):
    har, ex, _ = world
    har.real = (0.0, 0.0, 0.0)
    har.sim = (0.02, 0.0, 0.0)        # 2 cm < 15 cm tolerance
    ok = _spin_until(ex, lambda: har.last_ok is True and har.last_err is not None)
    assert ok and abs(har.last_err.x - 0.02) < 1e-6


def test_out_of_tolerance_flips_and_alerts(world):
    har, ex, _ = world
    har.sim = (0.5, 0.0, 0.0)          # 50 cm >> 15 cm tolerance
    ok = _spin_until(ex, lambda: har.last_ok is False and har.last_alert is not None)
    assert ok and 'OUT-OF-TOLERANCE' in har.last_alert


def test_command_to_motion_latency_measured(world):
    har, ex, _ = world
    har.cmd_vx = 0.2
    har.odom_vx = 0.2                  # robot moves shortly after the command
    ok = _spin_until(ex, lambda: har.last_latency is not None)
    assert ok and har.last_latency >= 0.0


def test_csv_evidence_written_with_columns(world):
    har, ex, tmp_path = world
    har.sim = (0.03, 0.0, 0.0)
    _spin_until(ex, lambda: har.last_err is not None, secs=3.0)
    # let a couple of 5 Hz ticks append rows
    _spin_until(ex, lambda: False, secs=0.6)
    files = glob.glob(os.path.join(str(tmp_path), 'sync_metrics_*.csv'))
    assert files, "a sync_metrics CSV should be created"
    with open(files[0], encoding='utf-8') as f:
        lines = f.read().strip().splitlines()
    assert lines[0] == metrics.csv_header()
    assert len(lines) >= 2          # header + at least one data row


def test_missing_world_fails_loud(world):
    """In real_only/both a missing/stale sim pose stream is itself a desync: sync_supervisor must
    flip /dt/sync_ok False and raise /dt/alerts, not silently skip the tick (F5)."""
    har, ex, _ = world
    har.emit_sim = False              # the sim world never reports its pose
    ok = _spin_until(ex, lambda: har.last_ok is False
                     and 'stale/absent' in (har.last_alert or ''), secs=4.0)
    assert ok, "a missing/stale pose stream must flip sync_ok False and alert, not go silent (F5)"


def test_unstamped_command_drops_latency_sample(world):
    """A /cmd_vel with a zero header stamp must NOT be turned into a fabricated ~0 ms latency by
    substituting wall-now; the sample is dropped instead (F6)."""
    har, ex, _ = world
    har.stamp_cmd = False             # /cmd_vel arrives with a zero (invalid) header stamp
    har.cmd_vx = 0.2
    har.odom_vx = 0.2
    fired = _spin_until(ex, lambda: har.last_latency is not None, secs=2.0)
    assert not fired, "an unstamped command must not fabricate a latency sample (F6)"


def _tl() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _stamped_odom(vx: float, t: float) -> Odometry:
    od = Odometry()
    od.header.stamp.sec = int(t)
    od.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    od.twist.twist.linear.x = float(vx)
    return od


class BothHarness(Node):
    """Drives the `both`-mode stop-skew path with deterministic odom stamps."""
    def __init__(self) -> None:
        super().__init__('sync_both_harness')
        self.last_alert: str | None = None
        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_sim = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.p_aodom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.p_sodom = self.create_publisher(Odometry, '/sim/odom', 10)
        self.p_safety = self.create_publisher(Bool, '/dt/safety', _tl())
        self.create_subscription(String, '/dt/alerts', lambda m: setattr(self, 'last_alert', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        self.p_real.publish(_pose(0.0, 0.0, 0.0))   # both worlds agree -> only STOP-SKEW can alert
        self.p_sim.publish(_pose(0.0, 0.0, 0.0))


def _pump(ex, n: int = 25) -> None:
    for _ in range(n):
        ex.spin_once(timeout_sec=0.02)


def test_both_mode_stop_skew_alerts_when_over_budget(tmp_path):
    """The documented stop_skew_ms tolerance is loaded, compared, and alerted on a safety event in
    `both` mode (F3); and _pending_stop_skew resets every tick even with the CSV disabled (F4)."""
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'both'),
        Parameter('stop_skew_ms', Parameter.Type.DOUBLE, 50.0),
        Parameter('sync_log_enable', Parameter.Type.BOOL, False),   # also proves F4 reset w/o CSV
        Parameter('sync_log_dir', Parameter.Type.STRING, str(tmp_path)),
    ])
    har = BothHarness()
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    try:
        _pump(ex, 15)                                   # poses start flowing -> stream_ok
        har.p_aodom.publish(_stamped_odom(0.3, 100.0))  # real moving
        har.p_sodom.publish(_stamped_odom(0.3, 100.0))  # sim moving
        _pump(ex, 15)
        har.p_aodom.publish(_stamped_odom(0.0, 100.1))  # real STOP at t=100.1
        har.p_sodom.publish(_stamped_odom(0.0, 100.0))  # sim STOP at t=100.0 -> skew 100 ms > 50
        _pump(ex, 15)
        har.p_safety.publish(Bool(data=True))           # safety edge captures the skew
        got = _spin_until(ex, lambda: 'STOP-SKEW' in (har.last_alert or ''), secs=4.0)
        assert got, "stop-skew over the documented budget must raise /dt/alerts (F3)"
        _pump(ex, 10)
        assert sup._pending_stop_skew is None, \
            "pending stop-skew must reset each tick even with the CSV off (F4)"
    finally:
        ex.shutdown()
        sup.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def _stamped_cmd(vx: float, t: float) -> TwistStamped:
    c = TwistStamped()
    c.header.stamp.sec = int(t)
    c.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    c.twist.linear.x = float(vx)
    return c


class LatencyHarness(Node):
    """Publishes agreeing real/sim poses (so the streams are fresh and in-tolerance) while the test
    drives /cmd_vel and /dt/odom_active stamps by hand to force a specific command->motion latency."""
    def __init__(self) -> None:
        super().__init__('sync_latency_harness')
        self.last_alert: str | None = None
        self.last_latency: float | None = None
        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_sim = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.p_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.create_subscription(String, '/dt/alerts', lambda m: setattr(self, 'last_alert', m.data), 10)
        self.create_subscription(Float64, '/dt/latency_ms', lambda m: setattr(self, 'last_latency', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        self.p_real.publish(_pose(0.0, 0.0, 0.0))    # both worlds agree -> only LATENCY can alert
        self.p_sim.publish(_pose(0.0, 0.0, 0.0))


def test_latency_over_budget_alerts(tmp_path):
    """A command->motion latency beyond latency_budget_ms must raise a LATENCY /dt/alerts (Rubric ②
    tolerance), not merely be measured. Deterministic stamps: command at t=100.0, motion at t=100.5
    -> 500 ms > the 250 ms budget."""
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'sim_only'),
        Parameter('latency_budget_ms', Parameter.Type.DOUBLE, 250.0),
        Parameter('sync_log_enable', Parameter.Type.BOOL, False),
        Parameter('sync_log_dir', Parameter.Type.STRING, str(tmp_path)),
    ])
    har = LatencyHarness()
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    try:
        _pump(ex, 15)                                   # poses start flowing -> stream_ok
        har.p_cmd.publish(_stamped_cmd(0.3, 100.0))     # command motion onset at t=100.0
        _pump(ex, 5)
        har.p_odom.publish(_stamped_odom(0.3, 100.5))   # robot actually moves at t=100.5 -> 500 ms
        got = _spin_until(ex, lambda: 'LATENCY' in (har.last_alert or ''), secs=4.0)
        assert got, "a latency beyond the documented budget must raise /dt/alerts"
        assert har.last_latency is not None and har.last_latency >= 499.0, \
            f"measured latency should be ~500 ms, got {har.last_latency}"
    finally:
        ex.shutdown()
        sup.destroy_node()
        har.destroy_node()
        rclpy.shutdown()
