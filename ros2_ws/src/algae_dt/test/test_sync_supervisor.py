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
        self.alert_count = 0

        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_sim = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.p_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.create_subscription(Vector3, '/dt/sync_error', lambda m: setattr(self, 'last_err', m), 10)
        self.create_subscription(Bool, '/dt/sync_ok', lambda m: setattr(self, 'last_ok', m.data), 10)
        self.create_subscription(String, '/dt/alerts', self._on_alert, 10)
        self.create_subscription(Float64, '/dt/latency_ms', lambda m: setattr(self, 'last_latency', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _on_alert(self, m) -> None:
        self.last_alert = m.data
        self.alert_count += 1

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
    """In sim_only/both a missing/stale sim pose stream is itself a desync: sync_supervisor must
    flip /dt/sync_ok False and raise /dt/alerts, not silently skip the tick (F5)."""
    har, ex, _ = world
    har.emit_sim = False              # the sim world never reports its pose
    ok = _spin_until(ex, lambda: har.last_ok is False
                     and 'stale/absent' in (har.last_alert or ''), secs=4.0)
    assert ok, "a missing/stale pose stream must flip sync_ok False and alert, not go silent (F5)"


def test_unstamped_command_still_measures_latency(world):
    """Latency is measured on the supervisor's OWN clock at message arrival — header stamps are not
    consulted (they cross clock domains between machines/sim). An unstamped /cmd_vel therefore still
    yields a valid (small, non-negative) latency sample."""
    har, ex, _ = world
    har.stamp_cmd = False             # zero header stamp: irrelevant to arrival-clock measurement
    har.cmd_vx = 0.2
    har.odom_vx = 0.2
    ok = _spin_until(ex, lambda: har.last_latency is not None, secs=4.0)
    # The upper bound is the discriminating part: a header-stamp implementation would difference
    # the zero cmd stamp against an epoch odom stamp -> ~1.7e12 ms, which must FAIL here.
    assert ok and 0.0 <= har.last_latency < 1000.0


def test_real_only_without_sim_world_is_healthy():
    """real_only has NO sim world: the supervisor must NOT alert-spam 'stream stale/absent' at the
    tick rate for the whole lab session, sync_ok must hold True, and the CSV evidence must collect
    rows (it previously got a header and nothing else)."""
    import glob as _glob
    import tempfile
    tmp = tempfile.mkdtemp()
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'real_only'),
        Parameter('sync_log_dir', Parameter.Type.STRING, tmp),
    ])
    har = Harness()
    har.emit_sim = False              # no /dt/sim_pose exists in real_only — by design
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    try:
        ok = _spin_until(ex, lambda: har.last_ok is True and har.last_err is not None, secs=6.0)
        assert ok, "real_only with a live real pose stream must report sync_ok True"
        assert har.last_err.x == 0.0, "pose discrepancy degenerates to 0 with no sim world"
        assert not (har.last_alert or '').startswith('SYNC pose stream'), \
            "no stale-stream alert storm in real_only"
        files = _glob.glob(os.path.join(tmp, 'sync_metrics_*.csv'))
        assert files
        with open(files[0], encoding='utf-8') as f:
            lines = f.read().strip().splitlines()
        assert len(lines) >= 2, "the Rubric-② CSV must collect data rows in real_only"
    finally:
        ex.shutdown()
        sup.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


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


def _pump_for(ex, secs: float) -> None:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)


def test_both_mode_stop_skew_alerts_when_over_budget(tmp_path):
    """The documented stop_skew_ms tolerance is loaded, compared, and alerted on a safety event in
    `both` mode (F3); and the pending stop-skew is consumed one-shot even with the CSV disabled (F4).
    Stop times are taken on the SUPERVISOR's clock at arrival, so the skew is forced by ACTUALLY
    delaying the sim stop by ~0.2 s of wall time (not by fabricating header stamps)."""
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
        har.p_aodom.publish(_stamped_odom(0.0, 100.1))  # real STOPS now...
        _pump_for(ex, 0.20)                             # ...the sim stops ~200 ms of WALL time later
        har.p_sodom.publish(_stamped_odom(0.0, 100.0))  # -> |skew| ~200 ms > 50 budget
        _pump(ex, 15)
        har.p_safety.publish(Bool(data=True))           # safety edge captures the skew
        got = _spin_until(ex, lambda: 'STOP-SKEW' in (har.last_alert or ''), secs=4.0)
        assert got, "stop-skew over the documented budget must raise /dt/alerts (F3)"
        _pump(ex, 10)
        assert sup._pending_stop_skew is None, \
            "pending stop-skew must be consumed one-shot even with the CSV off (F4)"
    finally:
        ex.shutdown()
        sup.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def test_both_mode_stop_skew_immune_to_clock_domain_mismatch(tmp_path):
    """REGRESSION (the headline both-mode bug): the real robot stamps odom with the Pi's WALL clock
    (~1.7e9 s) while the gz bridge stamps /sim/odom with SIM time (~seconds). Differencing those
    header stamps made every stop_skew astronomically over budget. Measured on the supervisor's own
    arrival clock, two near-simultaneous stops must produce NO stop-skew alert despite header stamps
    from wildly different clock domains."""
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'both'),
        Parameter('stop_skew_ms', Parameter.Type.DOUBLE, 300.0),
        Parameter('sync_log_enable', Parameter.Type.BOOL, False),
        Parameter('sync_log_dir', Parameter.Type.STRING, str(tmp_path)),
    ])
    har = BothHarness()
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    try:
        _pump(ex, 15)
        har.p_aodom.publish(_stamped_odom(0.3, 1.7e9))      # real: epoch wall-clock stamps
        har.p_sodom.publish(_stamped_odom(0.3, 5.0))        # sim: small sim-time stamps
        _pump(ex, 15)
        har.p_aodom.publish(_stamped_odom(0.0, 1.7e9 + 0.1))   # both stop (near-)simultaneously
        har.p_sodom.publish(_stamped_odom(0.0, 5.1))
        _pump(ex, 15)
        har.p_safety.publish(Bool(data=True))
        _pump_for(ex, 1.0)                                  # give the tick time to (not) alert
        assert 'STOP-SKEW' not in (har.last_alert or ''), \
            "near-simultaneous stops must not alert just because header clock domains differ"
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
    drives /cmd_vel and /dt/odom_active by hand to force a specific command->motion latency."""
    def __init__(self) -> None:
        super().__init__('sync_latency_harness')
        self.last_alert: str | None = None
        self.last_latency: float | None = None
        self.alert_count = 0
        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_sim = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.p_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.create_subscription(String, '/dt/alerts', self._on_alert, 10)
        self.create_subscription(Float64, '/dt/latency_ms', lambda m: setattr(self, 'last_latency', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _on_alert(self, m) -> None:
        self.last_alert = m.data
        self.alert_count += 1

    def _tick(self) -> None:
        self.p_real.publish(_pose(0.0, 0.0, 0.0))    # both worlds agree -> only LATENCY can alert
        self.p_sim.publish(_pose(0.0, 0.0, 0.0))


def test_latency_over_budget_alerts_exactly_once(tmp_path):
    """A command->motion latency beyond latency_budget_ms must raise a LATENCY /dt/alerts (Rubric ②
    tolerance) — and exactly ONCE per measured sample: one slow command must not flood /dt/alerts
    at the 5 Hz tick rate until the next motion onset (the sample is consumed one-shot). Latency is
    arrival-clock, so the 'slow robot' is a real ~0.4 s wall-time gap between command and motion."""
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
        har.p_cmd.publish(_stamped_cmd(0.3, 100.0))     # command...
        _pump_for(ex, 0.40)                             # ...the robot takes ~400 ms of WALL time
        har.p_odom.publish(_stamped_odom(0.3, 100.0))   # ...to actually move -> ~400 ms > 250
        got = _spin_until(ex, lambda: 'LATENCY' in (har.last_alert or ''), secs=4.0)
        assert got, "a latency beyond the documented budget must raise /dt/alerts"
        assert har.last_latency is not None and har.last_latency >= 300.0, \
            f"measured latency should be ~400 ms, got {har.last_latency}"
        count_at_alert = har.alert_count
        _pump_for(ex, 1.5)                              # ~7 more sync ticks
        assert har.alert_count == count_at_alert, \
            "one over-budget sample must alert ONCE, not every 5 Hz tick"
    finally:
        ex.shutdown()
        sup.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def test_out_of_tolerance_alert_is_edge_triggered_not_tick_spam(tmp_path):
    """A PERSISTING out-of-tolerance condition re-alerts at most every alert_repeat_s (edge-trigger
    + repeat throttle), never at the raw 5 Hz tick rate."""
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'sim_only'),
        Parameter('alert_repeat_s', Parameter.Type.DOUBLE, 5.0),
        Parameter('sync_log_enable', Parameter.Type.BOOL, False),
        Parameter('sync_log_dir', Parameter.Type.STRING, str(tmp_path)),
    ])
    har = Harness()
    har.sim = (0.5, 0.0, 0.0)        # 50 cm >> 15 cm tolerance, persisting
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    try:
        ok = _spin_until(ex, lambda: 'OUT-OF-TOLERANCE' in (har.last_alert or ''), secs=4.0)
        assert ok
        count_at_alert = har.alert_count
        _pump_for(ex, 1.5)           # ~7 sync ticks while still out of tolerance
        assert har.alert_count <= count_at_alert, \
            "a persisting condition must not re-alert within alert_repeat_s"
    finally:
        ex.shutdown()
        sup.destroy_node()
        har.destroy_node()
        rclpy.shutdown()
