"""In-process integration tests for twin_resync (PLAN T7.1, Rubric ②).

rclpy-only (skipped on a bare host). The pure policy rules are pinned in test_resync.py; here the
ROS wiring is exercised: /dt/resync_cmd + a sustained out-of-tolerance /dt/sync_error fire the
(faked) gz set_pose with the REAL robot's pose, publish /dt/resync_event, and a failed gz call
publishes /dt/alerts instead. The gz side effect is faked by overriding _set_entity_pose — the
same pattern test_dynamic_obstacle uses for _gz.
"""
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, Vector3            # noqa: E402
from rclpy.executors import SingleThreadedExecutor            # noqa: E402
from rclpy.node import Node                                    # noqa: E402
from rclpy.parameter import Parameter                          # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,           # noqa: E402
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Empty, String                   # noqa: E402

from algae_dt.lib import geometry                              # noqa: E402
from algae_dt.twin_resync import TwinResync                    # noqa: E402


def _latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)

FAST = [
    Parameter('mode', Parameter.Type.STRING, 'both'),
    Parameter('resync_sustain_s', Parameter.Type.DOUBLE, 0.4),
    Parameter('resync_cooldown_s', Parameter.Type.DOUBLE, 0.4),
    Parameter('resync_tick_hz', Parameter.Type.DOUBLE, 20.0),
]


class FakeGz(TwinResync):
    """TwinResync with the gz side effect captured instead of executed."""

    def __init__(self, ok=True, **kwargs):
        super().__init__(**kwargs)
        self.gz_ok = ok
        self.requests: list[str] = []

    def _set_entity_pose(self, req):
        self.requests.append(req)
        return (True, '') if self.gz_ok else (False, 'boom')


class Harness(Node):
    def __init__(self, localized: bool = True):
        super().__init__('resync_test_harness')
        self.real = (1.2, -0.4, 1.5)
        self.err = (0.0, 0.0)
        self.last_event = None
        self.last_alert = None
        self.p_cmd = self.create_publisher(Empty, '/dt/resync_cmd', 10)
        self.p_err = self.create_publisher(Vector3, '/dt/sync_error', 10)
        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_localized = self.create_publisher(Bool, '/dt/localized', _latched())
        self.p_localized.publish(Bool(data=localized))   # latched, like the mediator's
        self.create_subscription(String, '/dt/resync_event',
                                 lambda m: setattr(self, 'last_event', m.data), 10)
        self.create_subscription(String, '/dt/alerts',
                                 lambda m: setattr(self, 'last_alert', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = self.real[0], self.real[1]
        ps.pose.orientation.z, ps.pose.orientation.w = geometry.quaternion_from_yaw(self.real[2])
        self.p_real.publish(ps)
        self.p_err.publish(Vector3(x=self.err[0], y=self.err[1]))


def _world(node_cls=FakeGz, overrides=(), localized=True, **node_kw):
    rclpy.init()
    node = node_cls(parameter_overrides=FAST + list(overrides), **node_kw)
    har = Harness(localized=localized)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(har)
    return node, har, ex


def _teardown(node, har, ex):
    ex.shutdown()
    node.destroy_node()
    har.destroy_node()
    rclpy.shutdown()


def _spin_until(ex, pred, secs=6.0) -> bool:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def test_manual_resync_teleports_to_the_real_pose_and_publishes_event():
    node, har, ex = _world()
    try:
        _spin_until(ex, lambda: node._t_real is not None, secs=3.0)   # inputs flowing
        har.p_cmd.publish(Empty())
        assert _spin_until(ex, lambda: har.last_event is not None), \
            "a manual RESYNC with fresh inputs must execute and publish /dt/resync_event"
        assert har.last_event.startswith('manual')
        assert len(node.requests) == 1
        req = node.requests[0]
        assert 'name: "burger_sim"' in req
        assert 'x: 1.2' in req and 'y: -0.4' in req, "the teleport target is the REAL pose"
        assert 'orientation' in req, "the snap must include the real heading"
    finally:
        _teardown(node, har, ex)


def test_auto_resync_fires_after_sustained_out_of_tolerance_only():
    node, har, ex = _world()
    try:
        har.err = (0.30, 0.0)               # > tol_pose_xy_m (0.15), continuously
        assert _spin_until(ex, lambda: har.last_event is not None), \
            "a sustained out-of-tolerance error must auto-resync"
        assert har.last_event.startswith('auto')
        assert 'dxy=0.30' in har.last_event
    finally:
        _teardown(node, har, ex)


def test_transient_breach_does_not_auto_resync():
    # generous sustain (1.5 s) vs a short breach (0.3 s): wide margins so a loaded CI host can
    # never stretch the breach past the sustain window and flake this test
    node, har, ex = _world(overrides=[Parameter('resync_sustain_s', Parameter.Type.DOUBLE, 1.5)])
    try:
        _spin_until(ex, lambda: node._t_err is not None, secs=3.0)
        har.err = (0.30, 0.0)               # breach...
        _spin_until(ex, lambda: False, secs=0.3)
        har.err = (0.02, 0.0)               # ...recovers well inside the sustain window
        _spin_until(ex, lambda: False, secs=1.0)
        assert har.last_event is None, "a transient spike must not teleport the twin"
        assert node.requests == []
    finally:
        _teardown(node, har, ex)


def test_failed_gz_call_alerts_instead_of_event():
    node, har, ex = _world(ok=False)
    try:
        _spin_until(ex, lambda: node._t_real is not None, secs=3.0)
        har.p_cmd.publish(Empty())
        assert _spin_until(ex, lambda: har.last_alert is not None), \
            "a failed gz set_pose must publish /dt/alerts (fail-loud, RULES §C)"
        assert har.last_alert.startswith('RESYNC FAILED (manual)')
        assert har.last_event is None, "no /dt/resync_event when nothing was corrected"
    finally:
        _teardown(node, har, ex)


def test_resync_idles_outside_both_mode():
    node, har, ex = _world(overrides=[Parameter('mode', Parameter.Type.STRING, 'sim_only')])
    try:
        _spin_until(ex, lambda: node._t_real is not None, secs=3.0)
        har.p_cmd.publish(Empty())
        har.err = (0.50, 0.50)
        _spin_until(ex, lambda: False, secs=1.0)
        assert har.last_event is None and node.requests == [], \
            "sim_only has no mirror sim: the node must idle (no timer, click refused)"
    finally:
        _teardown(node, har, ex)


def test_no_resync_until_amcl_is_localized():
    """REAL-LAB GUARD: before the operator's 2D Pose Estimate, /dt/real_pose is identity-lifted
    odom with NO map meaning — a fire then could teleport the mirror into a wall, whose scan would
    then block the REAL robot through the dual-LiDAR gate. With fresh poses + errors but
    /dt/localized False, neither a click nor a sustained breach may fire; flipping localized True
    releases the queued click."""
    node, har, ex = _world(localized=False)
    try:
        _spin_until(ex, lambda: node._t_real is not None, secs=3.0)   # inputs ARE flowing
        har.err = (0.50, 0.50)                                        # sustained breach...
        har.p_cmd.publish(Empty())                                    # ...AND an operator click
        _spin_until(ex, lambda: False, secs=1.2)
        assert node.requests == [] and har.last_event is None, \
            "unlocalized pose must refuse every fire (manual AND auto)"
        assert node._pending_manual is True, "the click stays queued, not dropped"
        har.err = (0.0, 0.0)                                          # back in tolerance
        har.p_localized.publish(Bool(data=True))                      # 2D Pose Estimate lands
        assert _spin_until(ex, lambda: har.last_event is not None), \
            "localization must release the queued manual resync"
        assert har.last_event.startswith('manual')
    finally:
        _teardown(node, har, ex)


def test_no_resync_without_fresh_inputs():
    # No harness streams at all: a RESYNC click with no /dt/real_pose // /dt/sync_error must
    # stay queued, never executed ("never teleport onto a target the data doesn't support").
    rclpy.init()
    node = FakeGz(parameter_overrides=FAST)
    pub = rclpy.create_node('silent_resync_harness')
    p_cmd = pub.create_publisher(Empty, '/dt/resync_cmd', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(pub)
    try:
        p_cmd.publish(Empty())
        _spin_until(ex, lambda: False, secs=1.0)
        assert node.requests == []
        assert node._pending_manual is True, "the click stays queued until the data supports it"
    finally:
        ex.shutdown()
        node.destroy_node()
        pub.destroy_node()
        rclpy.shutdown()
