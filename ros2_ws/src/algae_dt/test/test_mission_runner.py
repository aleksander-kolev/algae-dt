"""In-process tests for mission_runner with an injected fake navigator (PLAN T2.2).

rclpy-only. Verifies honest task accounting (RULES §B-7/§B-8): SUCCEEDED -> spray + mark treated;
nav FAILED -> skipped (grey), NO spray; operator Stop/E-STOP mid-nav -> bloom stays PENDING.
The fake navigator stands in for Nav2 so the full mission loop runs without a live stack.
"""
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import TwistStamped              # noqa: E402
from nav2_simple_commander.robot_navigator import TaskResult   # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from rclpy.executors import SingleThreadedExecutor       # noqa: E402
from rclpy.node import Node                               # noqa: E402
from rclpy.parameter import Parameter                     # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,      # noqa: E402
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String                     # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray    # noqa: E402

from algae_dt.lib import blooms as B                       # noqa: E402
from algae_dt.mission_runner import MissionRunner, _COLORS  # noqa: E402


class FakeNavigator:
    def __init__(self, outcomes, complete_after=1):
        self.outcomes = list(outcomes)
        self.complete_after = complete_after
        self._i = -1
        self._polls = 0
        self.goals = []
        self.cancelled = False

    def waitUntilNav2Active(self):
        pass

    def goToPose(self, pose):
        self.goals.append(pose)
        self._i += 1
        self._polls = 0

    def isTaskComplete(self):
        self._polls += 1
        return self._polls >= self.complete_after

    def getResult(self):
        return self.outcomes[self._i] if 0 <= self._i < len(self.outcomes) else TaskResult.FAILED

    def cancelTask(self):
        self.cancelled = True


def _latched(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _state_of(marker: Marker) -> str:
    c = (round(marker.color.r, 2), round(marker.color.g, 2), round(marker.color.b, 2))
    for st, (r, g, b, _a) in _COLORS.items():
        if (round(r, 2), round(g, 2), round(b, 2)) == c:
            return st
    return '?'


class Harness(Node):
    def __init__(self, blooms_xy) -> None:
        super().__init__('mission_test_harness')
        self.blooms_xy = blooms_xy
        self.last_markers = None
        self.last_state = None
        self.max_omega = 0.0

        self.p_blooms = self.create_publisher(MarkerArray, '/dt/blooms', _latched())
        self.p_cmd = self.create_publisher(String, '/dt/mission_cmd', 10)
        self.p_estop = self.create_publisher(Bool, '/dt/estop', _latched())
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.create_subscription(MarkerArray, '/dt/markers', lambda m: setattr(self, 'last_markers', m), 10)
        self.create_subscription(String, '/dt/mission_state', lambda m: setattr(self, 'last_state', m.data), 10)
        self.create_subscription(TwistStamped, '/dt/cmd_vel_raw', self._on_spin, 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _on_spin(self, msg: TwistStamped) -> None:
        self.max_omega = max(self.max_omega, abs(msg.twist.angular.z))

    def _tick(self) -> None:
        arr = MarkerArray()
        for i, (x, y) in enumerate(self.blooms_xy):
            m = Marker()
            m.id = i
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            arr.markers.append(m)
        self.p_blooms.publish(arr)
        self.p_odom.publish(Odometry())     # robot at origin

    def send(self, cmd: str) -> None:
        self.p_cmd.publish(String(data=cmd))

    def estop(self, on: bool) -> None:
        self.p_estop.publish(Bool(data=on))


def _build(navigator, blooms_xy, spray_seconds=0.2):
    rclpy.init()
    runner = MissionRunner(navigator=navigator, parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'sim_only'),
        Parameter('spray_seconds', Parameter.Type.DOUBLE, spray_seconds),
    ])
    har = Harness(blooms_xy)
    ex = SingleThreadedExecutor()
    ex.add_node(runner)
    ex.add_node(har)
    return runner, har, ex


def _spin_until(ex, pred, secs=8.0) -> bool:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def _teardown(runner, har, ex):
    runner._running = False
    ex.shutdown()
    runner.destroy_node()
    har.destroy_node()
    rclpy.shutdown()


def _markers_by_id(har):
    return {m.id: _state_of(m) for m in har.last_markers.markers} if har.last_markers else {}


def test_success_sprays_and_marks_treated():
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED, TaskResult.SUCCEEDED]),
                             [(0.1, 0.0), (0.2, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 2), "blooms registered"
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'complete'), "mission completes"
        states = _markers_by_id(har)
        assert states == {0: B.TREATED, 1: B.TREATED}
        assert har.max_omega > 0.0, "spray spin must be published to /dt/cmd_vel_raw"
    finally:
        _teardown(runner, har, ex)


def test_nav_failure_skips_without_spraying():
    runner, har, ex = _build(FakeNavigator([TaskResult.FAILED]), [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'complete')
        assert _markers_by_id(har) == {0: B.SKIPPED}
        assert har.max_omega == 0.0, "a failed nav must NOT spray"
    finally:
        _teardown(runner, har, ex)


def test_estop_midnav_leaves_bloom_pending():
    # Navigation never completes on its own (complete_after huge); E-STOP interrupts it.
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], complete_after=10_000),
                             [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'navigating:0'), "reaches navigating"
        har.estop(True)
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.PENDING), \
            "operator E-STOP mid-nav must leave the bloom PENDING (honest accounting)"
        assert har.max_omega == 0.0
    finally:
        _teardown(runner, har, ex)
