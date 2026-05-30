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
        self.last_cmd: TwistStamped | None = None
        self.last_safety: bool | None = None
        self.last_estop: bool | None = None
        self.last_mode: str | None = None
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
        self.create_subscription(PoseStamped, '/dt/sim_pose',
                                 lambda m: setattr(self, 'last_sim_pose', m), 10)
        self.create_subscription(BatteryState, '/dt/health', lambda m: setattr(self, 'last_health', m), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        self.p_cmd.publish(TwistStamped(twist=_twist(self.cmd_vx)))
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


def test_latched_mode_and_sim_pose_and_health(world):
    har, ex = world
    ok = _spin_until(ex, lambda: har.last_mode == 'sim_only'
                     and har.last_sim_pose is not None
                     and har.last_health is not None and har.last_health.voltage > 0.0)
    assert ok, "mode latched, sim pose mirrored from /odom, synthetic battery published"
    assert har.last_estop is False    # no estop at startup
