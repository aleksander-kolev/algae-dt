"""Tests for fake_robot + the hardware-free bidirectional ① loop (PLAN T5.1b, DECISION D7). rclpy-only.

(1) fake_robot integrates the gated /cmd_vel into /odom and publishes /scan + /battery_state.
(2) End-to-end ① loop: /dt/cmd_vel_raw -> mediator gate -> /cmd_vel -> fake_robot moves -> /odom ->
    mediator -> /dt/real_pose, with /scan -> /dt/scan_active. Proves digital<->real both directions
    without any hardware.
"""
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped   # noqa: E402
from nav_msgs.msg import Odometry                                 # noqa: E402
from rclpy.executors import SingleThreadedExecutor                # noqa: E402
from rclpy.node import Node                                        # noqa: E402
from rclpy.parameter import Parameter                              # noqa: E402
from sensor_msgs.msg import BatteryState, LaserScan               # noqa: E402
from std_msgs.msg import Bool                                      # noqa: E402

from algae_dt.fake_robot import FakeRobot                          # noqa: E402
from algae_dt.twin_mediator import TwinMediator                    # noqa: E402


def _spin_until(ex, pred, secs=6.0):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


class CmdVelHarness(Node):
    """Publishes the gated /cmd_vel directly and watches /odom + /scan + /battery_state."""

    def __init__(self):
        super().__init__('fakebot_harness')
        self.vx = 0.0
        self.odom_x = None
        self.got_scan = False
        self.got_batt = False
        self.p_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom_x', m.pose.pose.position.x), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'got_scan', True), qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', lambda m: setattr(self, 'got_batt', True), 10)
        self.create_timer(1.0 / 30.0, lambda: self.p_cmd.publish(TwistStamped(twist=_t(self.vx))))


def _t(vx):
    t = Twist()
    t.linear.x = vx
    return t


def test_fake_robot_integrates_cmd_and_publishes_sensors():
    rclpy.init()
    bot = FakeRobot()
    har = CmdVelHarness()
    ex = SingleThreadedExecutor()
    ex.add_node(bot)
    ex.add_node(har)
    try:
        har.vx = 0.2
        assert _spin_until(ex, lambda: har.odom_x is not None and har.odom_x > 0.02), "odom advances under command"
        assert _spin_until(ex, lambda: har.got_scan and har.got_batt), "publishes /scan and /battery_state"
    finally:
        ex.shutdown()
        bot.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


class BusHarness(Node):
    """Drives the pre-safety bus and watches the mirrored real pose + active scan."""

    def __init__(self):
        super().__init__('bus_harness')
        self.vx = 0.0
        self.real_x = None
        self.got_scan_active = False
        self.p_bus = self.create_publisher(TwistStamped, '/dt/cmd_vel_raw', 10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(PoseStamped, '/dt/real_pose', lambda m: setattr(self, 'real_x', m.pose.position.x), 10)
        self.create_subscription(LaserScan, '/dt/scan_active', lambda m: setattr(self, 'got_scan_active', True), qos_profile_sensor_data)
        self.create_timer(1.0 / 30.0, lambda: self.p_bus.publish(TwistStamped(twist=_t(self.vx))))


class SafetyHarness(Node):
    """Commands forward on the bus; watches the gated /cmd_vel + /dt/safety."""

    def __init__(self):
        super().__init__('safety_harness')
        self.vx = 0.0
        self.cmd_x = None
        self.safety = None
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.p_bus = self.create_publisher(TwistStamped, '/dt/cmd_vel_raw', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', lambda m: setattr(self, 'cmd_x', m.twist.linear.x), 10)
        self.create_subscription(Bool, '/dt/safety', lambda m: setattr(self, 'safety', m.data), latched)
        self.create_timer(1.0 / 30.0, lambda: self.p_bus.publish(TwistStamped(twist=_t(self.vx))))


def test_front_obstacle_triggers_safety_stop_through_mediator():
    # Pillar III, hardware-free: a 20 cm obstacle on the (fake) real robot's /scan must zero forward
    # motion at the mediator and raise /dt/safety, even while forward is commanded on the bus.
    rclpy.init()
    bot = FakeRobot(parameter_overrides=[Parameter('fake_front_m', Parameter.Type.DOUBLE, 0.2)])
    med = TwinMediator(parameter_overrides=[Parameter('mode', Parameter.Type.STRING, 'real_only')])
    har = SafetyHarness()
    ex = SingleThreadedExecutor()
    for n in (bot, med, har):
        ex.add_node(n)
    try:
        _spin_until(ex, lambda: False, secs=1.5)
        har.vx = 0.2
        assert _spin_until(ex, lambda: har.safety is True, secs=8.0), "front obstacle raises /dt/safety"
        assert _spin_until(ex, lambda: har.cmd_x == 0.0, secs=3.0), "forward motion is zeroed at the gate"
    finally:
        ex.shutdown()
        for n in (bot, med, har):
            n.destroy_node()
        rclpy.shutdown()


def test_bidirectional_loop_through_mediator():
    rclpy.init()
    bot = FakeRobot()
    med = TwinMediator(parameter_overrides=[Parameter('mode', Parameter.Type.STRING, 'real_only')])
    har = BusHarness()
    ex = SingleThreadedExecutor()
    for n in (bot, med, har):
        ex.add_node(n)
    try:
        _spin_until(ex, lambda: False, secs=1.5)   # let the 4-hop discovery chain settle
        har.vx = 0.2                       # operator/Nav2 commands forward on the bus
        # gated cmd reaches the fake robot, which moves; its odom is mirrored to /dt/real_pose
        assert _spin_until(ex, lambda: har.real_x is not None and har.real_x > 0.02, secs=15.0), \
            "digital->real->digital loop: commanding the bus moves the (fake) real robot"
        assert har.got_scan_active, "real /scan is republished to /dt/scan_active"
    finally:
        ex.shutdown()
        for n in (bot, med, har):
            n.destroy_node()
        rclpy.shutdown()
