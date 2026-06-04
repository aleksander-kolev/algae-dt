"""fake_robot — a kinematic stand-in for the real TurtleBot3 (PLAN T5.1b, DECISION D7).

Publishes the real robot's BARE topics (/scan, /odom, /battery_state, /tf) and subscribes the
mediator's gated /cmd_vel (TwistStamped), integrating it into /odom — so commanding it actually
"moves" it. This makes pillar ① (real->digital and digital->real arrows) and the topic-collision
rule demonstrable WITHOUT hardware, and lets `both` mode be validated at home (the real side =
fake_robot on bare topics, the sim on /sim/*). Pure kinematics live in lib.sync; this is ROS
wiring. NOT used in sim_only (the Gazebo robot is the bare robot there).

TF: mirrors the real Burger's frame chain so AMCL/Nav2 costmaps can transform /scan —
odom -> base_footprint (dynamic, like the real odometry) plus the static base_footprint ->
base_link -> base_scan links from the burger URDF. (Broadcasting only odom->base_link left
base_scan unresolvable: every laser TF lookup failed and Nav2 was dead in the hardware-free path.)

Real-robot fidelity: the real turtlebot3_node STOPS when /cmd_vel goes silent; the fake robot
mirrors that with max_cmd_age_s — it must never coast forever on a stale command.
"""
from __future__ import annotations

import math

import functools

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from algae_dt.lib import geometry, hud, sync
from algae_dt.lib.ros_utils import declare_get

# Burger URDF static offsets (metres): base_footprint -> base_link, base_link -> base_scan.
_BASE_LINK_Z = 0.010
_SCAN_XYZ = (-0.032, 0.0, 0.172)


class FakeRobot(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('fake_robot', **kwargs)
        gp = functools.partial(declare_get, self)
        self.scan_n = int(gp('fake_scan_beams', 360))
        self.scan_far = gp('fake_scan_far_m', 3.0)         # background range when nothing is ahead
        self.front_m = gp('fake_front_m', 3.0)             # obstacle dead-ahead (set <0.25 to test stop)
        self.range_min = gp('scan_range_min_m', 0.12)
        self.range_max = gp('scan_range_max_m', 3.5)
        self.batt_start = gp('battery_sim_start_v', 12.5)
        self.batt_drain = gp('battery_sim_drain_vps', 0.02)
        self.batt_min = gp('battery_sim_min_v', 11.2)
        self.batt_empty_v = gp('battery_empty_v', 9.0)
        self.batt_full_v = gp('battery_full_v', 12.6)
        self.cmd_rate = gp('cmd_rate_hz', 20.0)
        self.max_cmd_age_s = gp('max_cmd_age_s', 0.5)      # stop on cmd silence, like the real Burger

        self._pose = (0.0, 0.0, 0.0)
        self._cmd = (0.0, 0.0)
        self._cmd_t = float('-inf')
        self._t = None
        self._t0 = self._now()
        self._tf = TransformBroadcaster(self)
        self._tf_static = StaticTransformBroadcaster(self)
        self._send_static_tf()

        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_batt = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)

        self.create_timer(1.0 / self.cmd_rate, self._step)
        self.create_timer(0.2, self._scan)            # ~5 Hz like the LDS-02
        self.create_timer(1.0, self._battery)
        self.get_logger().info(f"fake_robot up: {self.scan_n}-beam scan, front={self.front_m} m")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _send_static_tf(self) -> None:
        """The burger URDF's static links, so the TF tree matches the real robot's:
        base_footprint -> base_link -> base_scan."""
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for parent, child, (tx, ty, tz) in (
                ('base_footprint', 'base_link', (0.0, 0.0, _BASE_LINK_Z)),
                ('base_link', 'base_scan', _SCAN_XYZ)):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self._tf_static.sendTransform(tfs)

    def _on_cmd(self, msg: TwistStamped) -> None:
        self._cmd = (msg.twist.linear.x, msg.twist.angular.z)
        self._cmd_t = self._now()

    def _step(self) -> None:
        now = self._now()
        if self._t is None:
            self._t = now
            return
        dt = now - self._t
        self._t = now
        if dt <= 0.0:
            return
        if now - self._cmd_t > self.max_cmd_age_s:
            self._cmd = (0.0, 0.0)     # cmd_vel silent -> stop, exactly like the real turtlebot3_node
        self._pose = sync.integrate_unicycle(*self._pose, self._cmd[0], self._cmd[1], dt)
        x, y, yaw = self._pose
        stamp = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_footprint'   # the real Burger's odom child frame
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = geometry.quaternion_from_yaw(yaw)
        od.twist.twist.linear.x = self._cmd[0]
        od.twist.twist.angular.z = self._cmd[1]
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z, tf.transform.rotation.w = geometry.quaternion_from_yaw(yaw)
        self._tf.sendTransform(tf)

    def _scan(self) -> None:
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'base_scan'
        s.angle_min = -math.pi
        s.angle_increment = 2.0 * math.pi / self.scan_n
        s.angle_max = s.angle_min + (self.scan_n - 1) * s.angle_increment   # consistent geometry for Nav2
        s.range_min = self.range_min
        s.range_max = self.range_max
        s.ranges = [self.scan_far] * self.scan_n
        for i in (self.scan_n // 2 - 1, self.scan_n // 2, self.scan_n // 2 + 1):
            s.ranges[i] = self.front_m            # obstacle dead-ahead (clear by default)
        self.pub_scan.publish(s)

    def _battery(self) -> None:
        v = max(self.batt_min, self.batt_start - self.batt_drain * (self._now() - self._t0))
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = float(v)
        b.percentage = hud.battery_percentage(v, self.batt_empty_v, self.batt_full_v)
        b.present = True
        self.pub_batt.publish(b)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
