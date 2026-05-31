"""fake_robot — a kinematic stand-in for the real TurtleBot3.

Publishes the real robot's BARE topics (/scan, /odom, /battery_state, /tf odom->base_link) and
subscribes the mediator's gated /cmd_vel (TwistStamped), integrating it into /odom — so commanding
it actually moves. This lets real_only/both be exercised at home without hardware (the real side
becomes fake_robot on the bare topics, the sim on /sim/*). Kinematics live in lib.sync; this is
just the ROS wiring. Not used in sim_only — there the Gazebo robot is the bare robot.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import TransformBroadcaster

from algae_dt.lib import geometry, hud, sync


class FakeRobot(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('fake_robot', **kwargs)
        gp = self._declare
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

        self._pose = (0.0, 0.0, 0.0)
        self._cmd = (0.0, 0.0)
        self._t = None
        self._t0 = self._now()
        self._tf = TransformBroadcaster(self)

        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_batt = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)

        self.create_timer(1.0 / self.cmd_rate, self._step)
        self.create_timer(0.2, self._scan)            # ~5 Hz like the LDS-02
        self.create_timer(1.0, self._battery)
        self.get_logger().info(f"fake_robot up: {self.scan_n}-beam scan, front={self.front_m} m")

    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: TwistStamped) -> None:
        self._cmd = (msg.twist.linear.x, msg.twist.angular.z)

    def _step(self) -> None:
        now = self._now()
        if self._t is None:
            self._t = now
            return
        dt = now - self._t
        self._t = now
        if dt <= 0.0:
            return
        self._pose = sync.integrate_unicycle(*self._pose, self._cmd[0], self._cmd[1], dt)
        x, y, yaw = self._pose
        stamp = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = geometry.quaternion_from_yaw(yaw)
        od.twist.twist.linear.x = self._cmd[0]
        od.twist.twist.angular.z = self._cmd[1]
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
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
