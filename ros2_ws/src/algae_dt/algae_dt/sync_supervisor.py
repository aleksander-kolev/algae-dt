"""sync_supervisor — Real-Time Synchronization & Tolerances (Rubric ②). PLAN T3.2.

The most-overlooked, highest-value deliverable, made first-class. It MEASURES the real<->sim
discrepancy and reports it, not just a boolean:
  * pose discrepancy (Δxy, Δyaw) from /dt/real_pose vs /dt/sim_pose,
  * front-range sensor delta (only meaningful in `both`; 0 otherwise),
  * command->motion latency: when /cmd_vel commands motion, time until /dt/odom_active actually
    moves (|v|>motion_eps OR |w|>motion_eps),
  * stop_skew (real-stop vs sim-stop time delta) on safety events in `both`,
then publishes /dt/sync_error (Vector3), /dt/latency_ms (Float64), /dt/sync_ok (Bool, latched),
/dt/alerts (String, out of the documented twin.yaml tolerances) and appends a flushed CSV row each
tick. All maths live in the unit-tested lib.sync + lib.metrics; this node is the ROS wiring + I/O.
"""
from __future__ import annotations

import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

from algae_dt.lib import geometry, metrics, safety, sync

INF = float('inf')


def _latched() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SyncSupervisor(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('sync_supervisor', **kwargs)
        gp = self._declare
        self.mode = gp('mode', 'sim_only')
        self.front_sector_rad = gp('front_sector_rad', 0.70)
        self.range_min = gp('scan_range_min_m', 0.12)
        self.range_max = gp('scan_range_max_m', 3.5)
        self.sync_rate_hz = gp('sync_rate_hz', 5.0)
        self.tol_xy = gp('tol_pose_xy_m', 0.15)
        self.tol_yaw = gp('tol_pose_yaw_rad', 0.26)
        self.tol_sensor = gp('tol_sensor_range_m', 0.20)
        self.latency_budget_ms = gp('latency_budget_ms', 250.0)
        self.motion_eps_mps = gp('motion_eps_mps', 0.02)
        self.motion_eps_radps = gp('motion_eps_radps', 0.05)
        self.csv_prefix = gp('sync_log_csv', 'sync_metrics')
        self.csv_enable = gp('sync_log_enable', True)
        self.csv_dir = gp('sync_log_dir', '.')

        self._both = self.mode == 'both'

        # state
        self._real_pose = None          # (x, y, yaw)
        self._sim_pose = None
        self._active_front = INF
        self._sim_front = INF
        self._latency_ms = float('nan')
        self._cmd_moving = False
        self._t_cmd = None
        self._awaiting_motion = False
        self._real_moving = False
        self._sim_moving = False
        self._t_real_stop = None
        self._t_sim_stop = None
        self._safety_blocked = False
        self._pending_stop_skew = None

        # publishers
        self.pub_err = self.create_publisher(Vector3, '/dt/sync_error', 10)
        self.pub_latency = self.create_publisher(Float64, '/dt/latency_ms', 10)
        self.pub_ok = self.create_publisher(Bool, '/dt/sync_ok', _latched())
        self.pub_alerts = self.create_publisher(String, '/dt/alerts', 10)

        # subscriptions
        self.create_subscription(PoseStamped, '/dt/real_pose', self._on_real_pose, 10)
        self.create_subscription(PoseStamped, '/dt/sim_pose', self._on_sim_pose, 10)
        self.create_subscription(LaserScan, '/dt/scan_active', self._on_active_scan, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Odometry, '/dt/odom_active', self._on_active_odom, 10)
        self.create_subscription(Bool, '/dt/safety', self._on_safety, _latched())
        if self._both:
            self.create_subscription(LaserScan, '/sim/scan', self._on_sim_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/sim/odom', self._on_sim_odom, 10)

        self._csv = self._open_csv()
        self.create_timer(1.0 / self.sync_rate_hz, self._on_sync_timer)
        self.get_logger().info(
            f"sync_supervisor up: mode={self.mode} tol xy={self.tol_xy} yaw={self.tol_yaw} "
            f"sensor={self.tol_sensor} latency_budget={self.latency_budget_ms} ms"
            + (f" csv={self._csv.name}" if self._csv else " csv=off"))

    # ------------------------------------------------------------------ utils
    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp_s(self, stamp) -> float:
        s = stamp.sec + stamp.nanosec * 1e-9
        return s if s > 0.0 else self._now()

    @staticmethod
    def _xyyaw(ps: PoseStamped):
        p, o = ps.pose.position, ps.pose.orientation
        return (p.x, p.y, geometry.yaw_from_quaternion(o.z, o.w))

    def _front(self, scan: LaserScan) -> float:
        return safety.front_min_range(scan.ranges, scan.angle_min, scan.angle_increment,
                                      self.front_sector_rad,
                                      scan.range_min or self.range_min,
                                      scan.range_max or self.range_max)

    def _open_csv(self):
        if not self.csv_enable:
            return None
        os.makedirs(self.csv_dir, exist_ok=True)
        path = os.path.join(self.csv_dir, f"{self.csv_prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        f = open(path, 'a', encoding='utf-8')
        f.write(metrics.csv_header() + '\n')
        f.flush()
        return f

    # -------------------------------------------------------------- callbacks
    def _on_real_pose(self, msg): self._real_pose = self._xyyaw(msg)
    def _on_sim_pose(self, msg): self._sim_pose = self._xyyaw(msg)
    def _on_active_scan(self, msg): self._active_front = self._front(msg)
    def _on_sim_scan(self, msg): self._sim_front = self._front(msg)

    def _on_cmd(self, msg: TwistStamped) -> None:
        moving = (abs(msg.twist.linear.x) > self.motion_eps_mps
                  or abs(msg.twist.angular.z) > self.motion_eps_radps)
        if moving and not self._cmd_moving:
            self._t_cmd = self._stamp_s(msg.header.stamp)
            self._awaiting_motion = True
        self._cmd_moving = moving

    def _on_active_odom(self, msg: Odometry) -> None:
        moving = self._odom_moving(msg)
        now = self._stamp_s(msg.header.stamp)
        if self._awaiting_motion and moving and self._t_cmd is not None:
            self._latency_ms = max(0.0, metrics.latency_ms(self._t_cmd, now))
            self.pub_latency.publish(Float64(data=self._latency_ms))
            self._awaiting_motion = False
        if self._real_moving and not moving:
            self._t_real_stop = now
        self._real_moving = moving

    def _on_sim_odom(self, msg: Odometry) -> None:
        moving = self._odom_moving(msg)
        now = self._stamp_s(msg.header.stamp)
        if self._sim_moving and not moving:
            self._t_sim_stop = now
        self._sim_moving = moving

    def _odom_moving(self, msg: Odometry) -> bool:
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        return v > self.motion_eps_mps or abs(msg.twist.twist.angular.z) > self.motion_eps_radps

    def _on_safety(self, msg: Bool) -> None:
        blocked = bool(msg.data)
        # On a fresh block in `both`, capture the real-vs-sim stop-time skew once both have stopped.
        if blocked and not self._safety_blocked and self._both:
            if self._t_real_stop is not None and self._t_sim_stop is not None:
                self._pending_stop_skew = (self._t_real_stop - self._t_sim_stop) * 1000.0
        self._safety_blocked = blocked

    # --------------------------------------------------------------- sync tick
    def _on_sync_timer(self) -> None:
        if self._real_pose is None or self._sim_pose is None:
            return
        sim_front = self._sim_front if self._both else self._active_front   # 0 delta if single world
        err = sync.compute(self._real_pose, self._sim_pose, self._active_front, sim_front)
        self.pub_err.publish(Vector3(x=err.dxy, y=err.dyaw, z=err.sensor))

        ok = sync.within_tolerance(err, self.tol_xy, self.tol_yaw, self.tol_sensor)
        self.pub_ok.publish(Bool(data=ok))
        if not ok:
            self.pub_alerts.publish(String(data=(
                f"SYNC OUT-OF-TOLERANCE dxy={err.dxy:.3f}/{self.tol_xy} "
                f"dyaw={err.dyaw:.3f}/{self.tol_yaw} sensor={err.sensor:.3f}/{self.tol_sensor}")))
        if not math.isnan(self._latency_ms) and self._latency_ms > self.latency_budget_ms:
            self.pub_alerts.publish(String(data=(
                f"LATENCY {self._latency_ms:.0f} ms > budget {self.latency_budget_ms:.0f} ms")))

        if self._csv:
            lat = 0.0 if math.isnan(self._latency_ms) else self._latency_ms
            self._csv.write(metrics.csv_row(self._now(), err.dxy, err.dyaw, err.sensor,
                                            lat, ok, self._pending_stop_skew) + '\n')
            self._csv.flush()
            self._pending_stop_skew = None

    def destroy_node(self):
        if self._csv:
            self._csv.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyncSupervisor()
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
