"""twin_mediator — the digital-twin integration node (fan-in / fan-out).

The single command chokepoint and state mirror of the digital twin. It:
  * subscribes the pre-safety command bus /dt/cmd_vel_raw (TwistStamped) and the LiDAR scans,
  * applies the 25 cm dual-LiDAR safety gate (lib.safety, fail-safe) + Burger limits ONCE,
  * fans the safe command out to /cmd_vel (the ACTIVE robot, TwistStamped) and — in `both` —
    /sim/cmd_vel (the mirroring sim),
  * mirrors pose/sensor/battery state onto /dt/* for the GUI and sync_supervisor,
  * owns the latched /dt/estop (auto-trips on critical battery; RESUME clears it) and /dt/mode.

Topic model: the active robot is always on bare topics
(/scan,/odom,/cmd_vel,/battery_state) — in sim_only that bare robot IS the Gazebo sim; in
real_only/both it is the real Burger. Only in `both` does the mirroring sim live on /sim/* and the
mediator gate both scans and fan out to both cmd_vels. The real logic lives in the pure lib
modules; this node is just the ROS wiring around them.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from algae_dt.lib import geometry, hud, safety, sync

INF = float('inf')


def _latched(depth: int = 1) -> QoSProfile:
    """Transient-local QoS so a late subscriber (the GUI) immediately gets the current value."""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TwinMediator(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('twin_mediator', **kwargs)

        # --- parameters (defaults mirror config/twin.yaml; the launch binds the file via /**) ---
        gp = self._declare
        self.mode = gp('mode', 'sim_only')
        self.stop_distance_m = gp('stop_distance_m', 0.25)
        self.front_sector_rad = gp('front_sector_rad', 0.70)
        self.range_min = gp('scan_range_min_m', 0.12)
        self.range_max = gp('scan_range_max_m', 3.5)
        self.max_linear = gp('max_linear_mps', 0.22)
        self.max_angular = gp('max_angular_radps', 2.0)
        self.cmd_rate_hz = gp('cmd_rate_hz', 20.0)
        self.health_rate_hz = gp('health_rate_hz', 1.0)
        self.max_cmd_age_s = gp('max_cmd_age_s', 0.5)
        self.max_data_age_s = gp('max_data_age_s', 1.0)
        self.battery_low_v = gp('battery_low_v', 11.0)
        self.battery_critical_v = gp('battery_critical_v', 10.5)
        self.batt_sim_start = gp('battery_sim_start_v', 12.5)
        self.batt_sim_drain = gp('battery_sim_drain_vps', 0.02)
        self.batt_sim_min = gp('battery_sim_min_v', 11.2)
        self.batt_empty_v = gp('battery_empty_v', 9.0)
        self.batt_full_v = gp('battery_full_v', 12.6)
        self.sync_source = gp('sim_only_sync_source', 'commanded')   # 'none' disables the shadow real_pose

        self._both = self.mode == 'both'
        self._sim_is_bare = self.mode == 'sim_only'      # the bare robot is the sim

        # --- state ---
        self._last_cmd: TwistStamped | None = None
        self._last_cmd_t = 0.0
        self._scan: LaserScan | None = None
        self._scan_t = 0.0
        self._sim_scan: LaserScan | None = None
        self._sim_scan_t = 0.0
        self._battery_v: float | None = None
        self._estop_manual = False
        self._estop_battery = False
        self._estop_latched = False
        self._shadow = (0.0, 0.0, 0.0)        # commanded shadow pose (sim_only /dt/real_pose), MAP frame
        self._shadow_t: float | None = None
        self._shadow_anchored = False         # shadow is anchored to the robot's first map pose
        self._T_map_odom = (0.0, 0.0, 0.0)    # map<-odom from AMCL/TF (identity until localized)
        self._start_t = self._now()

        # --- publishers ---
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_cmd_sim = self.create_publisher(TwistStamped, '/sim/cmd_vel', 10) if self._both else None
        self.pub_real_pose = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.pub_sim_pose = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.pub_scan_active = self.create_publisher(LaserScan, '/dt/scan_active', qos_profile_sensor_data)
        self.pub_odom_active = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.pub_health = self.create_publisher(BatteryState, '/dt/health', 10)
        self.pub_safety = self.create_publisher(Bool, '/dt/safety', _latched())
        self.pub_estop = self.create_publisher(Bool, '/dt/estop', _latched())
        self.pub_mode = self.create_publisher(String, '/dt/mode', _latched())

        # --- subscriptions ---
        self.create_subscription(TwistStamped, '/dt/cmd_vel_raw', self._on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(Bool, '/dt/estop_cmd', self._on_estop_cmd, _latched())
        if self._both:
            self.create_subscription(LaserScan, '/sim/scan', self._on_sim_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/sim/odom', self._on_sim_odom, 10)

        # --- TF: map<-odom (AMCL) so /dt/*_pose are published in the MAP frame the GUI overlays on ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- timers ---
        self.create_timer(1.0 / self.cmd_rate_hz, self._on_cmd_timer)
        self.create_timer(1.0 / self.health_rate_hz, self._on_health_timer)
        self.create_timer(0.2, self._refresh_map_odom)   # AMCL map<-odom updates slowly

        # latched initial state
        self.pub_mode.publish(String(data=self.mode))
        self._publish_estop()
        self.get_logger().info(
            f"twin_mediator up: mode={self.mode} stop={self.stop_distance_m} m "
            f"front_sector={self.front_sector_rad} rad (full width)")

    # ------------------------------------------------------------------ utils
    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _refresh_map_odom(self) -> None:
        """Cache the latest map<-odom (AMCL). Kept identity until localized / if no TF (e.g. tests)."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'odom', RclpyTime())
            tr, rot = t.transform.translation, t.transform.rotation
            self._T_map_odom = (tr.x, tr.y, geometry.yaw_from_quaternion(rot.z, rot.w))
        except TransformException:
            pass                                          # map<-odom not published yet (normal pre-AMCL)
        except Exception as exc:                          # anything else is a real fault -> surface it
            self.get_logger().warn(f"map<-odom lookup failed: {exc!r}", throttle_duration_sec=5.0)

    @staticmethod
    def _pose_xyyaw(p):
        o = p.orientation
        return (p.position.x, p.position.y, geometry.yaw_from_quaternion(o.z, o.w))

    def _map_posestamped(self, xyyaw) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self._stamp()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = float(xyyaw[0]), float(xyyaw[1])
        ps.pose.orientation.z, ps.pose.orientation.w = geometry.quaternion_from_yaw(xyyaw[2])
        return ps

    # -------------------------------------------------------------- callbacks
    def _on_cmd(self, msg: TwistStamped) -> None:
        self._last_cmd = msg
        self._last_cmd_t = self._now()

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_t = self._now()
        self.pub_scan_active.publish(msg)   # /scan is always the active robot's scan

    def _on_sim_scan(self, msg: LaserScan) -> None:
        self._sim_scan = msg
        self._sim_scan_t = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        self.pub_odom_active.publish(msg)                     # raw odom frame (sync/latency uses it)
        # Lift the odom pose into the MAP frame via map<-odom (AMCL) so the GUI overlays it on the
        # map exactly where RViz shows the robot. Identity until AMCL localizes (then == odom frame).
        map_xyyaw = geometry.compose_pose_2d(self._T_map_odom, self._pose_xyyaw(msg.pose.pose))
        ps = self._map_posestamped(map_xyyaw)
        # In sim_only the bare /odom is the SIM's; otherwise it is the REAL robot's.
        (self.pub_sim_pose if self._sim_is_bare else self.pub_real_pose).publish(ps)
        if self._sim_is_bare and not self._shadow_anchored:   # anchor the commanded shadow to the start
            self._shadow = map_xyyaw
            self._shadow_anchored = True

    def _on_sim_odom(self, msg: Odometry) -> None:
        # In `both`, lift the sim's odom pose into the MAP frame via the same map<-odom as the real
        # robot (AMCL), so /dt/sim_pose is directly comparable to /dt/real_pose (both MAP frame).
        # Publishing it raw (odom frame) would make the sync discrepancy carry the full map<-odom
        # offset instead of the true real-vs-sim divergence. The twin starts co-located and mirrors
        # the real robot 1:1, so the real robot's map<-odom is the right lift for the sim too.
        map_xyyaw = geometry.compose_pose_2d(self._T_map_odom, self._pose_xyyaw(msg.pose.pose))
        self.pub_sim_pose.publish(self._map_posestamped(map_xyyaw))

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_v = msg.voltage
        self._update_battery_estop(msg.voltage)
        if self.mode != 'sim_only':
            self.pub_health.publish(msg)

    def _on_estop_cmd(self, msg: Bool) -> None:
        self._estop_manual = bool(msg.data)
        self._refresh_estop()

    # --------------------------------------------------------------- E-STOP
    def _update_battery_estop(self, voltage: float) -> None:
        crit = voltage <= self.battery_critical_v
        if crit and not self._estop_battery:
            self.get_logger().warn(f"battery critical ({voltage:.2f} V) -> auto E-STOP")
        self._estop_battery = crit
        self._refresh_estop()

    def _refresh_estop(self) -> None:
        latched = self._estop_manual or self._estop_battery
        if latched != self._estop_latched:
            self._estop_latched = latched
            self._publish_estop()
            self.get_logger().warn(f"/dt/estop -> {latched}")

    def _publish_estop(self) -> None:
        self.pub_estop.publish(Bool(data=self._estop_latched))

    # --------------------------------------------------------------- gate loop
    def _scan_status(self, scan: LaserScan | None, t: float, now: float) -> safety.ScanStatus:
        if scan is None:
            return safety.ScanStatus(has_data=False, front_min=INF, age_s=0.0)
        fm = safety.front_min_range(scan.ranges, scan.angle_min, scan.angle_increment,
                                    self.front_sector_rad,
                                    scan.range_min or self.range_min,
                                    scan.range_max or self.range_max)
        return safety.ScanStatus(has_data=True, front_min=fm, age_s=now - t)

    def _on_cmd_timer(self) -> None:
        now = self._now()
        active = self._scan_status(self._scan, self._scan_t, now)
        mirror = (self._scan_status(self._sim_scan, self._sim_scan_t, now)
                  if self._both else safety.ScanStatus(False, INF, 0.0))
        blocked = safety.gate(active, mirror, self.stop_distance_m, self.max_data_age_s)

        if self._last_cmd is None or (now - self._last_cmd_t) > self.max_cmd_age_s:
            base_vx, base_wz = 0.0, 0.0   # watchdog: bus silent -> stop
        else:
            base_vx = self._last_cmd.twist.linear.x
            base_wz = self._last_cmd.twist.angular.z

        vx, wz = safety.limit_command(base_vx, base_wz, blocked=blocked, estop=self._estop_latched,
                                      max_linear=self.max_linear, max_angular=self.max_angular)

        out = TwistStamped()
        out.header.stamp = self._stamp()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = vx
        out.twist.angular.z = wz
        self.pub_cmd.publish(out)
        if self.pub_cmd_sim is not None:
            self.pub_cmd_sim.publish(out)

        self.pub_safety.publish(Bool(data=blocked))
        self._integrate_shadow(vx, wz, now)

    def _integrate_shadow(self, vx: float, wz: float, now: float) -> None:
        """Commanded shadow pose for sim_only: anchored to the robot's first MAP pose, then integrate
        the safe command. Published as the 'real' reference (/dt/real_pose, MAP frame) the
        sync_supervisor compares to the achieved sim pose — and the GUI overlays on the map."""
        if self.mode != 'sim_only' or self.sync_source != 'commanded' or not self._shadow_anchored:
            return
        if self._shadow_t is None:
            self._shadow_t = now
            return
        dt = now - self._shadow_t
        self._shadow_t = now
        if dt <= 0.0:
            return
        self._shadow = sync.integrate_unicycle(*self._shadow, vx, wz, dt)
        self.pub_real_pose.publish(self._map_posestamped(self._shadow))

    # --------------------------------------------------------------- battery
    def _on_health_timer(self) -> None:
        if self.mode == 'sim_only':
            elapsed = self._now() - self._start_t
            v = max(self.batt_sim_min, self.batt_sim_start - self.batt_sim_drain * elapsed)
            msg = BatteryState()
            msg.header.stamp = self._stamp()
            msg.voltage = float(v)
            msg.percentage = hud.battery_percentage(v, self.batt_empty_v, self.batt_full_v)
            msg.present = True
            self.pub_health.publish(msg)
            self._update_battery_estop(v)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinMediator()
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
