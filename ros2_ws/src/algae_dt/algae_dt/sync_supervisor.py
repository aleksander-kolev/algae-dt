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

CLOCK DISCIPLINE (the one rule that keeps every number meaningful): all latency and stop-skew
times are taken from THIS node's clock (`_now()`) at message ARRIVAL — never from header stamps.
Header stamps cross clock domains: in `both` the real Burger stamps odom with the Pi's wall clock
(~1.7e9 s) while the gz bridge stamps /sim/odom with SIM time (~tens of seconds), and the Pi and
laptop wall clocks are not NTP-synced over lab Wi-Fi. Subtracting stamps across those domains
produced astronomical stop-skews and offset-polluted latencies. One observer clock = one time base.

ALERT DISCIPLINE: alerts are edge-triggered (fire when a condition BECOMES true) and re-fire at
most every `alert_repeat_s` while it stays true — a single bad sample can no longer flood
/dt/alerts at the 5 Hz tick rate. A latency sample is consumed ONE-SHOT: it is alerted/logged on
the tick it arrives and never re-judged afterwards.
"""
from __future__ import annotations

import functools
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
from algae_dt.lib.ros_utils import declare_get

INF = float('inf')


def _latched() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SyncSupervisor(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('sync_supervisor', **kwargs)
        gp = functools.partial(declare_get, self)
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
        self.csv_max_rows = int(gp('sync_log_max_rows', 36000))   # ~2 h at 5 Hz, then stop (bounded evidence)
        self.stop_skew_budget_ms = gp('stop_skew_ms', 300.0)
        self.pose_timeout_s = gp('sync_pose_timeout_s', 2.0)
        self.alert_repeat_s = gp('alert_repeat_s', 5.0)

        self._both = self.mode == 'both'
        # Which pose streams MUST be alive per mode: real_only has no sim world at all (the mediator
        # never publishes /dt/sim_pose there), so requiring it produced a permanent 5 Hz
        # stale-stream alert storm + an empty CSV for the whole lab session. sim_only publishes
        # BOTH (sim pose + commanded shadow as the 'real' reference), `both` publishes both.
        self._require_sim_stream = self.mode in ('sim_only', 'both')

        # state
        self._real_pose = None          # (x, y, yaw)
        self._sim_pose = None
        self._active_front = INF
        self._sim_front = INF
        self._latency_ms = float('nan')   # last measured value (kept for the GUI banner)
        self._latency_fresh = False       # one-shot: alert/log a sample exactly once
        self._cmd_moving = False
        self._t_cmd = None
        self._awaiting_motion = False
        self._real_moving = False
        self._sim_moving = False
        self._t_real_stop = None
        self._t_sim_stop = None
        self._safety_blocked = False
        self._pending_stop_skew = None
        self._t_real_pose = None
        self._t_sim_pose = None
        self._t_start = self._now()
        self._csv_rows = 0
        self._alert_last: dict[str, float] = {}   # alert key -> last publish time (edge+repeat gating)
        self._alert_active: dict[str, bool] = {}  # alert key -> condition was true last tick

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
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _xyyaw(ps: PoseStamped):
        return geometry.pose_xyyaw(ps.pose)

    def _front(self, scan: LaserScan) -> float:
        return safety.front_min_from_scan(scan, self.front_sector_rad,
                                          self.range_min, self.range_max)

    def _open_csv(self):
        if not self.csv_enable:
            return None
        os.makedirs(self.csv_dir, exist_ok=True)
        path = os.path.join(self.csv_dir, f"{self.csv_prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        f = open(path, 'a', encoding='utf-8')
        f.write(metrics.csv_header() + '\n')
        f.flush()
        return f

    def _alert(self, key: str, active: bool, text: str) -> None:
        """Edge-triggered + repeat-throttled /dt/alerts publish: fire when `active` BECOMES true,
        then at most every alert_repeat_s while it stays true. Resets on the falling edge so the
        next occurrence alerts immediately. (Was: every over-tolerance tick re-published at 5 Hz.)"""
        was = self._alert_active.get(key, False)
        self._alert_active[key] = active
        if not active:
            return
        now = self._now()
        last = self._alert_last.get(key)
        if (not was) or last is None or (now - last) >= self.alert_repeat_s:
            self.pub_alerts.publish(String(data=text))
            self._alert_last[key] = now

    # -------------------------------------------------------------- callbacks
    def _on_real_pose(self, msg):
        self._real_pose = self._xyyaw(msg)
        self._t_real_pose = self._now()

    def _on_sim_pose(self, msg):
        self._sim_pose = self._xyyaw(msg)
        self._t_sim_pose = self._now()

    def _on_active_scan(self, msg): self._active_front = self._front(msg)
    def _on_sim_scan(self, msg): self._sim_front = self._front(msg)

    def _on_cmd(self, msg: TwistStamped) -> None:
        moving = (abs(msg.twist.linear.x) > self.motion_eps_mps
                  or abs(msg.twist.angular.z) > self.motion_eps_radps)
        if moving and not self._cmd_moving:
            # ARRIVAL time on OUR clock (see module docstring): the matching motion-onset time below
            # is taken from the same clock, so the difference is a real one-clock latency.
            self._t_cmd = self._now()
            self._awaiting_motion = True
        self._cmd_moving = moving

    def _on_active_odom(self, msg: Odometry) -> None:
        moving = self._odom_moving(msg)
        now = self._now()
        if self._awaiting_motion and moving and self._t_cmd is not None:
            self._latency_ms = max(0.0, metrics.latency_ms(self._t_cmd, now))
            self._latency_fresh = True
            self.pub_latency.publish(Float64(data=self._latency_ms))
            self._awaiting_motion = False
        if self._real_moving and not moving:
            self._t_real_stop = now
        self._real_moving = moving

    def _on_sim_odom(self, msg: Odometry) -> None:
        moving = self._odom_moving(msg)
        if self._sim_moving and not moving:
            # SAME observer clock as _t_real_stop. The header stamp here is gz SIM time while the
            # real odom's stamp is the Pi's wall clock — differencing those two domains made every
            # both-mode stop_skew astronomically large (the pillar-③ 'synchronous stop' number).
            self._t_sim_stop = self._now()
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

    def _stream_ok(self) -> bool:
        """True iff every REQUIRED pose stream has reported within pose_timeout_s (fail-loud, F5).
        real_only requires only the real stream — there is no sim world to wait for."""
        now = self._now()
        required = [self._t_real_pose]
        if self._require_sim_stream:
            required.append(self._t_sim_pose)
        for t in required:
            if t is None or (now - t) > self.pose_timeout_s:
                return False
        return True

    # --------------------------------------------------------------- sync tick
    def _consume_stop_skew(self) -> float | None:
        """Alert on + hand back the pending safety-stop skew (one-shot). Evaluated even when a pose
        stream is stale — the skew was already captured and does not depend on pose freshness;
        early-returning past it silently lost the evidence for that safety event."""
        skew = self._pending_stop_skew
        self._pending_stop_skew = None
        self._alert('stop_skew',
                    skew is not None and abs(skew) > self.stop_skew_budget_ms,
                    f"STOP-SKEW {0.0 if skew is None else skew:.0f} ms > budget "
                    f"{self.stop_skew_budget_ms:.0f} ms")
        return skew

    def _on_sync_timer(self) -> None:
        if not self._stream_ok():
            skew = self._consume_stop_skew()      # don't lose a captured safety-event skew
            if skew is not None:
                self.get_logger().warn(f"stop_skew {skew:.0f} ms captured during a pose-stream gap "
                                       "(alerted; not logged to CSV — no sync row without poses)")
            # Startup: stay quiet until the timeout elapses. After that a missing/stale REQUIRED
            # stream IS a desync -> fail loud (sync_ok False + alert), never silently skip (F5).
            if self._now() - self._t_start > self.pose_timeout_s:
                self.pub_ok.publish(Bool(data=False))
                self._alert('stream', True,
                            "SYNC pose stream stale/absent (a world is not reporting)")
            return
        self._alert_active['stream'] = False      # falling edge: next stream loss alerts immediately
        sim_front = self._sim_front if self._both else self._active_front   # 0 delta if single world
        # real_only has NO sim world: the pose discrepancy degenerates to 0 by comparing the real
        # pose against itself (latency + the CSV evidence still measure normally there).
        sim_pose = self._sim_pose if self._require_sim_stream else self._real_pose
        err = sync.compute(self._real_pose, sim_pose, self._active_front, sim_front)
        self.pub_err.publish(Vector3(x=err.dxy, y=err.dyaw, z=err.sensor))

        ok = sync.within_tolerance(err, self.tol_xy, self.tol_yaw, self.tol_sensor)
        self.pub_ok.publish(Bool(data=ok))
        self._alert('sync', not ok,
                    f"SYNC OUT-OF-TOLERANCE dxy={err.dxy:.3f}/{self.tol_xy} "
                    f"dyaw={err.dyaw:.3f}/{self.tol_yaw} sensor={err.sensor:.3f}/{self.tol_sensor}")

        # Latency: judge each measured sample exactly once (one-shot). Re-checking the same stale
        # value every tick flooded /dt/alerts and froze a long-gone number into every CSV row.
        fresh_latency = self._latency_ms if self._latency_fresh else float('nan')
        self._alert('latency',
                    self._latency_fresh and self._latency_ms > self.latency_budget_ms,
                    f"LATENCY {self._latency_ms:.0f} ms > budget {self.latency_budget_ms:.0f} ms")
        self._latency_fresh = False

        skew = self._consume_stop_skew()

        if self._csv:
            if self._csv_rows < self.csv_max_rows:
                self._csv.write(metrics.csv_row(self._now(), err.dxy, err.dyaw, err.sensor,
                                                fresh_latency, ok, skew) + '\n')
                self._csv.flush()
                self._csv_rows += 1
                if self._csv_rows == self.csv_max_rows:
                    self.get_logger().warn(
                        f"sync CSV reached sync_log_max_rows={self.csv_max_rows}; logging stopped "
                        "(restart the node for a fresh file)")

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
