"""twin_mediator — the DT Integration Node (fan-in / fan-out). PLAN T1.4 + T2.3.

The single command chokepoint and state mirror of the digital twin. It:
  * subscribes the pre-safety command bus /dt/cmd_vel_raw (TwistStamped) and the LiDAR scans,
  * applies the 25 cm dual-LiDAR safety gate (lib.safety, fail-safe) + Burger limits ONCE,
  * fans the safe command out to /cmd_vel (the ACTIVE robot, TwistStamped) and — in `both` —
    /sim/cmd_vel (the mirroring sim),
  * mirrors pose/sensor/battery state onto /dt/* for the GUI and sync_supervisor,
  * owns the latched /dt/estop (auto-trips on critical battery; RESUME clears it) and /dt/mode.

Topic model (RULES §B-3 topic-collision rule): the ACTIVE robot is always on BARE topics
(/scan,/odom,/cmd_vel,/battery_state) — in sim_only that bare robot IS the Gazebo sim; in
real_only/both it is the real Burger. Only in `both` does the mirroring sim live on /sim/* and the
mediator gate both scans + fan out to both cmd_vels. All hard logic is in the unit-tested pure libs;
this node is the ROS wiring around them.
"""
from __future__ import annotations

import functools
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time as RclpyTime
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float64, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from algae_dt.lib import geometry, hud, safety, scanmerge, sync
from algae_dt.lib.ros_utils import declare_get

INF = float('inf')


def _latched(depth: int = 1) -> QoSProfile:
    """Transient-local QoS so a late subscriber (the GUI) immediately gets the current value."""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TwinMediator(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('twin_mediator', **kwargs)

        # --- parameters (defaults mirror config/twin.yaml; the launch binds the file via /**) ---
        gp = functools.partial(declare_get, self)
        self.mode = gp('mode', 'sim_only')
        self.stop_distance_m = gp('stop_distance_m', 0.25)
        self.front_sector_rad = gp('front_sector_rad', 0.70)
        self.range_min = gp('scan_range_min_m', 0.12)
        self.range_max = gp('scan_range_max_m', 3.5)
        self.max_linear = gp('max_linear_mps', 0.22)
        self.max_angular = gp('max_angular_radps', 2.84)
        self.cmd_rate_hz = gp('cmd_rate_hz', 20.0)
        self.health_rate_hz = gp('health_rate_hz', 1.0)
        self.max_cmd_age_s = gp('max_cmd_age_s', 0.5)
        self.max_data_age_s = gp('max_data_age_s', 1.0)
        self.sim_max_data_age_s = gp('sim_max_data_age_s', 2.0)   # looser budget for the MIRROR sim scan
        self.battery_low_v = gp('battery_low_v', 11.0)
        self.battery_critical_v = gp('battery_critical_v', 10.5)
        self.batt_sim_start = gp('battery_sim_start_v', 12.5)
        self.batt_sim_drain = gp('battery_sim_drain_vps', 0.02)
        self.batt_sim_min = gp('battery_sim_min_v', 11.2)
        self.batt_empty_v = gp('battery_empty_v', 9.0)
        self.batt_full_v = gp('battery_full_v', 12.6)
        self.sync_source = gp('sim_only_sync_source', 'commanded')   # 'none' disables the shadow real_pose
        self.sim_entity = gp('sim_entity_name', 'burger_sim')        # gz entity of the `both` mirror
        self.stop_release_m = gp('stop_release_m', 0.35)             # latch hysteresis (lib.safety.BlockLatch)
        self.stop_release_hold_s = gp('stop_release_hold_s', 0.3)
        self.rtf_comp_enable = gp('rtf_comp_enable', True)           # both: 1/RTF sim fan-out scaling
        self.rtf_comp_max = gp('rtf_comp_max', 3.0)
        self.rtf_window_s = gp('rtf_window_s', 3.0)

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
        self._battery_override_v: float | None = None   # sim_only demo override (None = off)
        self._estop_manual = False
        self._estop_battery = False                     # LATCHED on critical; cleared only by RESUME
        self._estop_latched = False
        # Fail-safe across a mediator RESTART: in real modes the battery latch lives only in THIS
        # process, so a fresh __init__ would publish /dt/estop=False and silently un-latch a prior
        # battery (auto) E-STOP while the pack is still critical (mission_runner/GUI would treat it as
        # RESUMED with no operator action). Start HELD until the first valid /battery_state proves the
        # pack healthy. sim_only's battery is synthetic + mediator-owned (resets with the demo), so it
        # keeps its immediate-clear startup.
        self._estop_startup_hold = (self.mode != 'sim_only')
        # Mirror-scan trust (both only): the sim scan may veto the REAL robot's forward motion
        # ONLY while the twin is IN SYNC. A diverged mirror measures the wrong place — in the lab
        # it grazed gz geometry the real robot had cleared and PHANTOM-BRAKED the real robot in
        # bursts (DWB then replanned around stops that didn't exist → stuttering, weaving,
        # "doesn't follow its own path"). In tolerance, the mirror stands where the real robot
        # stands, so its virtual-obstacle veto is meaningful (the pillar-③ box demo still stops
        # BOTH). Out of tolerance, the veto is misinformation and the auto-resync is already
        # correcting the divergence. The REAL robot's own LiDAR always gates regardless.
        self._sync_ok = True                  # default trust until the supervisor says otherwise
        self._shadow = (0.0, 0.0, 0.0)        # commanded shadow pose (sim_only /dt/real_pose), MAP frame
        self._shadow_t: float | None = None
        self._shadow_anchored = False         # shadow is anchored to the robot's first map pose
        self._T_map_odom = (0.0, 0.0, 0.0)    # map<-odom from AMCL/TF (identity until localized)
        self._start_t = self._now()
        # Sticky forward-stop (lab 2026-06 'the safety stop doesn't work'): once blocked, forward
        # stays cut until the front range clears stop_release_m for stop_release_hold_s — rotation
        # can no longer flicker the ±20° cone empty and lurch-creep past the obstacle. Raises on a
        # release distance below the stop distance (a silent fix would hide a safety config error).
        self._latch = safety.BlockLatch(self.stop_distance_m, self.stop_release_m,
                                        self.stop_release_hold_s)
        self._clock_samples: deque = deque()  # (wall_s, sim_s) window for the both-mode RTF estimate

        # --- publishers ---
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_cmd_sim = self.create_publisher(TwistStamped, '/sim/cmd_vel', 10) if self._both else None
        self.pub_real_pose = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.pub_sim_pose = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.pub_scan_active = self.create_publisher(LaserScan, '/dt/scan_active', qos_profile_sensor_data)
        # The scan Nav2's OBSTACLE sources consume (costmap layers + collision_monitor; the launch
        # rewrites their `topic` keys here). Real scan verbatim — except in `both` with a trusted
        # mirror, where the mirror's returns are overlaid so virtual obstacles get PLANNED around.
        self.pub_scan_nav = self.create_publisher(LaserScan, '/dt/scan_nav', qos_profile_sensor_data)
        self.pub_odom_active = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.pub_health = self.create_publisher(BatteryState, '/dt/health', 10)
        self.pub_safety = self.create_publisher(Bool, '/dt/safety', _latched())
        self.pub_estop = self.create_publisher(Bool, '/dt/estop', _latched())
        self.pub_mode = self.create_publisher(String, '/dt/mode', _latched())
        # Latched localization flag: False until the map<-odom transform (AMCL) first resolves.
        # Consumers that act on the MAP-frame /dt/real_pose (twin_resync's teleport target!) must
        # hold off until then — pre-seed the pose is identity-lifted odom with no map meaning, and
        # an auto-resync 5 s after launch could teleport the mirror into a wall, whose scan then
        # BLOCKS THE REAL ROBOT through the dual-LiDAR gate. sim_only / both+fake auto-seed AMCL,
        # so this flips True seconds after startup; real `both` flips on the 2D Pose Estimate.
        self.pub_localized = self.create_publisher(Bool, '/dt/localized', _latched())

        # --- subscriptions ---
        self.create_subscription(TwistStamped, '/dt/cmd_vel_raw', self._on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(Bool, '/dt/estop_cmd', self._on_estop_cmd, _latched())
        # Demo override for the sim_only synthetic battery (the natural drain floors at
        # battery_sim_min_v, ABOVE critical, so the auto-E-STOP beat needs a forced voltage):
        #   ros2 topic pub --once /dt/battery_override_v std_msgs/msg/Float64 "{data: 10.0}"
        # <= 0.0 clears the override. Ignored outside sim_only (the real battery is real).
        self.create_subscription(Float64, '/dt/battery_override_v', self._on_battery_override, 10)
        if self._both:
            self.create_subscription(LaserScan, '/sim/scan', self._on_sim_scan, qos_profile_sensor_data)
            self.create_subscription(TFMessage, '/sim/ground_truth', self._on_sim_ground_truth, 10)
            self.create_subscription(Bool, '/dt/sync_ok', self._on_sync_ok, _latched())
            if self.rtf_comp_enable:
                # Raw /clock tap (the node itself stays on wall time in `both`): RTF = sim-vs-wall
                # progress drives the mirror fan-out compensation. BEST_EFFORT matches any clock QoS.
                self.create_subscription(
                    Clock, '/clock', self._on_clock,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # --- TF: map<-odom (AMCL) so /dt/*_pose are published in the MAP frame the GUI overlays on ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- timers ---
        self.create_timer(1.0 / self.cmd_rate_hz, self._on_cmd_timer)
        self.create_timer(1.0 / self.health_rate_hz, self._on_health_timer)
        self.create_timer(0.2, self._refresh_map_odom)   # AMCL map<-odom updates slowly

        # latched initial state. In real modes start E-STOP HELD (fail-safe across a mediator
        # restart, see _estop_startup_hold); sim_only starts clear exactly as before.
        self._localized = False
        self.pub_localized.publish(Bool(data=False))
        self.pub_mode.publish(String(data=self.mode))
        self._estop_latched = self._estop_manual or self._estop_battery or self._estop_startup_hold
        self._publish_estop()
        self.get_logger().info(
            f"twin_mediator up: mode={self.mode} stop={self.stop_distance_m} m "
            f"(release {self.stop_release_m} m sustained {self.stop_release_hold_s}s) "
            f"front_sector={self.front_sector_rad} rad (full width)")

    # ------------------------------------------------------------------ utils
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
            if not self._localized:                       # first successful lookup = AMCL is up
                self._localized = True
                self.pub_localized.publish(Bool(data=True))
                self.get_logger().info(
                    "map<-odom resolved: /dt/*_pose are MAP-localized (twin resync armed)")
        except TransformException:
            pass                                          # map<-odom not published yet (normal pre-AMCL)
        except Exception as exc:                          # anything else is a real fault -> surface it
            self.get_logger().warn(f"map<-odom lookup failed: {exc!r}", throttle_duration_sec=5.0)

    @staticmethod
    def _pose_xyyaw(p):
        return geometry.pose_xyyaw(p)

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
        self.pub_scan_nav.publish(self._nav_scan(msg))

    def _nav_scan(self, msg: LaserScan) -> LaserScan:
        """/dt/scan_nav — what Nav2's obstacle sources see (USER GAP, lab 2026-06: 'placing things
        in the twin doesn't reflect on the robot's nav'). In `both` with a TRUSTED, fresh mirror,
        overlay the mirror's returns onto the real scan by angle (lib.scanmerge) so a virtual
        obstacle enters the REAL costmaps and Nav2 plans around it — the same trust rule as the
        gate (sync_ok), so a diverged mirror can no more poison the costmap than veto motion.
        AMCL keeps the bare /scan (its `scan_topic` key is untouched by the launch rewrite):
        localization must never see virtual returns. Everywhere else: verbatim pass-through."""
        if not (self._both and self._sync_ok and self._sim_scan is not None
                and (self._now() - self._sim_scan_t) <= self.sim_max_data_age_s):
            return msg
        sim = self._sim_scan
        merged = LaserScan()
        merged.header = msg.header
        merged.angle_min = msg.angle_min
        merged.angle_max = msg.angle_max
        merged.angle_increment = msg.angle_increment
        merged.time_increment = msg.time_increment
        merged.scan_time = msg.scan_time
        merged.range_min = msg.range_min
        merged.range_max = msg.range_max
        merged.ranges = scanmerge.merge_ranges(
            msg.ranges, msg.angle_min, msg.angle_increment,
            sim.ranges, sim.angle_min, sim.angle_increment,
            sim_range_min=max(sim.range_min, self.range_min) if sim.range_min > 0.0 else self.range_min,
            sim_range_max=min(sim.range_max, self.range_max) if sim.range_max > 0.0 else self.range_max,
            real_range_min=msg.range_min,
            real_range_max=msg.range_max if msg.range_max > 0.0 else self.range_max)
        return merged

    def _on_clock(self, msg: Clock) -> None:
        wall = time.monotonic()
        self._clock_samples.append((wall, msg.clock.sec + msg.clock.nanosec * 1e-9))
        while self._clock_samples and (wall - self._clock_samples[0][0]) > self.rtf_window_s:
            self._clock_samples.popleft()

    def _sim_fanout_factor(self) -> float:
        """clamp(1/RTF) for the `both` mirror fan-out (lib.sync): gz integrates /sim/cmd_vel in
        SIM time, so at RTF<1 the open-loop mirror under-travels per wall second and turns in the
        wrong places ('the twin doesn't take the path the real one takes', lab 2026-06). 1.0
        whenever RTF is unknown — and ~1.0 on a healthy lab GPU, where this is a no-op."""
        if not self._clock_samples:
            return 1.0
        return sync.rtf_compensation(sync.rtf_estimate(self._clock_samples), self.rtf_comp_max)

    def _on_sim_scan(self, msg: LaserScan) -> None:
        self._sim_scan = msg
        self._sim_scan_t = self._now()

    def _on_sync_ok(self, msg: Bool) -> None:
        if self._sync_ok and not msg.data:
            self.get_logger().info(
                "twin out of sync: the mirror's scan no longer gates the real robot "
                "(it measures the wrong place) until the twin re-syncs",
                throttle_duration_sec=5.0)
        self._sync_ok = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        # NOTE: /dt/odom_active is a raw pass-through and stays in the ODOM frame (sync_supervisor
        # only reads its twist + arrival time; mission_runner only its yaw for the spin count).
        # Anything needing a MAP-frame pose must use /dt/real_pose | /dt/sim_pose instead.
        self.pub_odom_active.publish(msg)
        # Lift the odom pose into the MAP frame via map<-odom (AMCL) so the GUI overlays it on the
        # map exactly where RViz shows the robot. Identity until AMCL localizes (then == odom frame).
        map_xyyaw = geometry.compose_pose_2d(self._T_map_odom, self._pose_xyyaw(msg.pose.pose))
        ps = self._map_posestamped(map_xyyaw)
        # In sim_only the bare /odom is the SIM's; otherwise it is the REAL robot's.
        (self.pub_sim_pose if self._sim_is_bare else self.pub_real_pose).publish(ps)
        if self._sim_is_bare and not self._shadow_anchored:   # anchor the commanded shadow to the start
            self._shadow = map_xyyaw
            self._shadow_anchored = True

    def _on_sim_ground_truth(self, msg: TFMessage) -> None:
        # In `both`, /dt/sim_pose is the gz GROUND-TRUTH model pose (sim_bridge.yaml bridges
        # /world/<w>/dynamic_pose/info as Pose_V -> TFMessage so entity NAMES survive), published
        # VERBATIM: the gz world frame == the map frame by construction (the arena world is built
        # to the course map — the same contract sim_only AMCL auto-seeding relies on). Two reasons
        # this replaced the old /sim/odom + map<-odom lift:
        #   * ground truth carries no odom integration error, so /dt/sync_error measures the TRUE
        #     real-vs-sim divergence (the old lift also borrowed the REAL robot's AMCL correction,
        #     silently crediting the sim with it);
        #   * it makes a twin_resync set_pose teleport REAL — pose, LiDAR viewpoint and the
        #     measured error move together (a teleport never lands in integrated odom, so under
        #     the old source the number could not be corrected, only restarted).
        for t in msg.transforms:
            if t.child_frame_id == self.sim_entity:
                tr, rot = t.transform.translation, t.transform.rotation
                self.pub_sim_pose.publish(self._map_posestamped(
                    (tr.x, tr.y, geometry.yaw_from_quaternion(rot.z, rot.w))))
                return

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_v = msg.voltage
        self._update_battery_estop(msg.voltage)
        if self.mode != 'sim_only':
            self.pub_health.publish(msg)

    def _on_battery_override(self, msg: Float64) -> None:
        if self.mode != 'sim_only':
            self.get_logger().warn("battery override ignored outside sim_only (real battery is real)")
            return
        self._battery_override_v = float(msg.data) if msg.data > 0.0 else None
        self.get_logger().warn(f"sim battery override -> {self._battery_override_v}")

    def _on_estop_cmd(self, msg: Bool) -> None:
        self._estop_manual = bool(msg.data)
        if not msg.data:
            # Operator RESUME clears EVERY latched cause, including the battery trip. Without this
            # the battery latch could never be released; with auto-relatching below, a still-critical
            # battery re-trips on the next sample, so RESUME cannot bypass a genuinely dead battery.
            self._estop_battery = False
        self._refresh_estop()

    # --------------------------------------------------------------- E-STOP
    def _update_battery_estop(self, voltage: float) -> None:
        # Ignore invalid frames: the real OpenCR emits voltage=0.0 / NaN at bringup or on a serial
        # hiccup; without this guard a single 0.0 frame (<= critical) would LATCH a phantom battery
        # E-STOP on a healthy pack, requiring a manual RESUME. A real pack is never 0 V.
        if not (math.isfinite(voltage) and voltage > 0.0):
            return
        # First valid healthy sample after (re)start releases the fail-safe startup hold.
        if self._estop_startup_hold and voltage > self.battery_critical_v:
            self._estop_startup_hold = False
        # LATCH on critical (RULES: 'latched ... RESUME clears it'). A sagging LiPo bounces above
        # and below the threshold under load; assigning `crit` each sample silently un-latched the
        # E-STOP and re-enabled motion with no operator action. Only RESUME (estop_cmd False)
        # clears the trip now.
        if voltage <= self.battery_critical_v and not self._estop_battery:
            self.get_logger().warn(f"battery critical ({voltage:.2f} V) -> auto E-STOP (latched)")
            self._estop_battery = True
        self._refresh_estop()

    def _refresh_estop(self) -> None:
        latched = self._estop_manual or self._estop_battery or self._estop_startup_hold
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
        fm = safety.front_min_from_scan(scan, self.front_sector_rad, self.range_min, self.range_max)
        return safety.ScanStatus(has_data=True, front_min=fm, age_s=now - t)

    def _on_cmd_timer(self) -> None:
        now = self._now()
        active = self._scan_status(self._scan, self._scan_t, now)
        # The mirror participates in the gate only while TRUSTED (in sync): see __init__. An
        # untrusted mirror is excluded exactly like the no-mirror modes — its staleness must not
        # fail-safe-block either (no data ≠ danger when the data describes the wrong place).
        mirror = (self._scan_status(self._sim_scan, self._sim_scan_t, now)
                  if self._both and self._sync_ok else safety.ScanStatus(False, INF, 0.0))
        # Sticky stop: the latch consumes the combined considered range (same fail-safe gating as
        # gate()) and releases only after a sustained clearance — see lib.safety.BlockLatch.
        blocked = self._latch.update(
            safety.combined_range(active, mirror, self.max_data_age_s,
                                  sim_max_data_age_s=self.sim_max_data_age_s), now)

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
            f = self._sim_fanout_factor()
            if f == 1.0:
                self.pub_cmd_sim.publish(out)
            else:
                # Separate message on purpose: the REAL command must never carry the sim scaling.
                sim_out = TwistStamped()
                sim_out.header.stamp = out.header.stamp
                sim_out.header.frame_id = out.header.frame_id
                sim_out.twist.linear.x = vx * f
                sim_out.twist.angular.z = wz * f
                self.pub_cmd_sim.publish(sim_out)

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
            if self._battery_override_v is not None:     # demo: force the auto-E-STOP beat
                v = self._battery_override_v
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
