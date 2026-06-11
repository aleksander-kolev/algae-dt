"""twin_resync — bounded-drift correction for the `both`-mode twin (Rubric ②). PLAN T7.1.

"Predict with the model, correct with the data": the sim mirrors the gated commands open-loop
(twin_mediator fan-out), so its pose error vs the real Burger ACCUMULATES — measured, alerted and
CSV-logged by sync_supervisor, but otherwise unbounded. This node closes the loop: it snaps the
sim entity back onto the real robot's pose via gz set_pose — on the operator's RESYNC click
(/dt/resync_cmd, std_msgs/Empty) any time, or AUTOMATICALLY once /dt/sync_error has stayed out of
the documented twin.yaml pose tolerances for resync_sustain_s. All policy rules (sustain, cooldown
spacing of attempts, stale-input refusal) live in the unit-tested lib.resync; this node is the ROS
wiring + the gz call (shared lib.gzcli).

Every EXECUTED resync publishes one /dt/resync_event (String, "<trigger> dxy=… dyaw=… -> sim
snapped to (x, y, yaw)") — sync_supervisor stamps it into the evidence CSV's `resync` column and
the GUI shows it on the SYNC banner. A FAILED gz call publishes /dt/alerts instead (fail-loud,
RULES §C) and is retried no sooner than the cooldown.

The teleport target is /dt/real_pose (map frame) passed to set_pose VERBATIM: the gz world frame
== the map frame by construction (worlds/algae_arena.world is built to maps/map.yaml with a shared
origin — the same contract sim_only AMCL auto-seeding relies on). The target needs no occupancy
projection: it is where the PHYSICAL robot actually stands, which is free space on the shared map
by definition. The teleport is GENUINE because /dt/sim_pose in `both` is gz GROUND TRUTH
(mediator ← /sim/ground_truth, see sim_bridge.yaml): pose, LiDAR viewpoint and the measured sync
error all move together. (Under the old /sim/odom-derived pose a teleport moved the robot but not
the number — the worst kind of fake fix.)

Only meaningful in mode:=both (the launch starts it there). In other modes the node idles by
design: sim_only has no second world to correct, real_only has no sim at all.
"""
from __future__ import annotations

import functools

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from algae_dt.lib import geometry, gzcli, resync
from algae_dt.lib.ros_utils import declare_get

NAN = float('nan')


def _latched() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TwinResync(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('twin_resync', **kwargs)
        gp = functools.partial(declare_get, self)
        self.mode = gp('mode', 'sim_only')
        # fail fast on names that would break the gz proto-text request (RULES §C)
        self.world = gzcli.safe_token(gp('world', 'default'), 'world')
        self.entity = gzcli.safe_token(gp('sim_entity_name', 'burger_sim'), 'sim_entity_name')
        self.tol_xy = gp('tol_pose_xy_m', 0.15)
        self.tol_yaw = gp('tol_pose_yaw_rad', 0.26)
        self.sustain_s = gp('resync_sustain_s', 5.0)
        self.cooldown_s = gp('resync_cooldown_s', 8.0)
        self.auto_enable = gp('resync_auto_enable', True)
        self.tick_hz = gp('resync_tick_hz', 2.0)
        self.z_m = gp('resync_z_m', 0.01)                       # = the launch's sim spawn z
        self.set_pose_timeout_ms = int(gp('set_pose_timeout_ms', 500))
        self.input_max_age_s = gp('sync_pose_timeout_s', 2.0)   # shared freshness budget

        # latest inputs (the policy itself is pure + immutable, lib.resync)
        self._state = resync.ResyncState()
        self._pending_manual = False
        self._err = (NAN, NAN)            # (dxy, dyaw) from /dt/sync_error
        self._t_err: float | None = None
        self._real: tuple[float, float, float] | None = None
        self._t_real: float | None = None
        # /dt/real_pose is the TELEPORT TARGET, and it only has map meaning once AMCL resolved
        # map<-odom (the mediator's latched /dt/localized). Pre-seed it is identity-lifted odom:
        # firing then could snap the mirror into a wall, whose scan would then BLOCK THE REAL
        # ROBOT through the dual-LiDAR gate. Until localized, every fire (manual included) is
        # refused — a click queues and executes once the 2D Pose Estimate lands.
        self._localized = False

        self.pub_event = self.create_publisher(String, '/dt/resync_event', 10)
        self.pub_alerts = self.create_publisher(String, '/dt/alerts', 10)

        # Deliberately NOT latched: a queued teleport replayed at a later (re)start would be an
        # unexpected robot move — the operator clicks again instead.
        self.create_subscription(Empty, '/dt/resync_cmd', self._on_resync_cmd, 10)
        self.create_subscription(Vector3, '/dt/sync_error', self._on_sync_error, 10)
        self.create_subscription(PoseStamped, '/dt/real_pose', self._on_real_pose, 10)
        self.create_subscription(Bool, '/dt/localized', self._on_localized, _latched())

        if self.mode == 'both':
            self.create_timer(1.0 / self.tick_hz, self._on_tick)
            self.get_logger().info(
                f"twin_resync up: world={self.world} entity={self.entity} "
                f"tol xy={self.tol_xy} yaw={self.tol_yaw} sustain={self.sustain_s}s "
                f"cooldown={self.cooldown_s}s auto={'on' if self.auto_enable else 'OFF'}")
        else:
            self.get_logger().info(
                f"twin_resync idle: mode={self.mode} has no mirror sim to correct "
                "(active only in `both`)")

    # ------------------------------------------------------------------ utils
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------- callbacks
    def _on_sync_error(self, msg: Vector3) -> None:
        self._err = (msg.x, msg.y)        # sync_supervisor publishes x=dxy, y=dyaw
        self._t_err = self._now()

    def _on_real_pose(self, msg: PoseStamped) -> None:
        self._real = geometry.pose_xyyaw(msg.pose)
        self._t_real = self._now()

    def _on_localized(self, msg: Bool) -> None:
        if msg.data and not self._localized:
            self.get_logger().info("AMCL localized: resync armed")
        self._localized = bool(msg.data)

    def _on_resync_cmd(self, _msg: Empty) -> None:
        if self.mode != 'both':
            self.get_logger().warn(
                f"RESYNC ignored: mode={self.mode} has no mirror sim to correct")
            return
        # Latch the intent: a click during the cooldown / a stream gap fires as soon as the
        # policy allows instead of being silently dropped.
        self._pending_manual = True
        if not self._localized:
            self.get_logger().warn(
                "RESYNC queued: AMCL is not localized yet (RViz 2D Pose Estimate) — the real "
                "pose has no map meaning, so the snap waits for localization")
        else:
            self.get_logger().info("operator RESYNC requested")

    # --------------------------------------------------------------- the loop
    def _inputs_fresh(self, now: float) -> bool:
        """Trustworthy-to-fire: fresh target + fresh error + a MAP-localized pose (AMCL up)."""
        return (self._localized
                and self._t_real is not None and (now - self._t_real) <= self.input_max_age_s
                and self._t_err is not None and (now - self._t_err) <= self.input_max_age_s)

    def _on_tick(self) -> None:
        now = self._now()
        self._state, trigger = resync.step(
            self._state, now=now, dxy=self._err[0], dyaw=self._err[1],
            tol_xy=self.tol_xy, tol_yaw=self.tol_yaw,
            sustain_s=self.sustain_s, cooldown_s=self.cooldown_s,
            auto_enable=self.auto_enable, manual_requested=self._pending_manual,
            inputs_fresh=self._inputs_fresh(now))
        if trigger is None:
            return
        self._pending_manual = False      # any executed resync satisfies a queued click
        self._execute(trigger)

    def _execute(self, trigger: str) -> None:
        x, y, yaw = self._real            # fresh by policy contract (inputs_fresh gated the fire)
        dxy, dyaw = self._err
        try:
            req = gzcli.pose_req(self.entity, x, y, self.z_m, yaw=yaw)
        except ValueError as exc:         # non-finite pose: refuse loudly, never send garbage
            self._fail(trigger, f"invalid target pose: {exc}")
            return
        ok, diag = self._set_entity_pose(req)
        if not ok:
            self._fail(trigger, f"gz set_pose: {diag}")
            return
        text = (f"{trigger} dxy={dxy:.2f} dyaw={dyaw:.2f} -> "
                f"sim snapped to ({x:.2f}, {y:.2f}, {yaw:.2f})")
        self.pub_event.publish(String(data=text))
        self.get_logger().info(f"RESYNC {text}")

    def _set_entity_pose(self, req: str) -> tuple[bool, str]:
        """The one gz side effect, kept overridable for the in-process tests."""
        return gzcli.call_boolean(f'/world/{self.world}/set_pose', 'gz.msgs.Pose',
                                  req, timeout_ms=self.set_pose_timeout_ms)

    def _fail(self, trigger: str, why: str) -> None:
        text = f"RESYNC FAILED ({trigger}): {why}"
        self.pub_alerts.publish(String(data=text))
        self.get_logger().warn(text + f" — retrying no sooner than {self.cooldown_s:.0f}s")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinResync()
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
