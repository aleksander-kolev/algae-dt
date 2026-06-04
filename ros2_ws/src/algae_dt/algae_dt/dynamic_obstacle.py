"""dynamic_obstacle — a moving obstacle for the pillar-III "live environment change" demo. PLAN T6.1.

Spawns a box into the running Gazebo world and sweeps it across the robot's path (sinusoidal, from
lib.trajectory) by calling the gz `create` / `set_pose` services. The robot's LiDAR then sees a
moving obstacle → the dual-LiDAR gate stops forward motion and/or Nav2 reroutes. The path is pure +
unit-tested; the gz calls are best-effort (a demo tool — failures are logged, never fatal). gz calls
are deferred out of __init__ so the node constructs without a running sim.

Run alongside a sim_only/both launch:
    ros2 run algae_dt dynamic_obstacle --ros-args -p center_x:=0.6 -p amplitude:=0.6 -p period:=12.0
"""
from __future__ import annotations

import functools
import re
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from algae_dt.lib import trajectory
from algae_dt.lib.ros_utils import declare_get


class DynamicObstacle(Node):
    _SET_POSE_REFRESH_AFTER = 20      # consecutive failed set_pose calls (~2 s at 10 Hz) before re-spawning

    def __init__(self, **kwargs) -> None:
        super().__init__('dynamic_obstacle', **kwargs)
        gp = functools.partial(declare_get, self)
        self.world = self._safe_token(gp('world', 'default'), 'world')
        self.name = self._safe_token(gp('obstacle_name', 'algae_obstacle'), 'obstacle_name')
        self.cx = gp('center_x', 0.6)
        self.cy = gp('center_y', 0.0)
        self.z = gp('center_z', 0.25)
        self.amplitude = gp('amplitude', 0.6)
        self.period = gp('period', 12.0)
        # `axis` tolerates ros2's YAML coercion of a bare `-p axis:=y` to bool True (the "Norway
        # problem"): declare it dynamically-typed and normalize to 'x'/'y' so the demo never crashes.
        self.declare_parameter('axis', 'y', ParameterDescriptor(dynamic_typing=True))
        self.axis = 'x' if str(self.get_parameter('axis').value).strip().lower() == 'x' else 'y'
        self.rate_hz = gp('rate_hz', 10.0)
        self.do_spawn = gp('spawn', True)
        self.spawn_timeout_ms = int(gp('spawn_timeout_ms', 5000))   # create can be slow on a throttled sim
        self.set_pose_timeout_ms = int(gp('set_pose_timeout_ms', 500))
        self.keepout_radius_m = gp('keepout_radius_m', 0.30)   # protected radius around the robot spawn

        # The box is <static>true</static> and moved by TELEPORT (set_pose) — Gazebo applies no
        # collision response, so a sweep through the robot spawn (0,0) would shove the box INTO the
        # robot. Clamp the amplitude to keep the swept segment out of the keep-out circle; refuse
        # outright when even the centre violates it.
        clamped = trajectory.clamp_amplitude_for_keepout(
            0.0, 0.0, self.cx, self.cy, self.amplitude, self.axis, self.keepout_radius_m)
        if clamped is None:
            self.get_logger().error(
                f"obstacle centre ({self.cx},{self.cy}) is inside the {self.keepout_radius_m} m "
                "keep-out around the robot spawn (0,0) — refusing to spawn/move the box")
            self.do_spawn = False
            self.amplitude = 0.0
            self._refused = True
        else:
            self._refused = False
            if clamped < self.amplitude:
                self.get_logger().warn(
                    f"amplitude clamped {self.amplitude} -> {clamped:.2f} m so the sweep keeps "
                    f">= {self.keepout_radius_m} m clearance from the robot spawn (0,0)")
                self.amplitude = clamped

        self._t0 = self._now()
        self._spawned = False
        self._set_pose_fails = 0          # consecutive set_pose failures after a confirmed spawn
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"dynamic_obstacle: '{self.name}' sweeping {self.axis} +/-{self.amplitude} m "
            f"about ({self.cx},{self.cy}) period {self.period}s in world '{self.world}'")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _safe_token(value: str, label: str) -> str:
        """Reject gz entity names/worlds that would break the protobuf --req text (quotes/braces/
        whitespace): validate at the boundary, fail fast (RULES §C) instead of emitting a bad request."""
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', value or ''):
            raise ValueError(f"{label} must match [A-Za-z0-9_.-]+ (got {value!r})")
        return value

    def _tick(self) -> None:
        if self._refused:
            return     # keep-out violation at construction: never spawn/teleport (already logged)
        # RETRY the create until the service CONFIRMS it (data:true) — never set _spawned on an
        # unconfirmed/slow call and then teleport a model that doesn't exist (silent failure, RULES §C).
        if self.do_spawn and not self._spawned:
            if not self._spawn():
                self.get_logger().warn("obstacle spawn not confirmed yet (slow/absent sim?); retrying",
                                       throttle_duration_sec=2.0)
                return
            self._spawned = True
            self.get_logger().info(f"obstacle '{self.name}' spawned")
        x, y = trajectory.oscillate(self._now() - self._t0, self.cx, self.cy,
                                    self.amplitude, self.period, self.axis)
        if self._set_pose(x, y) or not self._spawned:
            self._set_pose_fails = 0
        else:
            # The model was confirmed spawned but set_pose keeps failing (e.g. a world reset removed
            # it): don't freeze silently — re-arm the spawn so _tick re-creates it next pass instead
            # of teleporting a model that no longer exists.
            self._set_pose_fails += 1
            if self._set_pose_fails >= self._SET_POSE_REFRESH_AFTER:
                self.get_logger().warn(
                    f"obstacle '{self.name}' set_pose failing repeatedly; re-creating it",
                    throttle_duration_sec=5.0)
                self._spawned = False
                self._set_pose_fails = 0

    def _gz(self, service: str, reqtype: str, req: str, timeout_ms: int = 300) -> bool:
        """Call a gz Boolean service; return True iff it replied `data: true`. Best-effort: a missing
        sim / timeout logs a throttled warning and returns False so the caller can retry (never a
        silent give-up, RULES §C)."""
        try:
            r = subprocess.run(
                ['gz', 'service', '-s', service, '--reqtype', reqtype,
                 '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req],
                check=False, capture_output=True, text=True,
                timeout=max(1.0, timeout_ms / 1000.0 + 1.0))
            return 'data: true' in (r.stdout or '')
        except Exception as exc:                          # pragma: no cover - demo tool, gz-only
            self.get_logger().warn(f"gz {service} failed (sim running?): {exc}",
                                   throttle_duration_sec=2.0)
            return False

    def _spawn(self) -> bool:
        sdf = get_package_share_directory('algae_dt') + '/worlds/obstacle_box.sdf'
        return self._gz(f'/world/{self.world}/create', 'gz.msgs.EntityFactory',
                        f'sdf_filename: "{sdf}", name: "{self.name}", '
                        f'pose: {{position {{x: {self.cx} y: {self.cy} z: {self.z}}}}}',
                        timeout_ms=self.spawn_timeout_ms)

    def _set_pose(self, x: float, y: float) -> bool:
        return self._gz(f'/world/{self.world}/set_pose', 'gz.msgs.Pose',
                        f'name: "{self.name}", position {{x: {x} y: {y} z: {self.z}}}',
                        timeout_ms=self.set_pose_timeout_ms)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacle()
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
