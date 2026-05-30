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

import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from algae_dt.lib import trajectory


class DynamicObstacle(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('dynamic_obstacle', **kwargs)
        gp = self._declare
        self.world = gp('world', 'default')
        self.name = gp('obstacle_name', 'algae_obstacle')
        self.cx = gp('center_x', 0.6)
        self.cy = gp('center_y', 0.0)
        self.z = gp('center_z', 0.25)
        self.amplitude = gp('amplitude', 0.6)
        self.period = gp('period', 12.0)
        self.axis = gp('axis', 'y')
        self.rate_hz = gp('rate_hz', 10.0)
        self.do_spawn = gp('spawn', True)

        self._t0 = self._now()
        self._spawned = False
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"dynamic_obstacle: '{self.name}' sweeping {self.axis} +/-{self.amplitude} m "
            f"about ({self.cx},{self.cy}) period {self.period}s in world '{self.world}'")

    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        if self.do_spawn and not self._spawned:
            self._spawn()
            self._spawned = True
        x, y = trajectory.oscillate(self._now() - self._t0, self.cx, self.cy,
                                    self.amplitude, self.period, self.axis)
        self._set_pose(x, y)

    def _gz(self, service: str, reqtype: str, req: str, timeout_ms: int = 300) -> None:
        try:
            subprocess.run(
                ['gz', 'service', '-s', service, '--reqtype', reqtype,
                 '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req],
                check=False, capture_output=True, timeout=max(1.0, timeout_ms / 1000.0 + 1.0))
        except Exception as exc:                          # pragma: no cover - demo tool, gz-only
            self.get_logger().warn(f"gz {service} failed (sim running?): {exc}")

    def _spawn(self) -> None:
        sdf = get_package_share_directory('algae_dt') + '/worlds/obstacle_box.sdf'
        self._gz(f'/world/{self.world}/create', 'gz.msgs.EntityFactory',
                 f'sdf_filename: "{sdf}", name: "{self.name}", '
                 f'pose: {{position {{x: {self.cx} y: {self.cy} z: {self.z}}}}}', timeout_ms=2000)

    def _set_pose(self, x: float, y: float) -> None:
        self._gz(f'/world/{self.world}/set_pose', 'gz.msgs.Pose',
                 f'name: "{self.name}", position {{x: {x} y: {y} z: {self.z}}}')


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
