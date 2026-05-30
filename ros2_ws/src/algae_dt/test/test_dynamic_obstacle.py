"""Construct test for dynamic_obstacle (PLAN T6.1). rclpy-only; the path maths are in test_trajectory.

Verifies the demo node constructs and reads params without a running sim (no gz calls fire until the
timer ticks, which a no-spin construction never does)."""
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter                    # noqa: E402

from algae_dt.dynamic_obstacle import DynamicObstacle    # noqa: E402


def test_constructs_without_sim():
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, False),
            Parameter('amplitude', Parameter.Type.DOUBLE, 0.6),
            Parameter('axis', Parameter.Type.STRING, 'y'),
        ])
        assert abs(node.amplitude - 0.6) < 1e-9 and node.axis == 'y'
        node.destroy_node()
    finally:
        rclpy.shutdown()
