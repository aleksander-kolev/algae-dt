"""TDD for lib/geometry.py — the planar quaternion<->yaw pair + transform coverage (PLAN T2.1)."""
import math

from algae_dt.lib import geometry as g

MAP = g.MapInfo(resolution=0.05, origin_x=-2.051, origin_y=-4.194, width_px=86, height_px=110)


def test_quaternion_from_yaw_zero():
    z, w = g.quaternion_from_yaw(0.0)
    assert math.isclose(z, 0.0) and math.isclose(w, 1.0)


def test_quaternion_yaw_round_trip():
    for yaw in (0.0, 0.5, math.pi / 2, -1.2, math.pi - 0.01):
        z, w = g.quaternion_from_yaw(yaw)
        assert math.isclose(g.yaw_from_quaternion(z, w), yaw, abs_tol=1e-9)


def test_world_pixel_round_trip_nonorigin():
    col, row = g.world_to_pixel(1.0, -1.0, MAP)
    x, y = g.pixel_to_world(col, row, MAP)
    assert abs(x - 1.0) < MAP.resolution and abs(y - (-1.0)) < MAP.resolution


def test_world_to_pixel_row_is_flipped():
    # +y in the world maps to a SMALLER row (image origin is top-left).
    _, row_low = g.world_to_pixel(0.0, 0.0, MAP)
    _, row_high = g.world_to_pixel(0.0, 1.0, MAP)
    assert row_high < row_low


def test_euclidean():
    assert math.isclose(g.euclidean(0.0, 0.0, 3.0, 4.0), 5.0)
