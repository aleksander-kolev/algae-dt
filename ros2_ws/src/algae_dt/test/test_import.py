"""Smoke tests: the package + pure libs import, and the trivially-true geometry math holds.

Keeps the skeleton green on `pytest`. Real logic tests are added per PLAN.md (T1.3, T2.1, T3.1, T4.1)
as each pure lib is implemented TDD-style.
"""
import math

from algae_dt.lib import geometry as g


def test_package_imports():
    import algae_dt  # noqa: F401
    from algae_dt.lib import safety, blooms, sync, metrics, pgm  # noqa: F401


def test_world_pixel_roundtrip():
    m = g.MapInfo(resolution=0.05, origin_x=-2.051, origin_y=-4.194, width_px=86, height_px=110)
    col, row = g.world_to_pixel(0.0, 0.0, m)
    x, y = g.pixel_to_world(col, row, m)
    assert abs(x - 0.0) < m.resolution and abs(y - 0.0) < m.resolution


def test_angle_diff_wraps():
    assert abs(g.angle_diff(math.pi - 0.1, -math.pi + 0.1) - (-0.2)) < 1e-6


def test_latency_ms_sign():
    from algae_dt.lib.metrics import latency_ms
    assert latency_ms(1.0, 1.25) == 250.0
