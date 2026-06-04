"""TDD for lib/hud.py — pure operator-console formatting (PLAN T4.2).

No Qt/ROS here: the colour thresholds and scan projection are pure so the GUI stays a thin shell.
"""
import math

from algae_dt.lib import hud


def test_battery_color_thresholds():
    # amber below battery_low_v, red below battery_critical_v (twin.yaml).
    assert hud.battery_color(12.0, 11.0, 10.5) == 'green'
    assert hud.battery_color(10.9, 11.0, 10.5) == 'amber'
    assert hud.battery_color(11.0, 11.0, 10.5) == 'amber'    # at the low threshold
    assert hud.battery_color(10.5, 11.0, 10.5) == 'red'      # at the critical threshold
    assert hud.battery_color(9.0, 11.0, 10.5) == 'red'


def test_battery_percentage_clamps_and_scales():
    # linear state-of-charge: empty_v -> 0, full_v -> 1, clamped to [0, 1] (twin.yaml thresholds).
    assert math.isclose(hud.battery_percentage(12.6, 9.0, 12.6), 1.0, abs_tol=1e-9)
    assert math.isclose(hud.battery_percentage(9.0, 9.0, 12.6), 0.0, abs_tol=1e-9)
    assert math.isclose(hud.battery_percentage(10.8, 9.0, 12.6), 0.5, abs_tol=1e-9)
    assert hud.battery_percentage(13.0, 9.0, 12.6) == 1.0     # clamp high
    assert hud.battery_percentage(8.0, 9.0, 12.6) == 0.0      # clamp low
    assert hud.battery_percentage(11.0, 12.6, 12.6) == 0.0    # degenerate range -> safe 0


def test_scan_points_filters_invalid_and_projects_to_cartesian():
    # beam 0: r=1.0 @ -pi/2 -> (0,-1); beam 1: 9.9 (out of range); beam 2: 0.0 (no return)
    pts = hud.scan_points([1.0, 9.9, 0.0], -math.pi / 2, math.pi / 2, 0.12, 3.5)
    assert len(pts) == 1
    assert math.isclose(pts[0][0], 0.0, abs_tol=1e-9)
    assert math.isclose(pts[0][1], -1.0, abs_tol=1e-9)


def test_scan_points_handles_nan_inf():
    pts = hud.scan_points([float('nan'), float('inf'), 2.0], 0.0, math.pi / 2, 0.12, 3.5)
    assert len(pts) == 1                                   # 2.0 @ pi rad -> (-2, 0)
    assert math.isclose(pts[0][0], -2.0, abs_tol=1e-9)
    assert math.isclose(pts[0][1], 0.0, abs_tol=1e-9)


def test_safety_and_sync_text():
    assert hud.safety_text(True) == 'BLOCKED'
    assert hud.safety_text(False) == 'CLEAR'
    assert hud.sync_text(True) == 'IN SYNC'
    assert hud.sync_text(False) == 'OUT OF SYNC'
