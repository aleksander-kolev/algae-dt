"""Pure operator-console (HUD) formatting (no Qt, no ROS). Implemented TDD per PLAN T4.2.

Keeps the colour thresholds and scan projection out of the Qt widget so they are unit-testable and
the GUI stays a thin shell. Tests: test/test_hud.py.
"""
from __future__ import annotations

import math


def battery_color(voltage: float, low_v: float, critical_v: float) -> str:
    """'red' at/below critical, 'amber' at/below low, else 'green' (twin.yaml thresholds)."""
    if voltage <= critical_v:
        return 'red'
    if voltage <= low_v:
        return 'amber'
    return 'green'


def battery_percentage(voltage: float, empty_v: float, full_v: float) -> float:
    """Linear state-of-charge in [0, 1] from a LiPo voltage (empty_v -> 0, full_v -> 1, clamped).
    Single source of truth so nodes never hardcode the empty/full endpoints (RULES §C)."""
    if full_v <= empty_v:
        return 0.0
    return max(0.0, min(1.0, (voltage - empty_v) / (full_v - empty_v)))


def scan_points(ranges, angle_min: float, angle_increment: float,
                range_min: float, range_max: float) -> list[tuple[float, float]]:
    """Valid LaserScan beams projected to robot-frame (x, y); NaN/inf/out-of-range dropped."""
    pts = []
    for i, r in enumerate(ranges):
        if range_min <= r <= range_max:          # rejects NaN and inf too
            a = angle_min + i * angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def safety_text(blocked: bool) -> str:
    return 'BLOCKED' if blocked else 'CLEAR'


def sync_text(ok: bool) -> str:
    return 'IN SYNC' if ok else 'OUT OF SYNC'
