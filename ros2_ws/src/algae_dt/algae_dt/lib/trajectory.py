"""Pure dynamic-obstacle path (no ROS/gz). Implemented TDD per PLAN T6.1 (Rubric III).

A single sinusoidal sweep used by dynamic_obstacle to move a box across the robot's path so the
Nav2 reroute / dual-LiDAR stop (pillar III "environmental change") is reproducible. Tests:
test/test_trajectory.py.
"""
from __future__ import annotations

import math


def oscillate(t: float, cx: float, cy: float, amplitude: float,
              period: float, axis: str = 'y') -> tuple[float, float]:
    """Position at time t of a point oscillating +/-amplitude about (cx, cy) with the given period
    along 'x' or 'y'. period <= 0 holds it static at the centre."""
    off = amplitude * math.sin(2.0 * math.pi * t / period) if period > 0.0 else 0.0
    if axis == 'x':
        return (cx + off, cy)
    return (cx, cy + off)


def sweep_clearance(px: float, py: float, cx: float, cy: float,
                    amplitude: float, axis: str = 'y') -> float:
    """Minimum distance from point (px,py) to the oscillation's swept segment (centre +/- amplitude
    along `axis`). Used to keep the teleported demo box out of a keep-out zone (e.g. the robot
    spawn): the box is <static> so Gazebo applies no collision response when set_pose moves it."""
    if axis == 'x':
        ax, ay, bx, by = cx - amplitude, cy, cx + amplitude, cy
    else:
        ax, ay, bx, by = cx, cy - amplitude, cx, cy + amplitude
    # Point-to-segment distance (degenerate segment = point when amplitude == 0).
    vx, vy = bx - ax, by - ay
    seg_len_sq = vx * vx + vy * vy
    if seg_len_sq <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / seg_len_sq))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def clamp_amplitude_for_keepout(px: float, py: float, cx: float, cy: float,
                                amplitude: float, axis: str, keepout: float):
    """Largest amplitude <= `amplitude` whose swept segment stays >= `keepout` from (px,py).
    Returns None when even amplitude 0 (the centre itself) violates the keep-out — the caller must
    refuse to run rather than teleport a box onto the protected point."""
    if sweep_clearance(px, py, cx, cy, 0.0, axis) < keepout:
        return None
    if sweep_clearance(px, py, cx, cy, amplitude, axis) >= keepout:
        return amplitude
    # Bisect: clearance is monotonically non-increasing in amplitude.
    lo, hi = 0.0, amplitude
    for _ in range(40):
        mid = (lo + hi) / 2.0
        if sweep_clearance(px, py, cx, cy, mid, axis) >= keepout:
            lo = mid
        else:
            hi = mid
    return lo
