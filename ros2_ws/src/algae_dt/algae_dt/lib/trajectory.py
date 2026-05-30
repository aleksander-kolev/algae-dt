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
