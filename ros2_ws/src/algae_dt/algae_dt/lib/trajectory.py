"""Sinusoidal path for the moving obstacle. No ROS/gz here.

dynamic_obstacle uses this to sweep a box across the robot's path, which forces a Nav2 reroute
or a dual-LiDAR stop.
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
