"""Pure geometry helpers (no ROS). World<->map-pixel transforms + small math.

Fully implemented (trivial, high-confidence). See PLAN.md T2.1. Map params come from twin.yaml.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MapInfo:
    """Immutable map metadata (matches maps/map.yaml + twin.yaml)."""
    resolution: float
    origin_x: float
    origin_y: float
    width_px: int
    height_px: int


def world_to_pixel(x: float, y: float, m: MapInfo) -> tuple[int, int]:
    """World metres -> (col, row) pixel. Row is flipped (image origin top-left)."""
    col = int((x - m.origin_x) / m.resolution)
    row = int(m.height_px - 1 - (y - m.origin_y) / m.resolution)
    return col, row


def pixel_to_world(col: int, row: int, m: MapInfo) -> tuple[float, float]:
    """(col, row) pixel -> world metres (centre of the pixel)."""
    x = m.origin_x + (col + 0.5) * m.resolution
    y = m.origin_y + (m.height_px - 1 - row + 0.5) * m.resolution
    return x, y


def euclidean(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def yaw_from_quaternion(z: float, w: float) -> float:
    """Planar yaw from the (z, w) of a unit quaternion (x=y=0 for ground robots)."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    """(z, w) of the planar unit quaternion for a heading (x=y=0). Inverse of yaw_from_quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a-b wrapped to (-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d + (2.0 * math.pi if d <= -math.pi else 0.0)
