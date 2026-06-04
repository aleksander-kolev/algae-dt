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
    """World metres -> (col, row) pixel. Row is flipped (image origin top-left).

    The flip floors the (y-origin)/resolution term BEFORE subtracting from height-1, so a cell centre
    round-trips exactly: world_to_pixel(pixel_to_world(c,r)) == (c,r) (test_geometry pins this).
    math.floor (NOT int()) so a point in the sub-cell band below/left of the origin maps OUT of
    bounds (-1 / height_px) instead of silently truncating toward 0 onto a valid edge cell — int()
    made an off-map bloom land on the arena edge and get a goal instead of being rejected."""
    col = math.floor((x - m.origin_x) / m.resolution)
    row = m.height_px - 1 - math.floor((y - m.origin_y) / m.resolution)
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


def pose_xyyaw(pose) -> tuple[float, float, float]:
    """(x, y, yaw) from any geometry_msgs Pose-shaped object (duck-typed: .position/.orientation).
    The single quaternion->yaw extraction shared by mediator/supervisor/GUI (was 3 copies)."""
    p, o = pose.position, pose.orientation
    return (p.x, p.y, yaw_from_quaternion(o.z, o.w))


def map_info_from_params(get_param) -> MapInfo:
    """Build the MapInfo from the standard twin.yaml map_* parameters via a `get_param(name,
    default)` callable. Single source for the map defaults (was duplicated verbatim in
    mission_runner and operator_gui)."""
    return MapInfo(
        resolution=get_param('map_resolution', 0.05),
        origin_x=get_param('map_origin_x', -2.051),
        origin_y=get_param('map_origin_y', -4.194),
        width_px=get_param('map_width_px', 86),
        height_px=get_param('map_height_px', 110),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    """(z, w) of the planar unit quaternion for a heading (x=y=0). Inverse of yaw_from_quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose_pose_2d(a: tuple[float, float, float],
                    b: tuple[float, float, float]) -> tuple[float, float, float]:
    """Compose two planar poses a∘b: express pose b (given in a's child frame) in a's parent frame.

    Used to put an odom-frame robot pose into the map frame: compose_pose_2d(map_T_odom, odom_pose).
    Returns (x, y, yaw) with yaw normalized to [-pi, pi) (the bare wrap; angle_diff uses (-pi, pi]).
    """
    ax, ay, ayaw = a
    bx, by, byaw = b
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    x = ax + bx * ca - by * sa
    y = ay + bx * sa + by * ca
    yaw = (ayaw + byaw + math.pi) % (2.0 * math.pi) - math.pi
    return x, y, yaw


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a-b wrapped to (-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d + (2.0 * math.pi if d <= -math.pi else 0.0)
