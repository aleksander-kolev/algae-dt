"""Static-map occupancy grid + goal projection. No ROS.

Why this exists: a bloom dropped near a wall lands inside Nav2's costmap inflation/lethal zone, so
Nav2 cannot finish a plan to its exact centre — it runs recovery behaviours then ABORTS (bloom
greyed), or stalls against the 25 cm safety gate until the nav timeout. Either way the robot "starts
then does nothing". `mission_runner` therefore projects every bloom goal to the NEAREST point the
robot can actually occupy with `clearance_m` to every obstacle, and sprays there. Truly boxed-in
(no clear cell within `max_projection_m`) -> None, so the mission skips it immediately rather than
letting Nav2 churn.

The map is maps/map.pgm + map.yaml. For clearance purposes, obstacles are all non-free cells
(occupied or unknown) plus off-map cells, so projected goals sit well inside the arena.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from algae_dt.lib import geometry


@dataclass(frozen=True)
class Grid:
    """Immutable occupancy grid. `free` is row-major (row 0 on top, matching the PGM image), True
    where the cell is navigable free space."""
    width: int
    height: int
    free: tuple[bool, ...]


def from_pgm(img, occupied_thresh: float = 0.65, free_thresh: float = 0.196,
             negate: int = 0) -> Grid:
    """Classify a parsed PGM (lib.pgm.Pgm) into free/blocked using map_server's trinary rule.

    With negate=0 the occupancy probability of a pixel is (255-px)/255; a cell is navigable FREE only
    when that probability is strictly below `free_thresh` (so the conventional 205 'unknown' grey and
    every occupied pixel are treated as blocked — conservative, keeps goals inside the arena)."""
    free = []
    for px in img.pixels:
        occ = (px / 255.0) if negate else ((255 - px) / 255.0)
        free.append(occ < free_thresh)
    return Grid(img.width, img.height, tuple(free))


def is_free(grid: Grid, col: int, row: int) -> bool:
    """True iff (col,row) is in bounds AND a free cell. Off-map is blocked (don't hug the boundary)."""
    if col < 0 or row < 0 or col >= grid.width or row >= grid.height:
        return False
    return grid.free[row * grid.width + col]


def clear_radius_ok(grid: Grid, col: int, row: int, clearance_px: float) -> bool:
    """True iff (col,row) is free and NO blocked/off-map cell lies within Euclidean `clearance_px`."""
    if not is_free(grid, col, row):
        return False
    rad = int(math.ceil(clearance_px))
    for dr in range(-rad, rad + 1):
        for dc in range(-rad, rad + 1):
            if math.hypot(dc, dr) <= clearance_px and not is_free(grid, col + dc, row + dr):
                return False
    return True


def nearest_clear_cell(grid: Grid, col: int, row: int, clearance_px: float,
                       max_radius_px: float):
    """Nearest (by Euclidean distance) cell to (col,row) that satisfies `clear_radius_ok`, searching
    out to `max_radius_px`. Returns (col,row) or None. (col,row) itself wins when already clear."""
    R = int(math.ceil(max_radius_px))
    candidates = []
    for dr in range(-R, R + 1):
        for dc in range(-R, R + 1):
            d = math.hypot(dc, dr)
            if d <= max_radius_px:
                candidates.append((d, col + dc, row + dr))
    candidates.sort(key=lambda t: t[0])
    for _d, c, r in candidates:
        if clear_radius_ok(grid, c, r, clearance_px):
            return (c, r)
    return None


def reachable_goal(grid: Grid, map_info: geometry.MapInfo, x: float, y: float,
                   clearance_m: float, max_projection_m: float):
    """Project a world-frame goal (x,y) to the nearest reachable point with >= `clearance_m` to every
    obstacle, searching no farther than `max_projection_m`. Returns (x,y) world metres, or None when
    no such point exists within range (caller skips the bloom)."""
    if map_info.resolution <= 0.0:
        return None
    col, row = geometry.world_to_pixel(x, y, map_info)
    cell = nearest_clear_cell(grid, col, row,
                              clearance_px=clearance_m / map_info.resolution,
                              max_radius_px=max_projection_m / map_info.resolution)
    if cell is None:
        return None
    return geometry.pixel_to_world(cell[0], cell[1], map_info)
