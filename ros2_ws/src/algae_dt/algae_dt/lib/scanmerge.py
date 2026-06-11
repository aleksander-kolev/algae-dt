"""Pure scan merging (no ROS): overlay the TRUSTED mirror's LiDAR onto the real scan, by ANGLE.

The digital-twin direction sim->real for NAVIGATION (not just the 25 cm gate): the mediator
publishes /dt/scan_nav = merge_ranges(real, sim) and the launch points Nav2's obstacle sources
(costmap layers + collision_monitor) at it, so an obstacle placed in the Gazebo twin enters the
REAL robot's costmap and Nav2 plans around it. AMCL keeps the pure real /scan — localization must
never see virtual returns. Tests: test/test_scanmerge.py.

Properties (all pinned by tests):
 * conservative: a virtual return can only SHORTEN a beam (add an obstacle), never erase or
   lengthen a real return;
 * angle-matched: the LDS-02 starts at angle 0 while the gz lidar starts at -pi — beams are
   paired by wrapped ANGLE (nearest sim beam), never by array index, and beam counts may differ;
 * validity-aware: invalid beams (NaN/inf/0.0-sentinel/out-of-spec) on either side never poison
   the other side; if both sides are invalid the real sentinel passes through untouched.
"""
from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


def _valid(r: float, lo: float, hi: float) -> bool:
    return math.isfinite(r) and lo <= r <= hi


def merge_ranges(real_ranges, real_angle_min: float, real_angle_increment: float,
                 sim_ranges, sim_angle_min: float, sim_angle_increment: float,
                 *, sim_range_min: float, sim_range_max: float,
                 real_range_min: float = 0.0, real_range_max: float = float('inf')) -> list:
    """Per-beam MIN of the real scan and the angle-nearest sim beam (see module docstring).

    real_range_min/max default to permissive bounds: the caller usually passes the real scan's
    own self-reported bounds; 0.0/inf keep every finite positive real return valid.
    """
    out = list(real_ranges)
    n_sim = len(sim_ranges)
    if not out or n_sim == 0 or sim_angle_increment == 0.0:
        return out
    rlo = real_range_min if real_range_min > 0.0 else 1e-9   # 0.0 stays an invalid sentinel
    for k, real_r in enumerate(out):
        ang = real_angle_min + k * real_angle_increment
        j = int(round(((ang - sim_angle_min) % TWO_PI) / sim_angle_increment)) % n_sim
        sim_r = sim_ranges[j]
        if not _valid(sim_r, sim_range_min, sim_range_max):
            continue
        if _valid(real_r, rlo, real_range_max):
            out[k] = real_r if real_r <= sim_r else sim_r
        else:
            out[k] = sim_r                       # invalid real beam: the virtual return stands
    return out
