"""Pure forward-safety logic (no ROS). The 25 cm dual-LiDAR gate, fail-safe.

Implemented TDD per PLAN T1.3 (tests in test/test_safety.py). RULES §B-5: the gate fails SAFE —
a *considered* scan that goes stale is treated as blocked; startup with no data yet stays
unblocked so the robot can warm up; EITHER world seeing an obstacle < stop_distance zeroes
forward motion on BOTH. `front_sector_rad` (twin.yaml) is the FULL cone width.

API (the mediator calls `gate()`; the rest are its building blocks, each unit-tested):
- front_min_range(ranges, angle_min, angle_increment, sector_rad, range_min, range_max) -> float
- is_blocked(front_min, stop_distance_m) -> bool
- considered_range(front_min, has_data, age_s, max_data_age_s) -> float   (fail-safe gating)
- combine(*ranges) -> float                                               (dual-LiDAR OR-block)
- gate(real: ScanStatus, sim: ScanStatus, stop_distance_m, max_data_age_s) -> bool
"""
from __future__ import annotations

import math
from dataclasses import dataclass

INF = float('inf')


def front_min_range(ranges, angle_min: float, angle_increment: float,
                    sector_rad: float, range_min: float, range_max: float) -> float:
    """Minimum VALID range within the FULL-width front sector centred on 0 rad.

    `sector_rad` is the whole cone, so the half-angle is sector_rad/2. Beam angles are wrapped to
    (-pi, pi] before the sector test (a near return just behind 0 rad still counts as ahead).
    Beams that are NaN, +/-inf, or outside [range_min, range_max] are ignored. Returns +inf when
    no valid beam falls in the sector (treated downstream as "nothing ahead").
    """
    half = sector_rad / 2.0
    best = INF
    for i, r in enumerate(ranges):
        # Validity first (cheap, and NaN/inf both fail this comparison chain).
        if not (range_min <= r <= range_max):
            continue
        ang = angle_min + i * angle_increment
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi   # wrap to (-pi, pi]
        if -half <= ang <= half and r < best:
            best = r
    return best


def is_blocked(front_min: float, stop_distance_m: float) -> bool:
    """True iff the nearest front obstacle is STRICTLY closer than the stop distance."""
    return front_min < stop_distance_m


def considered_range(front_min: float, has_data: bool,
                     age_s: float, max_data_age_s: float) -> float:
    """Fail-safe gating of one scan's front range (RULES §B-5).

    - no data yet (startup): +inf  -> stays unblocked so the robot can warm up
    - had data but stale (age > max): 0.0 -> treated as an obstacle (blocked)
    - fresh: pass `front_min` through unchanged
    """
    if not has_data:
        return INF
    if age_s > max_data_age_s:
        return 0.0
    return front_min


def combine(*ranges: float) -> float:
    """Dual-LiDAR OR-block: the controlling range is the MIN across worlds, so EITHER world
    seeing an obstacle (or a stale scan forced to 0.0) blocks forward motion on BOTH."""
    return min(ranges) if ranges else INF


@dataclass(frozen=True)
class ScanStatus:
    """A scan's safety-relevant state at gate time (computed by the node, gated purely here)."""
    has_data: bool
    front_min: float   # +inf when no valid front beam / no data
    age_s: float       # seconds since the scan stamp (node clock)


def gate(real: ScanStatus, sim: ScanStatus,
         stop_distance_m: float, max_data_age_s: float) -> bool:
    """Forward-motion block decision for the dual-LiDAR twin, fail-safe.

    Returns True if forward motion must be zeroed. Each world is gated for staleness/startup
    independently, then OR-combined: a close obstacle OR a stale scan in EITHER world blocks both.
    An absent world (has_data=False, e.g. the real robot in sim_only) cannot force a block.
    """
    cr = considered_range(real.front_min, real.has_data, real.age_s, max_data_age_s)
    cs = considered_range(sim.front_min, sim.has_data, sim.age_s, max_data_age_s)
    return is_blocked(combine(cr, cs), stop_distance_m)
