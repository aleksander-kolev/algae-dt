"""Pure forward-safety logic (no ROS). The 25 cm dual-LiDAR gate, fail-safe.

Implemented TDD per PLAN T1.3 (tests in test/test_safety.py). RULES §B-5: the gate fails SAFE —
a *considered* scan that goes stale is treated as blocked; startup with no data yet stays
unblocked so the robot can warm up; EITHER world seeing an obstacle < stop_distance zeroes
forward motion on BOTH. `front_sector_rad` (twin.yaml) is the FULL cone width.

KNOWN SENSOR BLIND SPOT (documented, not fixable in software): the LDS-02 reports an obstacle
physically closer than its range_min (0.12 m) as 0.0 — the same value it uses for "no return".
Such beams are (correctly) dropped as invalid, so a flat object pressed inside 12 cm reads as
"nothing ahead". In any normal approach the 25 cm gate stops the robot long before 12 cm; the
blind spot only matters for an obstacle appearing instantaneously inside it.

API (the mediator calls `gate()`; the rest are its building blocks, each unit-tested):
- front_min_range(ranges, angle_min, angle_increment, sector_rad, range_min, range_max) -> float
- front_min_from_scan(scan, sector_rad, range_min, range_max) -> float    (bounds-clamped wrapper)
- is_blocked(front_min, stop_distance_m) -> bool
- considered_range(front_min, has_data, age_s, max_data_age_s) -> float   (fail-safe gating)
- combine(*ranges) -> float                                               (dual-LiDAR OR-block)
- gate(real, sim, stop_distance_m, max_data_age_s, sim_max_data_age_s=None) -> bool
"""
from __future__ import annotations

import math
from dataclasses import dataclass

INF = float('inf')


def front_min_range(ranges, angle_min: float, angle_increment: float,
                    sector_rad: float, range_min: float, range_max: float) -> float:
    """Minimum VALID range within the FULL-width front sector centred on 0 rad.

    `sector_rad` is the whole cone, so the half-angle is sector_rad/2. Beam angles are wrapped to
    [-pi, pi) before the sector test (a near return just behind 0 rad still counts as ahead).
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
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi   # wrap to [-pi, pi)
        if -half <= ang <= half and r < best:
            best = r
    return best


def front_min_from_scan(scan, sector_rad: float, range_min: float, range_max: float) -> float:
    """front_min_range over a LaserScan-shaped object (duck-typed), with the scan's self-reported
    validity bounds CLAMPED to the configured LDS-02 bounds: a driver reporting a too-small
    range_min cannot widen the admitted window (letting sub-spec noise 'block' the robot), and a
    too-small range_max cannot drop valid far returns. A 0.0 (unset) field falls back entirely.
    The single wrapper shared by twin_mediator and sync_supervisor (was 2 copies, unclamped)."""
    eff_min = max(scan.range_min, range_min) if scan.range_min > 0.0 else range_min
    eff_max = min(scan.range_max, range_max) if scan.range_max > 0.0 else range_max
    return front_min_range(scan.ranges, scan.angle_min, scan.angle_increment,
                           sector_rad, eff_min, eff_max)


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


def limit_command(vx: float, wz: float, *, blocked: bool, estop: bool,
                  max_linear: float, max_angular: float) -> tuple[float, float]:
    """Shape one commanded (vx, wz) for output: full-stop on E-STOP, clamp to the Burger limits,
    and zero FORWARD motion when the safety gate blocks (rotation and reverse still allowed)."""
    if estop:
        return 0.0, 0.0
    vx = max(-max_linear, min(max_linear, vx))
    wz = max(-max_angular, min(max_angular, wz))
    if blocked and vx > 0.0:
        vx = 0.0
    return vx, wz


def gate(real: ScanStatus, sim: ScanStatus,
         stop_distance_m: float, max_data_age_s: float,
         sim_max_data_age_s: float | None = None) -> bool:
    """Forward-motion block decision for the dual-LiDAR twin, fail-safe.

    Returns True if forward motion must be zeroed. Each world is gated for staleness/startup
    independently, then OR-combined: a close obstacle OR a stale scan in EITHER world blocks both.
    An absent world (has_data=False, e.g. the real robot in sim_only) cannot force a block.

    `sim_max_data_age_s` (default: same as the real budget) lets the MIRROR sim run a looser
    staleness window: the GPU-less/WSL-rendered sim LiDAR only manages ~2.5-3.5 Hz, and a single
    delayed sim frame past the real 1.0 s budget would otherwise nuisance-stop the REAL robot.
    The real (safety-critical) scan keeps the tight budget.
    """
    cr = considered_range(real.front_min, real.has_data, real.age_s, max_data_age_s)
    cs = considered_range(sim.front_min, sim.has_data, sim.age_s,
                          sim_max_data_age_s if sim_max_data_age_s is not None else max_data_age_s)
    return is_blocked(combine(cr, cs), stop_distance_m)
