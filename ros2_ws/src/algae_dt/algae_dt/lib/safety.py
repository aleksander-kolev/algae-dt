"""Pure forward-safety logic (no ROS). The 25 cm dual-LiDAR gate, fail-safe.

STUB — implement via TDD in PLAN.md T1.3. Logic ported from the old repo's safety lib:
- front_min_range(ranges, angle_min, angle_increment, sector_rad, range_min, range_max)
- is_blocked(front_min, stop_distance_m) -> bool
- fail-safe staleness: a CONSIDERED scan that goes stale (had data, now older than max_data_age_s)
  returns a blocking value (treated as obstacle); startup with no data stays unblocked (+inf).
- combine(real_blocked, sim_blocked) -> block forward motion if EITHER is blocked.
Keep it pure + table-tested (incl. NaN/inf handling, wrap-around front sector).
"""
from __future__ import annotations


def front_min_range(*args, **kwargs) -> float:  # noqa: D401 - stub
    raise NotImplementedError("PLAN.md T1.3 — implement with a failing test first")


def is_blocked(*args, **kwargs) -> bool:  # noqa: D401 - stub
    raise NotImplementedError("PLAN.md T1.3 — implement with a failing test first")
