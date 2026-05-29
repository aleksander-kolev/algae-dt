"""Pure real<->sim synchronisation math (no ROS). STUB — TDD in PLAN.md T3.1.

Design:
- pose_error(real_xy_yaw, sim_xy_yaw) -> (dxy, dyaw)
- sensor_error(real_front_range, sim_front_range) -> float
- within_tolerance(errors, tol_pose_xy, tol_pose_yaw, tol_sensor_range) -> bool
- classify(...) -> 'ok' | 'warn' (drives /dt/sync_ok and /dt/alerts)
Pure + table-tested against the documented thresholds in twin.yaml (Rubric ②).
"""
from __future__ import annotations


def pose_error(*args, **kwargs):  # noqa: D401 - stub
    raise NotImplementedError("PLAN.md T3.1 — implement with a failing test first")


def within_tolerance(*args, **kwargs) -> bool:  # noqa: D401 - stub
    raise NotImplementedError("PLAN.md T3.1 — implement with a failing test first")
