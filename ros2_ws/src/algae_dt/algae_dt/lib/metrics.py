"""Pure latency + CSV-row formatting (no ROS). STUB — TDD in PLAN.md T3.1.

Design (Rubric ②: "measured latency and sync error reported, ... logging when out of tolerance"):
- latency_ms(t_command_s, t_motion_s) -> float
- data_age_ms(t_now_s, t_stamp_s) -> float
- csv_header() -> str
- csv_row(stamp, dxy, dyaw, sensor_err, latency_ms, in_tol) -> str
Pure + tested so the logged numbers are trustworthy evidence for the review.
"""
from __future__ import annotations


def latency_ms(t_command_s: float, t_motion_s: float) -> float:
    return (t_motion_s - t_command_s) * 1000.0


def csv_header() -> str:
    return "stamp_s,dxy_m,dyaw_rad,sensor_err_m,latency_ms,in_tolerance"


def csv_row(*args, **kwargs) -> str:  # noqa: D401 - stub
    raise NotImplementedError("PLAN.md T3.1 — implement with a failing test first")
