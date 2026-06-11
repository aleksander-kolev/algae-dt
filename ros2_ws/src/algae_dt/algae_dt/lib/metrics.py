"""Pure latency + CSV-row formatting (no ROS). Implemented TDD per PLAN T3.1 (Rubric ②).

The sync CSV is the trustworthy evidence for the technical review, so its formatting lives here,
unit-tested: stable columns, NaN/inf rendered safely, booleans as 0/1, and the optional
safety-event `stop_skew_ms` (real-stop vs sim-stop time delta) blank on ordinary ticks.
Tests: test/test_metrics.py.

- latency_ms(t_command_s, t_motion_s) -> float   (command -> motion onset)
- csv_header() -> str
- csv_row(stamp_s, dxy, dyaw, sensor_err, latency_ms, in_tolerance, stop_skew_ms=None,
          resync=None) -> str
"""
from __future__ import annotations

from typing import Optional

_COLUMNS = ('stamp_s', 'dxy_m', 'dyaw_rad', 'sensor_err_m',
            'latency_ms', 'in_tolerance', 'stop_skew_ms', 'resync')


def latency_ms(t_command_s: float, t_motion_s: float) -> float:
    return (t_motion_s - t_command_s) * 1000.0


def csv_header() -> str:
    return ",".join(_COLUMNS)


def _f(v: float) -> str:
    # Python renders inf/nan specially regardless of precision, so this is crash-safe.
    return f"{v:.4f}"


def csv_row(stamp_s: float, dxy: float, dyaw: float, sensor_err: float,
            latency_ms: float, in_tolerance: bool,
            stop_skew_ms: Optional[float] = None,
            resync: Optional[str] = None) -> str:
    """One CSV line aligned with csv_header(). stop_skew_ms is blank except on a safety-stop tick;
    resync is blank except on the tick a twin resync executed ('manual'/'auto' — the evidence that
    drift was CORRECTED, not just measured). Commas are stripped so a tag can never split the row."""
    skew = '' if stop_skew_ms is None else f"{stop_skew_ms:.1f}"
    tag = '' if resync is None else str(resync).replace(',', ';')
    return ",".join([
        _f(stamp_s), _f(dxy), _f(dyaw), _f(sensor_err), _f(latency_ms),
        '1' if in_tolerance else '0', skew, tag,
    ])
