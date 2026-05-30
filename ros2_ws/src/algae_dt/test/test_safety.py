"""TDD for lib/safety.py — the 25 cm dual-LiDAR forward-safety gate (PLAN T1.3, RULES §B-5).

These tests pin the behaviours the mediator depends on:
- FULL-width front sector (front_sector_rad is the whole cone, not a half-angle),
- invalid-beam rejection (NaN / inf / out-of-[range_min,range_max]),
- angle wrap-around (a near return behind 0 rad still counts as "ahead"),
- fail-safe staleness (a scan that HAD data and goes stale => blocked; startup no-data => unblocked),
- dual-LiDAR OR-block (EITHER world close => forward blocked on BOTH).
"""
import math

from algae_dt.lib import safety

INF = float('inf')


# A clean synthetic scan with exact angles: 11 beams from -0.5 to +0.5 rad, step 0.1.
# Front sector full-width 0.70 => half-angle 0.35 => beams at indices 2..8 (angles -0.3..+0.3).
def _flat_scan(value=3.0, n=11):
    return [value] * n


SECTOR = 0.70           # FULL width => +/-0.35 rad
RMIN, RMAX = 0.12, 3.5  # LDS-02


# ----------------------------- front_min_range -----------------------------

def test_front_min_range_picks_nearest_in_sector():
    r = _flat_scan()
    r[5] = 0.8   # angle 0.0 (dead ahead)
    assert math.isclose(safety.front_min_range(r, -0.5, 0.1, SECTOR, RMIN, RMAX), 0.8)


def test_front_sector_is_full_width_not_half():
    # angle +0.3 (idx 8) is INSIDE the +/-0.35 half-angle; angle +0.4 (idx 9) is OUTSIDE.
    # If sector_rad were mistakenly treated as a half-angle, idx 9 would wrongly be included.
    r = _flat_scan()
    r[8] = 1.0   # angle +0.3, inside
    r[9] = 0.5   # angle +0.4, outside
    assert math.isclose(safety.front_min_range(r, -0.5, 0.1, SECTOR, RMIN, RMAX), 1.0)


def test_front_min_range_ignores_beams_outside_sector():
    r = [0.0] * 11   # 0.0 = no-return sentinel (out of range) -> all in-sector beams invalid
    r[0] = 0.3       # angle -0.5, a NEAR but out-of-sector return
    r[10] = 0.3      # angle +0.5, out of sector
    assert safety.front_min_range(r, -0.5, 0.1, SECTOR, RMIN, RMAX) == INF


def test_front_min_range_ignores_nan_and_inf():
    r = _flat_scan()
    r[4] = float('nan')
    r[5] = float('inf')
    r[6] = 1.2   # the only valid in-sector reading
    assert math.isclose(safety.front_min_range(r, -0.5, 0.1, SECTOR, RMIN, RMAX), 1.2)


def test_front_min_range_ignores_out_of_bounds():
    r = _flat_scan()
    r[4] = 0.05         # below range_min (noise) -> ignored
    r[5] = 9.0          # above range_max -> ignored
    r[6] = 0.0          # zero (no-return sentinel) -> ignored
    r[7] = 1.5          # valid
    assert math.isclose(safety.front_min_range(r, -0.5, 0.1, SECTOR, RMIN, RMAX), 1.5)


def test_front_min_range_no_valid_returns_inf():
    r = [float('nan')] * 11   # nothing valid anywhere
    assert safety.front_min_range(r, -0.5, 0.1, SECTOR, RMIN, RMAX) == INF


def test_front_min_range_handles_wraparound():
    # LDS-02-style: 360 beams from 0 rad, full circle. A near return at index 350
    # (angle ~6.10 rad => wraps to ~-0.18 rad) IS ahead; a near return behind (index 180,
    # angle ~pi) must be ignored.
    n = 360
    inc = 2.0 * math.pi / n
    r = [3.0] * n
    r[350] = 0.20   # wraps to ~-0.18 rad -> inside front cone
    r[180] = 0.15   # directly behind -> ignored
    assert math.isclose(safety.front_min_range(r, 0.0, inc, SECTOR, RMIN, RMAX), 0.20)


def test_front_min_range_empty_is_inf():
    assert safety.front_min_range([], 0.0, 0.1, SECTOR, RMIN, RMAX) == INF


# ------------------------------- is_blocked --------------------------------

def test_is_blocked_strictly_less_than_stop():
    assert safety.is_blocked(0.24, 0.25) is True
    assert safety.is_blocked(0.25, 0.25) is False   # exactly at stop distance is NOT blocked
    assert safety.is_blocked(0.26, 0.25) is False
    assert safety.is_blocked(INF, 0.25) is False


# ---------------------------- considered_range (fail-safe) -----------------

def test_considered_range_startup_no_data_stays_unblocked():
    # Never received a considered scan -> +inf so the robot can warm up (RULES §B-5).
    assert safety.considered_range(INF, has_data=False, age_s=999.0, max_data_age_s=1.0) == INF


def test_considered_range_stale_fails_safe_to_blocked():
    # Had data, now older than the window -> treat as an obstacle at 0.0 (blocked).
    assert safety.considered_range(2.0, has_data=True, age_s=1.5, max_data_age_s=1.0) == 0.0


def test_considered_range_fresh_passthrough():
    assert math.isclose(
        safety.considered_range(2.0, has_data=True, age_s=0.2, max_data_age_s=1.0), 2.0)


def test_considered_range_exactly_at_age_limit_is_fresh():
    assert math.isclose(
        safety.considered_range(2.0, has_data=True, age_s=1.0, max_data_age_s=1.0), 2.0)


# -------------------------------- combine ----------------------------------

def test_combine_takes_minimum():
    assert math.isclose(safety.combine(3.0, 0.2, INF), 0.2)


def test_combine_empty_is_inf():
    assert safety.combine() == INF


# ---------------------------------- gate -----------------------------------

def _status(has_data=True, front_min=INF, age_s=0.0):
    return safety.ScanStatus(has_data=has_data, front_min=front_min, age_s=age_s)


def test_gate_blocks_when_either_world_close():
    real = _status(front_min=3.0)
    sim = _status(front_min=0.20)            # sim sees a box within 25 cm
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is True


def test_gate_clear_when_both_far():
    real = _status(front_min=3.0)
    sim = _status(front_min=2.0)
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is False


def test_gate_failsafe_blocks_on_stale_scan():
    real = _status(front_min=3.0, age_s=5.0)   # had data, went stale
    sim = _status(front_min=3.0, age_s=0.1)
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is True


def test_gate_unblocked_on_startup_no_data():
    # sim_only-style: real never present, sim present-but-far -> not blocked.
    real = _status(has_data=False)
    sim = _status(has_data=True, front_min=3.0, age_s=0.1)
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is False


def test_gate_sim_only_ignores_absent_real_but_honors_sim_obstacle():
    real = _status(has_data=False)
    sim = _status(has_data=True, front_min=0.20, age_s=0.1)
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is True


# ------------------------------ limit_command ------------------------------
# Pure command-shaping the mediator applies every cycle: clamp to the Burger limits, zero forward
# when the gate blocks (rotation still allowed), and full-stop on E-STOP.

def test_limit_command_estop_zeros_everything():
    assert safety.limit_command(0.22, 2.0, blocked=False, estop=True,
                                 max_linear=0.22, max_angular=2.0) == (0.0, 0.0)


def test_limit_command_clamps_to_limits():
    vx, wz = safety.limit_command(1.0, 5.0, blocked=False, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert vx == 0.22 and wz == 2.0
    vx, wz = safety.limit_command(-1.0, -5.0, blocked=False, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert vx == -0.22 and wz == -2.0


def test_limit_command_blocked_zeros_forward_keeps_rotation():
    vx, wz = safety.limit_command(0.20, 1.0, blocked=True, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert vx == 0.0 and wz == 1.0


def test_limit_command_blocked_allows_reverse():
    # Backing away from an obstacle is permitted; only forward motion is cut.
    vx, wz = safety.limit_command(-0.1, 0.5, blocked=True, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert vx == -0.1 and wz == 0.5


def test_limit_command_clear_passes_through():
    vx, wz = safety.limit_command(0.1, 0.5, blocked=False, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert vx == 0.1 and wz == 0.5
