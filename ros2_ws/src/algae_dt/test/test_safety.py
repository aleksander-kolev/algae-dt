"""TDD for lib/safety.py — the 25 cm dual-LiDAR forward-safety gate (PLAN T1.3, RULES §B-5).

These tests pin the behaviours the mediator depends on:
- FULL-width front sector (front_sector_rad is the whole cone, not a half-angle),
- invalid-beam rejection (NaN / inf / out-of-[range_min,range_max]),
- angle wrap-around (a near return behind 0 rad still counts as "ahead"),
- fail-safe staleness (a scan that HAD data and goes stale => blocked; startup no-data => unblocked),
- dual-LiDAR OR-block (EITHER world close => forward blocked on BOTH).
"""
import math

import pytest

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


def test_gate_sim_scan_gets_its_own_looser_staleness_budget():
    # `both` mode: the WSL-rendered sim LiDAR runs ~2.5-3.5 Hz, so a sim frame 1.5 s old is normal.
    # With a per-world budget the REAL robot is not nuisance-stopped by it...
    real = _status(front_min=3.0, age_s=0.1)
    sim = _status(front_min=3.0, age_s=1.5)
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0,
                       sim_max_data_age_s=2.0) is False
    # ...but a sim scan beyond ITS budget still fail-safes to blocked,
    sim_stale = _status(front_min=3.0, age_s=2.5)
    assert safety.gate(real, sim_stale, stop_distance_m=0.25, max_data_age_s=1.0,
                       sim_max_data_age_s=2.0) is True
    # ...and the REAL scan keeps the TIGHT budget regardless.
    real_stale = _status(front_min=3.0, age_s=1.5)
    sim_fresh = _status(front_min=3.0, age_s=0.1)
    assert safety.gate(real_stale, sim_fresh, stop_distance_m=0.25, max_data_age_s=1.0,
                       sim_max_data_age_s=2.0) is True


def test_gate_default_sim_budget_equals_real_budget():
    real = _status(front_min=3.0, age_s=0.1)
    sim = _status(front_min=3.0, age_s=1.5)
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is True


def test_gate_blocks_when_REAL_world_close_not_only_sim():
    """Discriminating: a regression that ignored the REAL scan's range (only gating on sim) would
    pass every other 'blocked' test here, which all put the obstacle on the sim side. The real
    Burger seeing a 20 cm obstacle MUST block forward motion on its own."""
    real = _status(front_min=0.20)           # the REAL robot sees the box within 25 cm
    sim = _status(front_min=3.0)             # sim is clear
    assert safety.gate(real, sim, stop_distance_m=0.25, max_data_age_s=1.0) is True


# --------------------------- limit_command fail-safe -------------------------

def test_limit_command_nan_velocity_fails_to_stop_not_full_throttle():
    """A NaN command must STOP, never drive: min(0.22, NaN)->0.22 then max(-0.22, 0.22)->0.22, so an
    unvalidated NaN was AMPLIFIED to full forward+angular speed when the gate was clear (fail-unsafe).
    Every non-finite component now yields (0,0)."""
    for vx, wz in ((float('nan'), 0.1), (0.1, float('nan')),
                   (float('inf'), 0.0), (0.0, float('-inf')), (float('nan'), float('nan'))):
        out = safety.limit_command(vx, wz, blocked=False, estop=False,
                                   max_linear=0.22, max_angular=2.0)
        assert out == (0.0, 0.0), f"non-finite ({vx},{wz}) must fail safe to stop, got {out}"


def test_limit_command_finite_still_clamps_normally():
    # Over-limit commands scale BOTH components by ONE factor (curvature preserved, see the
    # dedicated test below). Here angular is the binding limit: s = 2.0/9.0.
    vx, wz = safety.limit_command(0.5, 9.0, blocked=False, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert math.isclose(wz, 2.0) and math.isclose(vx, 0.5 * (2.0 / 9.0))
    assert safety.limit_command(0.5, 0.0, blocked=True, estop=False,
                                max_linear=0.22, max_angular=2.0) == (0.0, 0.0)


# --------------------------- front_min_from_scan ----------------------------
# The bounds-clamped LaserScan wrapper shared by the mediator and sync_supervisor.

class _Scan:
    def __init__(self, ranges, range_min=0.12, range_max=3.5, angle_min=-0.5, angle_increment=0.1):
        self.ranges = ranges
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_increment = angle_increment


def test_front_min_from_scan_uses_scan_fields():
    r = _flat_scan()
    r[5] = 0.8
    assert math.isclose(safety.front_min_from_scan(_Scan(r), SECTOR, RMIN, RMAX), 0.8)


def test_front_min_from_scan_clamps_a_too_small_range_min():
    # A driver reporting range_min=0.05 must NOT widen the admitted window: a 0.08 'return' is
    # sub-spec noise on the LDS-02 and may not block the robot.
    r = _flat_scan()
    r[5] = 0.08
    r[6] = 1.5
    fm = safety.front_min_from_scan(_Scan(r, range_min=0.05), SECTOR, RMIN, RMAX)
    assert math.isclose(fm, 1.5)


def test_front_min_from_scan_clamps_a_too_small_range_max():
    # A driver reporting range_max=10 must not admit beyond-spec returns either: with every other
    # beam invalid (0.0 sentinel), the lone 5.0 m return must be dropped by the clamped 3.5 m max.
    r = [0.0] * 11
    r[5] = 5.0     # beyond the LDS-02 3.5 m spec
    fm = safety.front_min_from_scan(_Scan(r, range_max=10.0), SECTOR, RMIN, RMAX)
    assert fm == INF


def test_front_min_from_scan_zero_fields_fall_back_to_config():
    r = _flat_scan()
    r[5] = 0.8
    fm = safety.front_min_from_scan(_Scan(r, range_min=0.0, range_max=0.0), SECTOR, RMIN, RMAX)
    assert math.isclose(fm, 0.8)


# ------------------------------ limit_command ------------------------------
# Pure command-shaping the mediator applies every cycle: clamp to the Burger limits, zero forward
# when the gate blocks (rotation still allowed), and full-stop on E-STOP.

def test_limit_command_estop_zeros_everything():
    assert safety.limit_command(0.22, 2.0, blocked=False, estop=True,
                                 max_linear=0.22, max_angular=2.0) == (0.0, 0.0)


def test_limit_command_clamps_to_limits_preserving_curvature():
    """REGRESSION (lab 2026-06, 'doesn't follow the RViz path / strange turns'): clamping vx alone
    while wz passed through CHANGED the commanded curvature wz/vx — stock burger.yaml plans up to
    0.3 m/s, the Burger ceiling is 0.22, so every saturated arc was executed ~36% tighter than DWB
    planned (wall-hugging, weaving on real hardware; the ideal-motor sim never saturates and hid
    it). One shared scale factor now caps both components: the arc geometry is preserved and only
    the speed along it drops; the output still never exceeds either limit."""
    vx, wz = safety.limit_command(1.0, 5.0, blocked=False, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert math.isclose(vx, 0.22) and math.isclose(wz, 1.1)   # s = 0.22/1.0 binds
    assert math.isclose(wz / vx, 5.0 / 1.0)                   # curvature wz/vx preserved
    vx, wz = safety.limit_command(-1.0, -5.0, blocked=False, estop=False,
                                  max_linear=0.22, max_angular=2.0)
    assert math.isclose(vx, -0.22) and math.isclose(wz, -1.1)


def test_limit_command_within_limits_is_never_rescaled():
    # The factor only ever REDUCES an over-limit command; in-spec commands pass through bit-exact
    # (Nav2's planned (0.22, 1.0)-class commands must not be touched).
    assert safety.limit_command(0.22, 1.0, blocked=False, estop=False,
                                max_linear=0.22, max_angular=2.84) == (0.22, 1.0)
    assert safety.limit_command(0.0, 2.8, blocked=False, estop=False,
                                max_linear=0.22, max_angular=2.84) == (0.0, 2.8)


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


# ------------------------- combined_range + BlockLatch -----------------------
# REGRESSION (lab 2026-06, 'the safety stop doesn't work'): the gate zeroes only FORWARD motion
# while rotation passes (course rule: rotate/back-up stay allowed). Under a live Nav2 goal, DWB
# keeps commanding arcs; rotating swings the obstacle out of the ±20° front cone, the instant the
# cone reads clear forward re-enables, the robot LURCHES, the obstacle re-enters the cone, blocks
# again… net effect: a stuttering CREEP past the obstacle instead of a visible stop. The latch
# makes the stop sticky: once blocked it releases only after the front range stays beyond a
# release distance (> stop) for a sustained hold — flicker-free, but never blocks on startup.

def test_combined_range_is_min_of_considered_worlds():
    real = _status(front_min=0.40, age_s=0.1)
    sim = _status(front_min=0.30, age_s=0.1)
    assert math.isclose(safety.combined_range(real, sim, max_data_age_s=1.0), 0.30)
    # Staleness still fail-safes through the combined range (stale world -> 0.0).
    stale = _status(front_min=3.0, age_s=5.0)
    assert safety.combined_range(real, stale, max_data_age_s=1.0) == 0.0


def test_gate_and_combined_range_agree():
    real = _status(front_min=0.20)
    sim = _status(has_data=False)
    rng = safety.combined_range(real, sim, max_data_age_s=1.0)
    assert safety.is_blocked(rng, 0.25) is safety.gate(real, sim, 0.25, 1.0) is True


def test_block_latch_blocks_immediately_below_stop():
    latch = safety.BlockLatch(stop_distance_m=0.25, release_distance_m=0.35, release_hold_s=0.3)
    assert latch.update(3.0, now_s=0.0) is False     # clear path: no block
    assert latch.update(0.20, now_s=0.1) is True     # obstacle inside 25 cm: block at once


def test_block_latch_holds_through_hysteresis_band():
    # 0.25..0.35 m is the hysteresis band: NOT enough to release (a flickering cone sits here).
    latch = safety.BlockLatch(0.25, 0.35, 0.3)
    latch.update(0.20, now_s=0.0)
    assert latch.update(0.30, now_s=1.0) is True     # nominally past stop, still held
    assert latch.update(0.30, now_s=9.0) is True     # band never starts the release clock


def test_block_latch_requires_sustained_clearance():
    latch = safety.BlockLatch(0.25, 0.35, 0.3)
    latch.update(0.20, now_s=0.0)                    # block
    assert latch.update(3.0, now_s=0.1) is True      # clear, but hold not yet elapsed
    assert latch.update(3.0, now_s=0.39) is True     # 0.29 s of clearance: still held
    assert latch.update(3.0, now_s=0.45) is False    # >= 0.3 s sustained: released


def test_block_latch_reblocks_and_resets_the_hold_clock():
    latch = safety.BlockLatch(0.25, 0.35, 0.3)
    latch.update(0.20, now_s=0.0)
    latch.update(3.0, now_s=0.2)                     # clearing…
    latch.update(0.20, now_s=0.25)                   # obstacle back: re-block, reset clock
    assert latch.update(3.0, now_s=0.5) is True      # clearance clock restarts HERE
    assert latch.update(3.0, now_s=0.79) is True     # 0.29 s since the restart: still held
    assert latch.update(3.0, now_s=0.81) is False    # 0.31 s sustained: released


def test_block_latch_stale_scan_blocks_and_recovers_like_an_obstacle():
    # A stale considered-scan arrives here as 0.0 (fail-safe) -> latches; recovery is sustained.
    latch = safety.BlockLatch(0.25, 0.35, 0.3)
    assert latch.update(0.0, now_s=0.0) is True
    assert latch.update(3.0, now_s=0.1) is True
    assert latch.update(3.0, now_s=0.5) is False


def test_block_latch_never_blocks_on_startup_inf():
    # Startup no-data is +inf (RULES §B-5: stays unblocked so the robot can warm up).
    latch = safety.BlockLatch(0.25, 0.35, 0.3)
    assert latch.update(INF, now_s=0.0) is False
    assert latch.update(INF, now_s=10.0) is False


def test_block_latch_validates_release_distance():
    with pytest.raises(ValueError):
        safety.BlockLatch(stop_distance_m=0.25, release_distance_m=0.20, release_hold_s=0.3)
