"""TDD for lib/sync.py — pure real<->sim synchronisation math (PLAN T3.1, Rubric ②).

Thresholds referenced are the documented ones in config/twin.yaml (tol_pose_xy_m=0.15,
tol_pose_yaw_rad=0.26, tol_sensor_range_m=0.20). Also covers the sim_only "commanded shadow pose"
unicycle integrator that lets pillar ② be demonstrated without a real robot.
"""
import math

from algae_dt.lib import sync as S

INF = float('inf')
TOL_XY, TOL_YAW, TOL_SENSOR = 0.15, 0.26, 0.20


# --------------------------------- pose_error ------------------------------

def test_pose_error_xy_and_yaw():
    dxy, dyaw = S.pose_error((0.0, 0.0, 0.0), (0.3, 0.4, 0.5))
    assert math.isclose(dxy, 0.5)           # 3-4-5 triangle
    assert math.isclose(dyaw, 0.5)


def test_pose_error_yaw_wraps_shortest():
    # real just below +pi, sim just above -pi: true difference is 0.10 rad, not ~2*pi.
    dxy, dyaw = S.pose_error((0.0, 0.0, math.pi - 0.05), (0.0, 0.0, -math.pi + 0.05))
    assert math.isclose(dyaw, 0.10, abs_tol=1e-9)


# -------------------------------- sensor_error -----------------------------

def test_sensor_error_basic_abs_diff():
    assert math.isclose(S.sensor_error(1.0, 1.3), 0.3)


def test_sensor_error_both_inf_agree_zero():
    assert S.sensor_error(INF, INF) == 0.0


def test_sensor_error_one_inf_is_inf():
    assert S.sensor_error(1.0, INF) == INF
    assert S.sensor_error(INF, 1.0) == INF


# ------------------------------ within_tolerance ---------------------------

def test_within_tolerance_true_at_thresholds():
    err = S.SyncError(dxy=0.15, dyaw=0.26, sensor=0.20)   # exactly at limits => still OK
    assert S.within_tolerance(err, TOL_XY, TOL_YAW, TOL_SENSOR) is True


def test_within_tolerance_false_when_xy_exceeds():
    err = S.SyncError(dxy=0.16, dyaw=0.0, sensor=0.0)
    assert S.within_tolerance(err, TOL_XY, TOL_YAW, TOL_SENSOR) is False


def test_within_tolerance_false_when_yaw_exceeds():
    err = S.SyncError(dxy=0.0, dyaw=0.30, sensor=0.0)
    assert S.within_tolerance(err, TOL_XY, TOL_YAW, TOL_SENSOR) is False


def test_within_tolerance_false_when_sensor_exceeds():
    err = S.SyncError(dxy=0.0, dyaw=0.0, sensor=0.25)
    assert S.within_tolerance(err, TOL_XY, TOL_YAW, TOL_SENSOR) is False


def test_compute_builds_syncerror():
    err = S.compute((0.0, 0.0, 0.0), (0.3, 0.4, 0.0), 1.0, 1.1)
    assert math.isclose(err.dxy, 0.5) and math.isclose(err.sensor, 0.1, abs_tol=1e-9)


def test_compute_wires_dyaw_into_syncerror():
    """The one compute() test fed identical yaws (0,0), so dyaw=0 and its wiring into SyncError was
    never asserted — a regression dropping dyaw would pass. Feed differing yaws and pin dyaw."""
    err = S.compute((0.0, 0.0, 0.0), (0.0, 0.0, 0.30), 1.0, 1.0)
    assert math.isclose(err.dyaw, 0.30, abs_tol=1e-9)
    assert err.sensor == 0.0


# ----------------------- commanded shadow-pose integrator ------------------

def test_integrate_unicycle_straight_line():
    x, y, yaw = S.integrate_unicycle(0.0, 0.0, 0.0, v=1.0, omega=0.0, dt=2.0)
    assert math.isclose(x, 2.0) and math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_integrate_unicycle_pure_rotation_in_place():
    x, y, yaw = S.integrate_unicycle(1.0, 2.0, 0.0, v=0.0, omega=1.0, dt=math.pi / 2)
    assert math.isclose(x, 1.0) and math.isclose(y, 2.0)
    assert math.isclose(yaw, math.pi / 2)


def test_integrate_unicycle_wraps_yaw():
    # spinning past +pi wraps into (-pi, pi].
    _, _, yaw = S.integrate_unicycle(0.0, 0.0, math.pi - 0.1, v=0.0, omega=1.0, dt=0.3)
    assert -math.pi < yaw <= math.pi
    assert math.isclose(yaw, -(math.pi - 0.2), abs_tol=1e-9)


def test_integrate_unicycle_curved_arc():
    """The v!=0 AND omega!=0 branch (the exact-arc kinematics — the only nontrivial path) was never
    exercised: prior tests were straight-line (omega=0) or in-place (v=0). A quarter circle of
    radius 1 from the origin heading +x ends at (1,1) heading +pi/2."""
    x, y, yaw = S.integrate_unicycle(0.0, 0.0, 0.0, v=1.0, omega=1.0, dt=math.pi / 2)
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 1.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)


# ------------------------------ RTF compensation ----------------------------
# REGRESSION (lab 2026-06, 'the twin doesn't take the path the real one takes'): the `both` mirror
# follows the gated commands OPEN-LOOP, but Gazebo integrates them in SIM time. At real-time-factor
# RTF < 1 a command held for one wall second covers only RTF of the real robot's ground — the
# mirror under-travels and turns in the wrong places, structurally, until a resync snaps it. The
# mediator measures RTF from /clock-vs-wall and scales the SIM fan-out by 1/RTF (clamped) so the
# mirror covers the same ground per WALL second as the real Burger.

def test_rtf_estimate_steady_half_speed():
    samples = [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)]   # (wall_s, sim_s)
    assert math.isclose(S.rtf_estimate(samples), 0.5)


def test_rtf_estimate_insufficient_span_is_none():
    assert S.rtf_estimate([]) is None
    assert S.rtf_estimate([(0.0, 0.0)]) is None
    assert S.rtf_estimate([(0.0, 0.0), (0.3, 0.3)]) is None          # < default 0.5 s wall span


def test_rtf_estimate_paused_sim_is_none():
    # A paused sim makes no sim-time progress: no estimate (factor falls back to 1.0), never a
    # divide-by-zero or a huge bogus factor.
    assert S.rtf_estimate([(0.0, 5.0), (2.0, 5.0)]) is None


def test_rtf_compensation_speeds_up_a_slow_sim_clamped():
    assert math.isclose(S.rtf_compensation(0.5, max_factor=3.0), 2.0)
    assert math.isclose(S.rtf_compensation(0.1, max_factor=3.0), 3.0)   # clamped


def test_rtf_compensation_slows_a_fast_sim_clamped():
    assert math.isclose(S.rtf_compensation(2.0, max_factor=3.0), 0.5)
    assert math.isclose(S.rtf_compensation(10.0, max_factor=3.0), 1.0 / 3.0)   # clamped


def test_rtf_compensation_unknown_or_degenerate_is_unity():
    assert S.rtf_compensation(None, max_factor=3.0) == 1.0
    assert S.rtf_compensation(0.0, max_factor=3.0) == 1.0
    assert S.rtf_compensation(-1.0, max_factor=3.0) == 1.0


def test_rtf_compensation_at_unity_rtf_is_a_no_op():
    assert math.isclose(S.rtf_compensation(1.0, max_factor=3.0), 1.0)
