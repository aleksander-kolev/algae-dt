"""TDD for lib/trajectory.py — the pure dynamic-obstacle path (PLAN T6.1, Rubric III)."""
import math

from algae_dt.lib import trajectory as tr


def test_oscillate_starts_at_center():
    assert tr.oscillate(0.0, 1.0, 2.0, amplitude=0.5, period=4.0, axis='y') == (1.0, 2.0)


def test_oscillate_quarter_period_is_full_amplitude_on_y():
    x, y = tr.oscillate(1.0, 1.0, 2.0, amplitude=0.5, period=4.0, axis='y')   # t = period/4
    assert math.isclose(x, 1.0) and math.isclose(y, 2.5)


def test_oscillate_axis_x():
    x, y = tr.oscillate(1.0, 1.0, 2.0, amplitude=0.5, period=4.0, axis='x')
    assert math.isclose(x, 1.5) and math.isclose(y, 2.0)


def test_oscillate_zero_period_is_static():
    assert tr.oscillate(3.3, 1.0, 2.0, amplitude=0.5, period=0.0, axis='y') == (1.0, 2.0)


# ------------------------- keep-out (teleported demo box) -------------------------

def test_sweep_clearance_point_beside_the_swept_segment():
    # Sweep along y about (1,0) +/-1: closest approach to the origin is the x distance, 1.0.
    assert math.isclose(tr.sweep_clearance(0.0, 0.0, 1.0, 0.0, 1.0, 'y'), 1.0)


def test_sweep_clearance_point_beyond_the_segment_end():
    # Sweep along x about (2,0) +/-0.5 -> segment [1.5, 2.5]; from the origin the nearest end is 1.5.
    assert math.isclose(tr.sweep_clearance(0.0, 0.0, 2.0, 0.0, 0.5, 'x'), 1.5)


def test_sweep_clearance_zero_amplitude_is_point_distance():
    assert math.isclose(tr.sweep_clearance(0.0, 0.0, 3.0, 4.0, 0.0, 'y'), 5.0)


def test_clamp_amplitude_passes_a_safe_sweep_through():
    assert tr.clamp_amplitude_for_keepout(0.0, 0.0, 0.6, 0.0, 0.6, 'y', 0.30) == 0.6


def test_clamp_amplitude_shrinks_an_unsafe_sweep():
    # Sweep along x about (0.6,0) +/-0.6 reaches x=0.0 (the robot spawn). Clamped so the swept
    # segment keeps >= 0.30 m from the origin: max amplitude = 0.6 - 0.30 = 0.30.
    out = tr.clamp_amplitude_for_keepout(0.0, 0.0, 0.6, 0.0, 0.6, 'x', 0.30)
    assert out is not None
    assert math.isclose(out, 0.30, abs_tol=1e-6)
    assert tr.sweep_clearance(0.0, 0.0, 0.6, 0.0, out, 'x') >= 0.30 - 1e-9


def test_clamp_amplitude_refuses_a_centre_inside_the_keepout():
    assert tr.clamp_amplitude_for_keepout(0.0, 0.0, 0.1, 0.0, 0.6, 'y', 0.30) is None


def test_clamp_negative_amplitude_still_enforces_keepout():
    """-A and +A sweep the IDENTICAL segment, but the old clamp returned a value LARGER than a
    negative input - the caller's `clamped < amplitude` guard then discarded the clamp entirely
    and the box swept straight through the keep-out (onto the robot spawn). The clamp now
    normalizes to the magnitude at the boundary."""
    a = tr.clamp_amplitude_for_keepout(0.0, 0.0, 0.6, 0.0, -0.6, 'x', 0.30)
    assert a is not None and 0.0 <= a <= 0.6, "clamp must return a magnitude <= |amplitude|"
    assert tr.sweep_clearance(0.0, 0.0, 0.6, 0.0, a, 'x') >= 0.30 - 1e-6, \
        "the clamped sweep must keep the keep-out clearance"
