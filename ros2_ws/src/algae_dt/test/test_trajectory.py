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
