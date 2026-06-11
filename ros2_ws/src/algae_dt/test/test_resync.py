"""TDD for lib/resync.py — the bounded-drift resync policy (Rubric ②).

The policy decides when the twin may snap the sim back onto the real robot ("predict with the
model, correct with the data"). Every rule is pinned here BEFORE the node wiring:
  * manual fires immediately (even in tolerance), auto only after a SUSTAINED out-of-tolerance,
  * cooldown spaces attempts (manual included) so nothing can teleport-storm the sim,
  * stale inputs / NaN errors never fire (never teleport onto data you don't trust),
  * the state is immutable (step returns a new state; the input is never mutated).
"""
import math

from algae_dt.lib import resync as R

NAN = float('nan')


def _step(state, **kw):
    defaults = dict(now=0.0, dxy=0.0, dyaw=0.0, tol_xy=0.15, tol_yaw=0.26,
                    sustain_s=5.0, cooldown_s=8.0, auto_enable=True,
                    manual_requested=False, inputs_fresh=True)
    defaults.update(kw)
    return R.step(state, **defaults)


def test_manual_fires_immediately_even_in_tolerance():
    st, trig = _step(R.ResyncState(), now=1.0, dxy=0.0, dyaw=0.0, manual_requested=True)
    assert trig == R.MANUAL
    assert st.last_attempt_t == 1.0


def test_manual_requires_fresh_inputs():
    st, trig = _step(R.ResyncState(), now=1.0, manual_requested=True, inputs_fresh=False)
    assert trig is None, "never teleport onto a target the data doesn't support"
    assert st.last_attempt_t is None


def test_auto_fires_only_after_sustained_out_of_tolerance():
    st = R.ResyncState()
    st, trig = _step(st, now=0.0, dxy=0.30)          # breach begins
    assert trig is None and st.out_since == 0.0
    st, trig = _step(st, now=4.9, dxy=0.30)          # still inside the sustain window
    assert trig is None
    st, trig = _step(st, now=5.0, dxy=0.30)          # sustained -> fire
    assert trig == R.AUTO
    assert st.last_attempt_t == 5.0 and st.out_since is None


def test_transient_recovery_resets_the_sustain_clock():
    st = R.ResyncState()
    st, _ = _step(st, now=0.0, dxy=0.30)
    st, _ = _step(st, now=3.0, dxy=0.05)             # back in tolerance: clock resets
    assert st.out_since is None
    st, trig = _step(st, now=4.0, dxy=0.30)          # breach restarts at t=4
    assert trig is None
    st, trig = _step(st, now=8.9, dxy=0.30)          # 4.9 s < sustain
    assert trig is None
    st, trig = _step(st, now=9.0, dxy=0.30)          # 5.0 s sustained -> fire
    assert trig == R.AUTO


def test_cooldown_blocks_both_triggers_then_releases():
    st, trig = _step(R.ResyncState(), now=0.0, manual_requested=True)
    assert trig == R.MANUAL
    st, trig = _step(st, now=1.0, manual_requested=True)
    assert trig is None, "manual inside the cooldown must not fire"
    st, trig = _step(st, now=2.0, dxy=0.30)
    st, trig = _step(st, now=7.9, dxy=0.30)          # sustained > sustain_s but still cooling down
    assert trig is None, "auto inside the cooldown must not fire"
    st, trig = _step(st, now=8.0, dxy=0.30)          # cooldown over + sustain satisfied
    assert trig == R.AUTO


def test_auto_disabled_never_fires_auto_but_manual_still_works():
    st = R.ResyncState()
    for t in (0.0, 5.0, 60.0):
        st, trig = _step(st, now=t, dxy=0.50, auto_enable=False)
        assert trig is None
    st, trig = _step(st, now=61.0, dxy=0.50, auto_enable=False, manual_requested=True)
    assert trig == R.MANUAL


def test_nan_error_is_not_out_of_tolerance():
    st = R.ResyncState()
    for t in (0.0, 10.0, 20.0):
        st, trig = _step(st, now=t, dxy=NAN, dyaw=NAN)
        assert trig is None
        assert st.out_since is None, "NaN must never count as a breach (NaN > tol is False)"


def test_stale_inputs_reset_the_sustain_clock():
    st = R.ResyncState()
    st, _ = _step(st, now=0.0, dxy=0.30)
    st, trig = _step(st, now=4.5, dxy=0.30, inputs_fresh=False)   # stream gap mid-sustain
    assert trig is None and st.out_since is None
    st, trig = _step(st, now=5.0, dxy=0.30)                       # breach measured anew from t=5
    assert trig is None
    st, trig = _step(st, now=10.0, dxy=0.30)
    assert trig == R.AUTO


def test_yaw_only_breach_counts():
    st = R.ResyncState()
    st, _ = _step(st, now=0.0, dyaw=0.50)
    st, trig = _step(st, now=5.0, dyaw=0.50)
    assert trig == R.AUTO


def test_step_never_mutates_the_input_state():
    st0 = R.ResyncState(out_since=1.0, last_attempt_t=None)
    _step(st0, now=2.0, dxy=0.30, manual_requested=True)
    assert st0 == R.ResyncState(out_since=1.0, last_attempt_t=None), \
        "step must return a NEW state, never mutate (immutability rule)"


def test_fire_resets_out_since_and_stamps_the_attempt():
    st = R.ResyncState(out_since=0.0)
    st, trig = _step(st, now=6.0, dxy=0.30)
    assert trig == R.AUTO
    assert st == R.ResyncState(out_since=None, last_attempt_t=6.0)


def test_infinite_error_counts_as_breach():
    st = R.ResyncState()
    st, _ = _step(st, now=0.0, dxy=math.inf)
    st, trig = _step(st, now=5.0, dxy=math.inf)
    assert trig == R.AUTO
