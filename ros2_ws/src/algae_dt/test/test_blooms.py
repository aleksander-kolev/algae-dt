"""TDD for lib/blooms.py — immutable bloom bookkeeping (PLAN T2.1, RULES §C immutability).

Pins: transitions RETURN NEW objects (never mutate), treated/skipped are TERMINAL, and an
in-progress ("half-treated", i.e. ACTIVE) bloom is NOT skipped by nearest_untreated — an aborted
spray reverts to PENDING and is re-selectable (honest accounting, BEST_APPROACHES §Honest task
accounting).
"""
import pytest

from algae_dt.lib import blooms as B


def _field(*xy):
    f = B.BloomField()
    for i, (x, y) in enumerate(xy):
        f = B.add(f, B.Bloom(id=i, x=x, y=y))
    return f


def test_bloom_defaults():
    b = B.Bloom(id=1, x=0.5, y=-0.5)
    assert b.radius == 0.15 and b.state == B.PENDING


def test_add_returns_new_field_original_unchanged():
    f0 = B.BloomField()
    f1 = B.add(f0, B.Bloom(id=0, x=1.0, y=2.0))
    assert len(f0.blooms) == 0      # original untouched
    assert len(f1.blooms) == 1 and f1.blooms[0].id == 0


def test_clear_empties_without_mutating_original():
    f1 = _field((1.0, 1.0), (2.0, 2.0))
    f2 = B.clear(f1)
    assert len(f1.blooms) == 2      # original untouched
    assert len(f2.blooms) == 0


def test_mark_treated_is_immutable():
    f1 = _field((1.0, 0.0))
    f2 = B.mark_treated(f1, 0)
    assert f1.blooms[0].state == B.PENDING     # original bloom unchanged
    assert f2.blooms[0].state == B.TREATED


def test_mark_skipped_sets_state():
    f2 = B.mark_skipped(_field((1.0, 0.0)), 0)
    assert f2.blooms[0].state == B.SKIPPED


def test_set_active_then_revert_to_pending():
    f = _field((1.0, 0.0))
    f = B.set_active(f, 0)
    assert f.blooms[0].state == B.ACTIVE
    f = B.set_pending(f, 0)                     # aborted spray -> back to pending
    assert f.blooms[0].state == B.PENDING


def test_nearest_untreated_returns_closest_pending():
    # blooms at (1,0) id0 and (5,0) id1; from (0,0) nearest is id0.
    f = _field((1.0, 0.0), (5.0, 0.0))
    assert B.nearest_untreated(f, 0.0, 0.0).id == 0


def test_nearest_untreated_skips_terminal_states():
    f = _field((1.0, 0.0), (5.0, 0.0))
    f = B.mark_treated(f, 0)                    # nearest is now terminal
    nxt = B.nearest_untreated(f, 0.0, 0.0)
    assert nxt is not None and nxt.id == 1      # so the far PENDING one is chosen
    f = B.mark_skipped(f, 1)
    assert B.nearest_untreated(f, 0.0, 0.0) is None


def test_nearest_untreated_includes_active_not_skipped():
    # An ACTIVE (half-treated) bloom must remain selectable, not be skipped over.
    f = _field((1.0, 0.0), (5.0, 0.0))
    f = B.set_active(f, 0)
    assert B.nearest_untreated(f, 0.0, 0.0).id == 0


def test_nearest_untreated_none_when_all_terminal():
    f = _field((1.0, 0.0))
    f = B.mark_treated(f, 0)
    assert B.nearest_untreated(f, 0.0, 0.0) is None


def test_nearest_untreated_empty_field_is_none():
    assert B.nearest_untreated(B.BloomField(), 0.0, 0.0) is None


def test_mark_unknown_id_raises():
    with pytest.raises(KeyError):
        B.mark_treated(_field((1.0, 0.0)), 99)


def test_counts_summarises_states():
    f = _field((1.0, 0.0), (2.0, 0.0), (3.0, 0.0))
    f = B.mark_treated(f, 0)
    f = B.set_active(f, 1)
    c = B.counts(f)
    assert c[B.TREATED] == 1 and c[B.ACTIVE] == 1 and c[B.PENDING] == 1
