"""Pure bloom bookkeeping (no ROS), IMMUTABLE. STUB — TDD in PLAN.md T2.1.

Design (ported): a frozen Bloom (id, x, y, radius, state) and a frozen BloomField with pure
transitions that RETURN NEW objects (never mutate):
- add(field, bloom) / clear(field)
- mark_treated(field, id) / mark_skipped(field, id) / set_active(field, id)
- nearest_untreated(field, x, y) -> Bloom | None
Tests must assert: immutability (original unchanged), a half-treated bloom is NOT skippable by
nearest_untreated, treated/skipped are terminal.
"""
from __future__ import annotations

from dataclasses import dataclass

PENDING, ACTIVE, TREATED, SKIPPED = 'pending', 'active', 'treated', 'skipped'


@dataclass(frozen=True)
class Bloom:
    id: int
    x: float
    y: float
    radius: float = 0.15
    state: str = PENDING


def nearest_untreated(*args, **kwargs):  # noqa: D401 - stub
    raise NotImplementedError("PLAN.md T2.1 — implement with a failing test first")
