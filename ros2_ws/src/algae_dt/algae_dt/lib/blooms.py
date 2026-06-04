"""Pure bloom bookkeeping (no ROS), IMMUTABLE. Implemented TDD per PLAN T2.1.

A frozen `Bloom` (id, x, y, radius, state) and a frozen `BloomField` whose transitions RETURN NEW
objects and never mutate (RULES §C). `treated`/`skipped` are terminal; an in-progress `active`
("half-treated") bloom is still selectable by `nearest_untreated`, and an aborted spray reverts to
`pending` via `set_pending` — so a half-treated bloom is never wrongly skipped (honest accounting,
BEST_APPROACHES). Tests: test/test_blooms.py.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from algae_dt.lib.geometry import euclidean

PENDING, ACTIVE, TREATED, SKIPPED = 'pending', 'active', 'treated', 'skipped'
TERMINAL = (TREATED, SKIPPED)            # states nearest_untreated will never return


@dataclass(frozen=True)
class Bloom:
    id: int
    x: float
    y: float
    radius: float = 0.15
    state: str = PENDING


@dataclass(frozen=True)
class BloomField:
    """An immutable collection of blooms (order preserved)."""
    blooms: tuple[Bloom, ...] = ()


def add(field: BloomField, bloom: Bloom) -> BloomField:
    return BloomField(field.blooms + (bloom,))


def clear(field: BloomField) -> BloomField:
    return BloomField(())


def _set_state(field: BloomField, bloom_id: int, state: str) -> BloomField:
    found = False
    out = []
    for b in field.blooms:
        if b.id == bloom_id:
            out.append(replace(b, state=state))
            found = True
        else:
            out.append(b)
    if not found:
        raise KeyError(f"no bloom with id={bloom_id}")
    return BloomField(tuple(out))


def mark_treated(field: BloomField, bloom_id: int) -> BloomField:
    return _set_state(field, bloom_id, TREATED)


def mark_skipped(field: BloomField, bloom_id: int) -> BloomField:
    return _set_state(field, bloom_id, SKIPPED)


def set_active(field: BloomField, bloom_id: int) -> BloomField:
    return _set_state(field, bloom_id, ACTIVE)


def set_pending(field: BloomField, bloom_id: int) -> BloomField:
    return _set_state(field, bloom_id, PENDING)


def retry_skipped(field: BloomField) -> BloomField:
    """Reset SKIPPED blooms to PENDING so a fresh Start retries previously-failed targets; TREATED
    stays terminal (honest accounting). Returns a new field (immutable). Used by mission restart."""
    return BloomField(tuple(replace(b, state=PENDING) if b.state == SKIPPED else b
                            for b in field.blooms))


def by_id(field: BloomField, bloom_id: int) -> Bloom | None:
    for b in field.blooms:
        if b.id == bloom_id:
            return b
    return None


def nearest_untreated(field: BloomField, x: float, y: float) -> Bloom | None:
    """Nearest non-terminal bloom (pending OR active) to (x, y); None if none remain."""
    best: Bloom | None = None
    best_d = float('inf')
    for b in field.blooms:
        if b.state in TERMINAL:
            continue
        d = euclidean(x, y, b.x, b.y)
        if d < best_d:
            best_d, best = d, b
    return best
