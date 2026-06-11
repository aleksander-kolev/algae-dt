"""Pure resync policy (no ROS): when may the twin snap the sim back onto the real robot?

The `both`-mode sim mirrors the gated commands open-loop (twin_mediator fan-out), so its pose
error relative to the real Burger ACCUMULATES — measured and alerted by sync_supervisor, but
unbounded. This policy is the missing "correct" half of the predict/correct loop: it decides when
a correction (gz set_pose teleport, executed by twin_resync) is justified.

Rules (each pinned by test/test_resync.py):
  * MANUAL fires immediately, even in tolerance (e.g. squaring up before a demo take).
  * AUTO fires only after the pose error has been CONTINUOUSLY out of tolerance for sustain_s —
    a transient spike during a spray spin must not teleport the twin.
  * cooldown_s spaces resync ATTEMPTS (manual and auto alike): a failing gz call or a hammered
    RESYNC button can never teleport-storm the sim.
  * Stale inputs never fire and reset the sustain clock — never teleport onto a target the data
    doesn't support. NaN errors are not "out of tolerance" (NaN > tol is False by design).

Immutable: step() returns a NEW state; the caller owns the single current state.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

MANUAL = 'manual'
AUTO = 'auto'


@dataclass(frozen=True)
class ResyncState:
    out_since: float | None = None       # when the error became (and stayed) out of tolerance
    last_attempt_t: float | None = None  # when the last resync attempt fired (None = never)


def step(state: ResyncState, *, now: float, dxy: float, dyaw: float,
         tol_xy: float, tol_yaw: float, sustain_s: float, cooldown_s: float,
         auto_enable: bool, manual_requested: bool, inputs_fresh: bool,
         ) -> tuple[ResyncState, str | None]:
    """One policy tick -> (new_state, trigger) with trigger in {MANUAL, AUTO, None}."""
    if not inputs_fresh:
        # No trustworthy target/error: never fire, and restart the sustain measurement.
        return replace(state, out_since=None), None

    out = (dxy > tol_xy) or (dyaw > tol_yaw)          # NaN compares False on purpose
    out_since = (state.out_since if (out and state.out_since is not None)
                 else (now if out else None))

    in_cooldown = (state.last_attempt_t is not None
                   and (now - state.last_attempt_t) < cooldown_s)
    if not in_cooldown:
        if manual_requested:
            return ResyncState(out_since=None, last_attempt_t=now), MANUAL
        if auto_enable and out_since is not None and (now - out_since) >= sustain_s:
            return ResyncState(out_since=None, last_attempt_t=now), AUTO
    return replace(state, out_since=out_since), None
