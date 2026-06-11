"""TDD for lib/scanmerge.py — virtual obstacles must reach the REAL robot's navigation.

USER-REPORTED GAP (lab 2026-06): placing an obstacle in the Gazebo twin did not change the real
robot's nav — Nav2's costmaps read only the real /scan, so a sim-only obstacle existed for the
25 cm gate (last-second veto, ±20° cone) but never for path planning. The fix: the mediator
publishes /dt/scan_nav — the REAL scan with the TRUSTED mirror's returns overlaid (per-beam MIN,
matched BY ANGLE, never by index: the LDS-02 starts at angle 0, the gz lidar at -pi) — and the
launch points every Nav2 obstacle source (costmap layers + collision_monitor) at it. AMCL stays
on the pure real /scan: localization must never see virtual obstacles.

Merging is CONSERVATIVE: a virtual return can only SHORTEN a beam (add an obstacle), never erase
a real one; invalid beams on either side never poison the other side's data.
"""
import math

from algae_dt.lib import scanmerge

INF = float('inf')


def _merge(real, sim, real_angle_min=0.0, sim_angle_min=0.0, n=None,
           sim_range_min=0.12, sim_range_max=3.5):
    n_r, n_s = len(real), len(sim)
    return scanmerge.merge_ranges(
        real, real_angle_min, 2.0 * math.pi / max(n_r, 1),
        sim, sim_angle_min, 2.0 * math.pi / max(n_s, 1),
        sim_range_min=sim_range_min, sim_range_max=sim_range_max)


def test_identity_when_sim_sees_nothing():
    real = [2.0, 2.5, 3.0, 2.0]
    sim = [INF, INF, INF, INF]
    assert _merge(real, sim) == real


def test_virtual_obstacle_shortens_the_matching_beam():
    real = [3.0] * 8
    sim = [3.0] * 8
    sim[2] = 1.0                       # virtual box at beam angle 2*(2pi/8)
    out = _merge(real, sim)
    assert out[2] == 1.0
    assert out[:2] == [3.0, 3.0] and out[3:] == [3.0] * 5


def test_merge_matches_by_angle_not_index():
    # Real scan starts at 0 rad (LDS-02), sim scan starts at -pi (gz convention): the sim's
    # index-0 beam points BACKWARD relative to the real index-0 beam. A virtual return on the
    # sim beam pointing at +pi/2 must land on the REAL beam pointing at +pi/2 (index n/4),
    # not on index n/2+n/4 where naive index-copy would put it.
    n = 8
    real = [3.0] * n
    sim = [INF] * n
    sim[6] = 1.0                       # -pi + 6*(2pi/8) = +pi/2
    out = _merge(real, sim, real_angle_min=0.0, sim_angle_min=-math.pi)
    assert out[2] == 1.0               # 0 + 2*(2pi/8) = +pi/2
    assert sum(1 for r in out if r != 3.0) == 1


def test_real_beam_never_lengthened_by_sim():
    real = [0.5] * 4                   # real obstacle is CLOSER than the sim's return
    sim = [2.0] * 4
    assert _merge(real, sim) == [0.5] * 4


def test_invalid_real_beam_still_carries_the_virtual_return():
    # The real LDS-02 reports no-return as 0.0 (invalid) or inf. A virtual obstacle on that beam
    # must still be marked — min() over the RAW values would wrongly keep 0.0 and lose it.
    real = [0.0, INF, float('nan'), 3.0]
    sim = [1.0, 1.0, 1.0, 1.0]
    out = _merge(real, sim)
    assert out[0] == 1.0 and out[1] == 1.0 and out[2] == 1.0 and out[3] == 1.0


def test_invalid_sim_beams_never_poison_real_data():
    real = [3.0, 3.0, 3.0, 3.0]
    sim = [0.0, float('nan'), INF, 0.05]   # 0.05 < sim_range_min: sub-spec noise
    assert _merge(real, sim) == real


def test_original_invalid_sentinel_preserved_when_both_invalid():
    real = [0.0, INF]
    sim = [INF, 0.0]
    out = _merge(real, sim)
    assert out[0] == 0.0 and out[1] == INF   # untouched, sentinel semantics preserved


def test_different_beam_counts_resample_by_angle():
    n_r, n_s = 8, 4
    real = [3.0] * n_r
    sim = [INF] * n_s
    sim[1] = 1.0                       # sim beam at 1*(2pi/4) = pi/2
    out = scanmerge.merge_ranges(real, 0.0, 2.0 * math.pi / n_r,
                                 sim, 0.0, 2.0 * math.pi / n_s,
                                 sim_range_min=0.12, sim_range_max=3.5)
    assert out[2] == 1.0               # real beam at 2*(2pi/8) = pi/2


def test_empty_inputs_are_safe():
    assert scanmerge.merge_ranges([], 0.0, 0.1, [1.0], 0.0, 0.1,
                                  sim_range_min=0.12, sim_range_max=3.5) == []
    real = [2.0, 2.0]
    assert scanmerge.merge_ranges(real, 0.0, math.pi, [], 0.0, 0.1,
                                  sim_range_min=0.12, sim_range_max=3.5) == real
