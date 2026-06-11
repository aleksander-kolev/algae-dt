"""Pure-lib tests for lib/occupancy — the static-map goal projection used by mission_runner to
keep navigation goals OUT of the costmap inflation/lethal zone (fixes "bloom near a wall -> robot
starts then does nothing"). No ROS; runs on the host. TDD per PLAN T2.4.

The projection answers one question: given a bloom the operator dropped (possibly hugging a wall),
what is the NEAREST point the robot can actually stand that has >= `clearance` to every obstacle?
That projected point is where Nav2 is sent and where the spray spins. Truly boxed-in -> None (the
mission skips it immediately instead of letting Nav2 churn for the full nav timeout).
"""
from algae_dt.lib import geometry, occupancy, pgm


def _grid(rows: list[str]) -> occupancy.Grid:
    """Build a Grid from an ASCII map: '#' = obstacle, '.' = free. Row-major, row 0 on top."""
    h = len(rows)
    w = len(rows[0])
    free = tuple(ch == '.' for row in rows for ch in row)
    return occupancy.Grid(w, h, free)


# --------------------------------------------------------------- from_pgm
def test_from_pgm_classifies_free_only_below_free_thresh():
    # negate=0 => occ = (255-px)/255, free iff occ < free_thresh (map_server rule).
    #   px=0   -> occ=1.00 -> occupied (not free)
    #   px=128 -> occ=0.498 -> unknown  (not free)
    #   px=255 -> occ=0.00 -> free
    img = pgm.Pgm(width=3, height=1, maxval=255, pixels=bytes([0, 128, 255]))
    grid = occupancy.from_pgm(img, free_thresh=0.196, negate=0)
    assert grid.width == 3 and grid.height == 1
    assert grid.free == (False, False, True)


def test_from_pgm_unknown_value_is_not_free():
    # px=205 is the conventional "unknown" grey: occ=0.196, NOT strictly < free_thresh -> blocked.
    img = pgm.Pgm(width=1, height=1, maxval=255, pixels=bytes([205]))
    grid = occupancy.from_pgm(img, free_thresh=0.196, negate=0)
    assert grid.free == (False,)


def test_from_pgm_negate_inverts_the_scale():
    # negate=1 => occ = px/255: a BLACK pixel is now free, a WHITE one occupied.
    img = pgm.Pgm(width=2, height=1, maxval=255, pixels=bytes([0, 255]))
    grid = occupancy.from_pgm(img, free_thresh=0.196, negate=1)
    assert grid.free == (True, False)


# --------------------------------------------------------------- is_free
def test_is_free_out_of_bounds_is_blocked():
    grid = _grid(["..", ".."])
    assert occupancy.is_free(grid, 0, 0) is True
    assert occupancy.is_free(grid, -1, 0) is False
    assert occupancy.is_free(grid, 0, -1) is False
    assert occupancy.is_free(grid, 2, 0) is False
    assert occupancy.is_free(grid, 0, 2) is False


# --------------------------------------------------------------- clear_radius_ok
def test_clear_radius_ok_true_in_open_interior():
    grid = _grid(["." * 11] * 11)          # all free, 11x11
    assert occupancy.clear_radius_ok(grid, 5, 5, clearance_px=3.0) is True


def test_clear_radius_ok_false_next_to_wall():
    grid = _grid(["." * 11] * 11)
    # the map edge counts as blocked, so a cell 1 px from the edge fails a 2 px clearance
    assert occupancy.clear_radius_ok(grid, 0, 5, clearance_px=2.0) is False


# --------------------------------------------------------------- nearest_clear_cell
def test_nearest_clear_returns_same_cell_when_already_clear():
    grid = _grid(["." * 15] * 15)
    assert occupancy.nearest_clear_cell(grid, 7, 7, clearance_px=3.0, max_radius_px=5.0) == (7, 7)


def test_nearest_clear_projects_off_the_wall():
    rows = ["##########",
            "#........#",
            "#........#",
            "#........#",
            "##########"]
    grid = _grid(rows)
    # (1,1) hugs the top+left walls -> not clear at 1.0 px (a wall cell sits 1.0 px away, within
    # the edge-aware 1.0+0.5 radius); the nearest clear cell must be off the wall.
    res = occupancy.nearest_clear_cell(grid, 1, 1, clearance_px=1.0, max_radius_px=4.0)
    assert res is not None
    assert res != (1, 1)
    assert occupancy.clear_radius_ok(grid, res[0], res[1], clearance_px=1.0)


def test_clear_radius_counts_the_obstacle_cells_edge():
    # An obstacle CELL is a 1x1 square: its edge is up to 0.5 px nearer than its centre, so the
    # disqualifying radius is clearance+0.5. An obstacle centre at 2.24 px (dc=2,dr=1) must FAIL a
    # 2.0 px clearance (2.24 <= 2.5) even though its CENTRE is beyond 2.0 — the centre-only rule
    # let projected goals sit ~half a cell deeper into the wall than promised.
    rows = ["." * 9] * 9
    rows = [list(r) for r in rows]
    rows[5][6] = '#'                       # obstacle at (col=6, row=5)
    grid = _grid([''.join(r) for r in rows])
    assert occupancy.clear_radius_ok(grid, 4, 4, clearance_px=2.0) is False   # 2.24 <= 2.5
    assert occupancy.clear_radius_ok(grid, 4, 4, clearance_px=1.5) is True    # 2.24 >  2.0


def test_nearest_clear_none_when_boxed_in():
    grid = _grid(["###", "#.#", "###"])     # single free pocket, walls on all sides
    assert occupancy.nearest_clear_cell(grid, 1, 1, clearance_px=1.0, max_radius_px=3.0) is None


# --------------------------------------------------------------- reachable_goal (world)
def _map_info() -> geometry.MapInfo:
    return geometry.MapInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width_px=20, height_px=20)


def test_reachable_goal_open_space_returns_near_itself():
    grid = _grid(["." * 20] * 20)
    mi = _map_info()
    # a target well inside the free area projects to ~itself (within one cell)
    gx, gy = 0.5, 0.5
    out = occupancy.reachable_goal(grid, mi, gx, gy, clearance_m=0.10, max_projection_m=0.25)
    assert out is not None
    assert geometry.euclidean(out[0], out[1], gx, gy) <= mi.resolution * 1.5


def test_reachable_goal_near_wall_is_pushed_inward_but_bounded():
    # left two columns are wall; a target just inside must be pushed right, but not arbitrarily far
    rows = ["##" + "." * 18 for _ in range(20)]
    grid = _grid(rows)
    mi = _map_info()
    tx, ty = geometry.pixel_to_world(2, 10, mi)        # the cell immediately right of the wall
    out = occupancy.reachable_goal(grid, mi, tx, ty, clearance_m=0.15, max_projection_m=0.40)
    assert out is not None
    # pushed to the right (greater x in world frame), and within the projection cap of the target
    assert out[0] > tx
    assert geometry.euclidean(out[0], out[1], tx, ty) <= 0.40 + 1e-9


def test_reachable_goal_offmap_target_returns_none():
    grid = _grid(["." * 20] * 20)
    mi = _map_info()
    assert occupancy.reachable_goal(grid, mi, 99.0, 99.0, clearance_m=0.10, max_projection_m=0.25) is None


def test_reachable_goal_offmap_negative_band_returns_none():
    # The sub-cell strip just below/left of the origin: int() truncation used to map (-0.02,-0.02)
    # onto a VALID edge cell and hand Nav2 a goal for an off-map bloom. floor + the explicit bounds
    # check must reject it.
    grid = _grid(["." * 20] * 20)
    mi = _map_info()
    assert occupancy.reachable_goal(grid, mi, -0.02, -0.02,
                                    clearance_m=0.10, max_projection_m=0.25) is None


def test_reachable_goal_nonfinite_target_returns_none():
    """A non-finite goal coordinate is the degenerate 'off-map' case: it must return None (the
    documented contract), not raise — geometry.world_to_pixel would throw on floor(NaN/inf)."""
    grid = _grid(["." * 20] * 20)
    mi = _map_info()
    for bad in (float('nan'), float('inf'), float('-inf')):
        assert occupancy.reachable_goal(grid, mi, bad, 0.5, clearance_m=0.1, max_projection_m=0.25) is None
        assert occupancy.reachable_goal(grid, mi, 0.5, bad, clearance_m=0.1, max_projection_m=0.25) is None


def test_reachable_goal_cap_rejects_existing_but_too_far_clear_cell():
    """The projection cap must be a TRUE upper bound: when the only clearance-satisfying cell exists
    but lies beyond max_projection_m, the bloom is skipped (None), not chased. Prior 'bounded' tests
    realized only a 0.15 m move against a 0.40 m cap, so a broken/absent cap would have passed."""
    # left 10 columns are wall; a target buried at col 2 needs a big push to reach clearance.
    rows = ["#" * 10 + "." * 10 for _ in range(20)]
    grid = _grid(rows)
    mi = _map_info()                                   # 0.05 m/px
    tx, ty = geometry.pixel_to_world(2, 10, mi)
    # tight cap (0.10 m = 2 px) cannot reach the clear interior -> None
    assert occupancy.reachable_goal(grid, mi, tx, ty, clearance_m=0.15, max_projection_m=0.10) is None
    # generous cap (0.80 m = 16 px) can reach it -> a real projected goal
    out = occupancy.reachable_goal(grid, mi, tx, ty, clearance_m=0.15, max_projection_m=0.80)
    assert out is not None and out[0] > tx


# --------------------------------------------------------------- raycast_scan
def _mi(w: int = 10, h: int = 10, res: float = 0.1) -> geometry.MapInfo:
    return geometry.MapInfo(resolution=res, origin_x=0.0, origin_y=0.0, width_px=w, height_px=h)


_WALLED = _grid(['.......#..'] * 10)      # vertical wall at col 7 -> x in [0.7, 0.8)


def test_raycast_hits_the_wall_at_the_right_distance():
    trig = occupancy.beam_trig(1, 0.0, 0.0)         # one beam straight ahead
    (r,) = occupancy.raycast_scan(_WALLED, _mi(), 0.25, 0.55, 0.0, trig=trig,
                                  range_min=0.12, range_max=3.5)
    # wall face at x=0.7, robot at x=0.25 -> 0.45 m, +/- one 0.05 m march step
    assert abs(r - 0.45) <= 0.05 + 1e-9


def test_raycast_open_beam_is_no_return_inf():
    trig = occupancy.beam_trig(1, 0.0, 0.0)
    (r,) = occupancy.raycast_scan(_WALLED, _mi(), 0.25, 0.55, 3.14159265, trig=trig,
                                  range_min=0.12, range_max=3.5)   # facing -x: leaves the map
    assert r == float('inf')


def test_raycast_yaw_rotates_the_beams():
    # same pose, facing +y: the col-7 wall is no longer ahead -> no return
    trig = occupancy.beam_trig(1, 0.0, 0.0)
    (r,) = occupancy.raycast_scan(_WALLED, _mi(), 0.25, 0.55, 1.5707963, trig=trig,
                                  range_min=0.12, range_max=3.5)
    assert r == float('inf')


def test_raycast_blind_spot_reads_zero_like_the_lds02():
    # a hit closer than range_min must read 0.0 ("no return"), the documented LDS-02 blind spot
    trig = occupancy.beam_trig(1, 0.0, 0.0)
    (r,) = occupancy.raycast_scan(_WALLED, _mi(), 0.65, 0.55, 0.0, trig=trig,
                                  range_min=0.12, range_max=3.5)   # wall face only ~0.05 m ahead
    assert r == 0.0


def test_raycast_respects_range_max():
    trig = occupancy.beam_trig(1, 0.0, 0.0)
    (r,) = occupancy.raycast_scan(_WALLED, _mi(), 0.05, 0.55, 0.0, trig=trig,
                                  range_min=0.12, range_max=0.5)   # wall at 0.65 > range_max
    assert r == float('inf')


def test_raycast_multi_beam_geometry_matches_lds_layout():
    # 4 beams from -pi step pi/2 (the LDS layout shape): indices = [back, right, FRONT, left].
    # Facing +x with the wall ahead: only the FRONT beam (index 2) sees it; right/left exit the
    # arena; the back beam exits at x<0.
    trig = occupancy.beam_trig(4, -3.14159265, 1.5707963)
    rr = occupancy.raycast_scan(_WALLED, _mi(), 0.25, 0.55, 0.0, trig=trig,
                                range_min=0.12, range_max=3.5)
    assert rr[0] == float('inf') and rr[1] == float('inf') and rr[3] == float('inf')
    assert abs(rr[2] - 0.45) <= 0.05 + 1e-9


def test_raycast_on_the_real_course_map_sees_walls():
    # the actual 86x110 course map: from the spawn (0,0) facing +x SOME beams must return finite
    # wall hits and they must respect [range_min, range_max] (proves the inlined pixel transform
    # agrees with the real map geometry end-to-end)
    from algae_dt.lib.ros_utils import load_package_map

    try:
        img = load_package_map()
    except Exception:
        import pytest
        pytest.skip("installed course map not available on this host")
    grid = occupancy.from_pgm(img)
    mi = geometry.MapInfo(resolution=0.05, origin_x=-2.051, origin_y=-4.194,
                          width_px=86, height_px=110)
    trig = occupancy.beam_trig(360, -3.14159265, 2 * 3.14159265 / 360)
    rr = occupancy.raycast_scan(grid, mi, 0.0, 0.0, 0.0, trig=trig,
                                range_min=0.12, range_max=3.5)
    finite = [r for r in rr if r != float('inf') and r > 0.0]
    assert len(finite) > 90, "from the arena centre most beams should hit walls"
    assert all(0.12 <= r <= 3.5 + 1e-9 for r in finite)
    assert len(set(round(r, 2) for r in finite)) > 5, "a real arena is not a uniform ring"
