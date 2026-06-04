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
    # negate=0 => occ = (255-px)/255. occupied_thresh=0.65, free_thresh=0.196.
    #   px=0   -> occ=1.00 -> occupied (not free)
    #   px=128 -> occ=0.498 -> unknown  (not free)
    #   px=255 -> occ=0.00 -> free
    img = pgm.Pgm(width=3, height=1, maxval=255, pixels=bytes([0, 128, 255]))
    grid = occupancy.from_pgm(img, occupied_thresh=0.65, free_thresh=0.196, negate=0)
    assert grid.width == 3 and grid.height == 1
    assert grid.free == (False, False, True)


def test_from_pgm_unknown_value_is_not_free():
    # px=205 is the conventional "unknown" grey: occ=0.196, NOT strictly < free_thresh -> blocked.
    img = pgm.Pgm(width=1, height=1, maxval=255, pixels=bytes([205]))
    grid = occupancy.from_pgm(img, occupied_thresh=0.65, free_thresh=0.196, negate=0)
    assert grid.free == (False,)


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
    # (1,1) hugs the top+left walls -> not clear at 1.5 px; the nearest clear cell must be off the wall
    res = occupancy.nearest_clear_cell(grid, 1, 1, clearance_px=1.5, max_radius_px=4.0)
    assert res is not None
    assert res != (1, 1)
    assert occupancy.clear_radius_ok(grid, res[0], res[1], clearance_px=1.5)


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
