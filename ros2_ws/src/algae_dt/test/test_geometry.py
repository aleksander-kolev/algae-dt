"""TDD for lib/geometry.py — the planar quaternion<->yaw pair + transform coverage (PLAN T2.1)."""
import math

from algae_dt.lib import geometry as g

MAP = g.MapInfo(resolution=0.05, origin_x=-2.051, origin_y=-4.194, width_px=86, height_px=110)


def test_quaternion_from_yaw_zero():
    z, w = g.quaternion_from_yaw(0.0)
    assert math.isclose(z, 0.0) and math.isclose(w, 1.0)


def test_quaternion_yaw_round_trip():
    for yaw in (0.0, 0.5, math.pi / 2, -1.2, math.pi - 0.01):
        z, w = g.quaternion_from_yaw(yaw)
        assert math.isclose(g.yaw_from_quaternion(z, w), yaw, abs_tol=1e-9)


def test_world_pixel_round_trip_nonorigin():
    col, row = g.world_to_pixel(1.0, -1.0, MAP)
    x, y = g.pixel_to_world(col, row, MAP)
    assert abs(x - 1.0) < MAP.resolution and abs(y - (-1.0)) < MAP.resolution


def test_world_to_pixel_row_is_flipped():
    # +y in the world maps to a SMALLER row (image origin is top-left).
    _, row_low = g.world_to_pixel(0.0, 0.0, MAP)
    _, row_high = g.world_to_pixel(0.0, 1.0, MAP)
    assert row_high < row_low


def test_pixel_world_pixel_is_identity():
    # A cell CENTRE must round-trip back to the SAME cell: world_to_pixel(pixel_to_world(c,r))==(c,r).
    # This is the property goal-projection (lib.occupancy) and GUI click-to-place rely on; it exposes
    # the row off-by-one (int() applied to the whole flip expression instead of flooring the divide).
    for col, row in [(0, 0), (10, 20), (43, 55), (85, 109), (50, 0), (0, 109)]:
        x, y = g.pixel_to_world(col, row, MAP)
        assert g.world_to_pixel(x, y, MAP) == (col, row), f"round-trip lost ({col},{row})"


def test_world_to_pixel_negative_band_is_out_of_bounds():
    # The sub-cell strip just below/left of the origin must map OUT of bounds. int() truncated
    # -0.4 to 0 and landed an off-map point on a valid edge cell; floor must give -1 / height_px.
    mi = g.MapInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width_px=20, height_px=20)
    col, row = g.world_to_pixel(-0.02, -0.02, mi)
    assert col == -1
    assert row == 20            # below origin_y -> one past the bottom row


def test_euclidean():
    assert math.isclose(g.euclidean(0.0, 0.0, 3.0, 4.0), 5.0)


def test_pose_xyyaw_duck_typed():
    class _V:                       # geometry_msgs-shaped stand-ins (no ROS import in pure tests)
        def __init__(self, **kw): self.__dict__.update(kw)
    pose = _V(position=_V(x=1.0, y=-2.0),
              orientation=_V(z=math.sin(0.25), w=math.cos(0.25)))   # yaw 0.5
    x, y, yaw = g.pose_xyyaw(pose)
    assert (x, y) == (1.0, -2.0) and math.isclose(yaw, 0.5, abs_tol=1e-9)


def test_map_info_from_params_reads_the_standard_keys():
    vals = {'map_resolution': 0.1, 'map_origin_x': -1.0, 'map_origin_y': -2.0,
            'map_width_px': 50, 'map_height_px': 60}
    mi = g.map_info_from_params(lambda name, default: vals.get(name, default))
    assert mi == g.MapInfo(0.1, -1.0, -2.0, 50, 60)


def test_compose_pose_2d_identity():
    assert g.compose_pose_2d((0.0, 0.0, 0.0), (1.0, 2.0, 0.5)) == (1.0, 2.0, 0.5)


def test_compose_pose_2d_translation_only():
    assert g.compose_pose_2d((1.0, 1.0, 0.0), (2.0, 3.0, 0.0)) == (3.0, 4.0, 0.0)


def test_compose_pose_2d_rotation():
    # parent rotated +90deg: a child at (1,0) in the child frame lands at (0,1) in the parent.
    x, y, yaw = g.compose_pose_2d((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-9) and math.isclose(y, 1.0)
    assert math.isclose(yaw, math.pi / 2)


def test_compose_pose_2d_matches_amcl_case():
    # map->odom = (-1.69,-1.05, 64.1deg); odom pose (-0.26,-2.26, ?) -> map pose near amcl (0.15,-2.31)
    mx, my, _ = g.compose_pose_2d((-1.690, -1.051, 1.119), (-0.261, -2.260, 0.0))
    assert abs(mx - 0.15) < 0.2 and abs(my - (-2.31)) < 0.2


def test_compose_pose_2d_wraps_yaw():
    _, _, yaw = g.compose_pose_2d((0.0, 0.0, math.pi - 0.1), (0.0, 0.0, 0.2))
    assert -math.pi < yaw <= math.pi
    assert math.isclose(yaw, -(math.pi - 0.1), abs_tol=1e-9)
