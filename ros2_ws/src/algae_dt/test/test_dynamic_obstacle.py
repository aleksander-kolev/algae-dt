"""Construct test for dynamic_obstacle (PLAN T6.1). rclpy-only; the path maths are in test_trajectory.

Verifies the demo node constructs and reads params without a running sim (no gz calls fire until the
timer ticks, which a no-spin construction never does)."""
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter                    # noqa: E402

from algae_dt.dynamic_obstacle import DynamicObstacle    # noqa: E402


def test_constructs_without_sim():
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, False),
            Parameter('amplitude', Parameter.Type.DOUBLE, 0.6),
            Parameter('axis', Parameter.Type.STRING, 'y'),
        ])
        assert abs(node.amplitude - 0.6) < 1e-9 and node.axis == 'y'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_spawn_retries_until_confirmed_then_moves():
    """A slow/failed gz create must NOT make the node give up: it used to set _spawned unconditionally
    after one unchecked call, then teleport a model that never spawned. It must RETRY the create until
    the service confirms (data:true) and only THEN start moving the box."""
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, True),
            Parameter('obstacle_name', Parameter.Type.STRING, 'algae_obstacle'),
        ])
        calls = []
        seq = iter([False, False, True])          # create fails twice (slow sim), then succeeds
        def fake_gz(service, reqtype, req, timeout_ms=300):
            calls.append(service)
            return next(seq) if service.endswith('/create') else True
        node._gz = fake_gz
        created = lambda: sum(c.endswith('/create') for c in calls)
        moved = lambda: sum(c.endswith('/set_pose') for c in calls)

        node._tick(); assert not node._spawned and moved() == 0, "failed create -> not spawned, no move"
        node._tick(); assert not node._spawned and moved() == 0, "still retrying, never moves a ghost"
        node._tick(); assert node._spawned and created() == 3, "retries create until confirmed"
        assert moved() >= 1, "only moves once the obstacle actually exists"
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_axis_tolerates_yaml_bool_coercion():
    """`ros2 run … -p axis:=y` YAML-coerces bare `y` to bool True (the "Norway problem"); the node
    must accept it (dynamic typing) and normalize to 'y', not crash with InvalidParameterTypeException."""
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, False),
            Parameter('axis', Parameter.Type.BOOL, True),     # what an unquoted `axis:=y` becomes
        ])
        assert node.axis == 'y'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_rejects_unsafe_entity_name():
    """An entity name/world that would break the gz protobuf --req text is rejected at construction
    (validate at the boundary, fail fast) instead of producing a malformed request (F23)."""
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            DynamicObstacle(parameter_overrides=[
                Parameter('spawn', Parameter.Type.BOOL, False),
                Parameter('obstacle_name', Parameter.Type.STRING, 'evil" name'),
            ])
    finally:
        rclpy.shutdown()


def test_keepout_clamps_an_amplitude_that_would_sweep_the_robot_spawn():
    """The <static> box is TELEPORTED (no collision response): a sweep along x about (0.6,0)
    +/-0.6 m passes straight through the robot spawn (0,0). The amplitude must be clamped so the
    swept segment keeps >= keepout_radius_m clearance (0.6 - 0.3 = 0.3)."""
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, False),
            Parameter('axis', Parameter.Type.STRING, 'x'),
            Parameter('amplitude', Parameter.Type.DOUBLE, 0.6),
        ])
        assert abs(node.amplitude - 0.30) < 1e-6, \
            f"amplitude must be clamped to keep the 0.30 m keep-out (got {node.amplitude})"
        assert not node._refused
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_keepout_refuses_a_centre_on_top_of_the_robot_spawn():
    """A sweep CENTRE inside the keep-out cannot be saved by clamping: the node must refuse to
    spawn/teleport entirely (and say so), never shove the box into the robot."""
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('center_x', Parameter.Type.DOUBLE, 0.1),
            Parameter('amplitude', Parameter.Type.DOUBLE, 0.6),
        ])
        assert node._refused and node.do_spawn is False
        calls = []
        node._gz = lambda *a, **kw: calls.append(a) or True
        node._tick()
        assert calls == [], "a refused obstacle must never issue gz calls"
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_set_pose_failure_rearms_spawn():
    """A confirmed-spawned obstacle whose set_pose then keeps failing (e.g. a world reset removed the
    entity) must NOT freeze silently: after _SET_POSE_REFRESH_AFTER consecutive failures the node
    re-arms the spawn and re-creates the box rather than teleporting a model that no longer exists."""
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, True),
            Parameter('obstacle_name', Parameter.Type.STRING, 'algae_obstacle'),
        ])
        creates = []
        def fake_gz(service, reqtype, req, timeout_ms=300):
            if service.endswith('/create'):
                creates.append(service)
                return True            # create always confirms
            return False               # set_pose always fails (entity keeps disappearing)
        node._gz = fake_gz
        # A confirmed spawn, then a run of failing set_pose calls, must force a re-spawn (2nd create).
        for _ in range(node._SET_POSE_REFRESH_AFTER + 3):
            node._tick()
        assert len(creates) >= 2, \
            "repeated set_pose failures must re-arm the spawn and re-create the obstacle, not freeze"
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_negative_amplitude_cannot_bypass_the_keepout_clamp():
    """amplitude:=-0.6 used to invert the `clamped < amplitude` guard: the clamp was silently
    skipped and the static box teleported straight through the robot spawn (0,0). The node now
    takes the magnitude, so the sweep keeps the keep-out clearance regardless of the typed sign."""
    from algae_dt.lib import trajectory
    rclpy.init()
    try:
        node = DynamicObstacle(parameter_overrides=[
            Parameter('spawn', Parameter.Type.BOOL, False),
            Parameter('center_x', Parameter.Type.DOUBLE, 0.6),
            Parameter('center_y', Parameter.Type.DOUBLE, 0.0),
            Parameter('axis', Parameter.Type.STRING, 'x'),
            Parameter('amplitude', Parameter.Type.DOUBLE, -0.6),
        ])
        assert node.amplitude >= 0.0, "amplitude is a magnitude after the fix"
        assert trajectory.sweep_clearance(0.0, 0.0, node.cx, node.cy, node.amplitude,
                                          node.axis) >= node.keepout_radius_m - 1e-6, \
            "the swept segment must keep the keep-out clearance from the robot spawn"
        node.destroy_node()
    finally:
        rclpy.shutdown()
