"""In-process tests for mission_runner with an injected fake navigator (PLAN T2.2).

rclpy-only. Verifies honest task accounting (RULES §B-7/§B-8): SUCCEEDED -> spray + mark treated;
nav FAILED -> skipped (grey), NO spray; operator Stop/E-STOP mid-nav -> bloom stays PENDING.
The fake navigator stands in for Nav2 so the full mission loop runs without a live stack.
"""
import math
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from nav2_simple_commander.robot_navigator import TaskResult   # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from rclpy.executors import SingleThreadedExecutor       # noqa: E402
from rclpy.node import Node                               # noqa: E402
from rclpy.parameter import Parameter                     # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,      # noqa: E402
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String                     # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray    # noqa: E402

from algae_dt.lib import blooms as B                       # noqa: E402
from algae_dt.lib import geometry, occupancy                # noqa: E402
from algae_dt.mission_runner import MissionRunner, _COLORS  # noqa: E402


class FakeNavigator:
    """Stands in for nav2_simple_commander.BasicNavigator, matching its REAL contract:
    goToPose returns goal-acceptance (False = rejected, like a goal in collision), and
    waitUntilNav2Active may raise (the reused-navigator failure that breaks the 2nd mission)."""

    def __init__(self, outcomes, complete_after=1, accept=True, raise_on_active=False,
                 raise_on_go=False):
        self.outcomes = list(outcomes)
        self.complete_after = complete_after
        self.accept = accept                  # goToPose acceptance (real BasicNavigator returns bool)
        self.raise_on_active = raise_on_active  # simulate waitUntilNav2Active blowing up on reuse
        self.raise_on_go = raise_on_go          # simulate goToPose throwing (action-server fault)
        self._i = -1
        self._polls = 0
        self.goals = []
        self.cancelled = False

    def waitUntilNav2Active(self):
        if self.raise_on_active:
            raise RuntimeError("simulated Nav2 readiness failure on the reused navigator")

    def goToPose(self, pose):
        if self.raise_on_go:
            raise RuntimeError("simulated goToPose action-server fault")
        self.goals.append(pose)
        self._i += 1
        self._polls = 0
        return self.accept

    def isTaskComplete(self):
        self._polls += 1
        return self._polls >= self.complete_after

    def getResult(self):
        return self.outcomes[self._i] if 0 <= self._i < len(self.outcomes) else TaskResult.FAILED

    def cancelTask(self):
        self.cancelled = True


def _latched(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _state_of(marker: Marker) -> str:
    c = (round(marker.color.r, 2), round(marker.color.g, 2), round(marker.color.b, 2))
    for st, (r, g, b, _a) in _COLORS.items():
        if (round(r, 2), round(g, 2), round(b, 2)) == c:
            return st
    return '?'


class Harness(Node):
    def __init__(self, blooms_xy) -> None:
        super().__init__('mission_test_harness')
        self.blooms_xy = blooms_xy
        self.emit_blooms = True
        self.last_markers = None
        self.last_state = None
        self.max_omega = 0.0
        self.sim_yaw = 0.0              # simulated ACHIEVED robot yaw (published back on /dt/odom_active)
        self.rotation_gain = 1.0        # achieved/commanded rate; <1 models a robot that under-rotates
        self.robot_xy = (0.0, 0.0)      # MAP-frame robot position fed back on /dt/sim_pose
        self._cmd_omega = 0.0           # latest commanded angular rate seen on /dt/cmd_vel_raw
        self._last_tick_t = None
        self._tick_n = 0

        self.p_blooms = self.create_publisher(MarkerArray, '/dt/blooms', _latched())
        self.p_cmd = self.create_publisher(String, '/dt/mission_cmd', 10)
        self.p_estop = self.create_publisher(Bool, '/dt/estop', _latched())
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.p_pose = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)   # sim_only map pose
        self.create_subscription(MarkerArray, '/dt/markers', lambda m: setattr(self, 'last_markers', m), 10)
        self.create_subscription(String, '/dt/mission_state', lambda m: setattr(self, 'last_state', m.data), 10)
        self.create_subscription(TwistStamped, '/dt/cmd_vel_raw', self._on_spin, 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _on_spin(self, msg: TwistStamped) -> None:
        # Track the latest commanded angular rate; the achieved yaw is integrated against wall-clock in
        # _tick (scaled by rotation_gain) and fed back on /dt/odom_active, so the closed-loop spray
        # measures real turning and a slow robot (gain<1) forces it to keep spinning to N achieved revs.
        self._cmd_omega = msg.twist.angular.z
        self.max_omega = max(self.max_omega, abs(self._cmd_omega))

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9   # integrate achieved yaw vs wall-clock
        if self._last_tick_t is not None:
            self.sim_yaw += self.rotation_gain * self._cmd_omega * (now - self._last_tick_t)
        self._last_tick_t = now
        if self.emit_blooms:
            arr = MarkerArray()
            for i, (x, y) in enumerate(self.blooms_xy):
                m = Marker()
                m.id = i
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                arr.markers.append(m)
            self.p_blooms.publish(arr)
        od = Odometry()                      # simulated achieved yaw (spray closed loop)
        od.pose.pose.orientation.z = math.sin(self.sim_yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(self.sim_yaw / 2.0)
        self.p_odom.publish(od)
        # MAP-frame pose at ~10 Hz (every 3rd tick): plenty for nearest/arrival, and it keeps the
        # shared test executor from starving the 30 Hz odom feedback the spray closed loop needs
        # (the loop over-rotates when its odom view lags behind the harness integration).
        self._tick_n += 1
        if self._tick_n % 3 == 0:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = self.robot_xy
            ps.pose.orientation.w = 1.0
            self.p_pose.publish(ps)

    def send(self, cmd: str) -> None:
        self.p_cmd.publish(String(data=cmd))

    def estop(self, on: bool) -> None:
        self.p_estop.publish(Bool(data=on))


def _build(navigator, blooms_xy, spray_revolutions=0.05, spray_omega=1.0, grid=None):
    rclpy.init()
    runner = MissionRunner(navigator=navigator, parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'sim_only'),
        Parameter('spray_revolutions', Parameter.Type.DOUBLE, spray_revolutions),
        Parameter('spray_omega_radps', Parameter.Type.DOUBLE, spray_omega),
    ])
    # Pure state-machine tests run with goal projection OFF (grid=None) so outcomes are deterministic
    # regardless of whether the installed static map loads; projection tests inject a synthetic grid.
    runner._grid = grid
    har = Harness(blooms_xy)
    ex = SingleThreadedExecutor()
    ex.add_node(runner)
    ex.add_node(har)
    return runner, har, ex


def _spin_until(ex, pred, secs=8.0) -> bool:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def _teardown(runner, har, ex):
    runner._running = False
    ex.shutdown()
    runner.destroy_node()
    har.destroy_node()
    rclpy.shutdown()


def _markers_by_id(har):
    return {m.id: _state_of(m) for m in har.last_markers.markers} if har.last_markers else {}


def test_success_sprays_and_marks_treated():
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED, TaskResult.SUCCEEDED]),
                             [(0.1, 0.0), (0.2, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 2), "blooms registered"
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'complete'), "mission completes"
        states = _markers_by_id(har)
        assert states == {0: B.TREATED, 1: B.TREATED}
        assert har.max_omega > 0.0, "spray spin must be published to /dt/cmd_vel_raw"
    finally:
        _teardown(runner, har, ex)


def test_nav_failure_skips_loudly_with_partial_summary():
    """A failed nav must (1) skip without spraying, (2) raise an operator-visible /dt/alerts line
    naming the bloom, and (3) end in the PARTIAL summary state ('complete (0 treated, 1 skipped)')
    — never a bare 'complete'. An operator watched a nav-failed mission report 'complete' and
    reasonably concluded the twin faked a mission; partial outcomes must be loud and explicit."""
    from std_msgs.msg import String as _String
    runner, har, ex = _build(FakeNavigator([TaskResult.FAILED]), [(0.1, 0.0)])
    alerts = []
    har.create_subscription(_String, '/dt/alerts', lambda m: alerts.append(m.data), 10)
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'complete (0 treated, 1 skipped)'), \
            f"partial mission must publish the summary state, got {har.last_state!r}"
        assert _markers_by_id(har) == {0: B.SKIPPED}
        assert har.max_omega == 0.0, "a failed nav must NOT spray"
        assert any('BLOOM 0 SKIPPED' in a for a in alerts), \
            f"a skip must raise an operator-visible alert, got {alerts}"
    finally:
        _teardown(runner, har, ex)


def test_all_treated_mission_stays_plain_complete():
    """The clean path keeps the plain 'complete' state — the summary wording is reserved for
    partial outcomes so a fully successful mission never reads as qualified."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'complete')
        assert _markers_by_id(har) == {0: B.TREATED}
    finally:
        _teardown(runner, har, ex)


def test_clear_during_navigation_does_not_crash_worker():
    """A Clear arriving mid-navigation empties the field, so the worker's honest-accounting write
    targets a now-removed bloom id. The worker must NOT die with an unhandled KeyError (F1)."""
    errors = []
    prev_hook = threading.excepthook
    threading.excepthook = lambda args: errors.append(args)
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], complete_after=10_000),
                             [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'navigating:0'), "reaches navigating"
        har.emit_blooms = False          # stop the harness re-adding the bloom after Clear
        har.send('clear')                # empties the field while the worker is mid-navigation
        assert _spin_until(ex, lambda: not (runner._worker and runner._worker.is_alive())), \
            "worker thread exits after Clear"
        assert errors == [], \
            f"mission worker crashed mid-flight (F1): {[getattr(e, 'exc_value', e) for e in errors]}"
    finally:
        threading.excepthook = prev_hook
        _teardown(runner, har, ex)


def test_nav_timeout_skips_bloom():
    """Navigation never completes; the wall-clock watchdog (nav_goal_timeout_s) fires -> 'failed' ->
    bloom skipped (grey), no spray (F22 coverage of the timeout branch)."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], complete_after=10_000),
                             [(0.1, 0.0)])
    runner.nav_goal_timeout_s = 0.3      # force the deadline branch quickly
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.SKIPPED), \
            "nav timeout must skip the bloom (grey)"
        assert har.max_omega == 0.0, "a timed-out nav must NOT spray"
    finally:
        _teardown(runner, har, ex)


def test_gross_arrival_far_pose_skips_in_sim_only():
    """Nav2 reports SUCCEEDED but the sim_only gross-arrival check sees the robot far from the bloom
    -> skipped, NOT treated (F22 coverage of the _gross_arrived False branch)."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(5.0, 0.0)])  # bloom 5 m away
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: (har.last_state or '').startswith('complete'))
        assert _markers_by_id(har) == {0: B.SKIPPED}, "SUCCEEDED + far gross-pose -> skipped, not treated"
        assert har.max_omega == 0.0
    finally:
        _teardown(runner, har, ex)


def test_start_after_completion_retries_skipped_bloom():
    """A bloom that fails (skipped) on the first mission must be retried by a fresh Start after the
    mission completes — so the operator can re-run instead of being stuck (then it succeeds)."""
    runner, har, ex = _build(FakeNavigator([TaskResult.FAILED, TaskResult.SUCCEEDED]), [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.SKIPPED), "first attempt -> skipped"
        assert _spin_until(ex, lambda: (har.last_state or '').startswith('complete'))
        har.send('start')                                    # retry
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.TREATED), \
            "a fresh Start after completion must retry the skipped bloom (which now succeeds)"
    finally:
        _teardown(runner, har, ex)


def test_estop_midnav_leaves_bloom_pending():
    # Navigation never completes on its own (complete_after huge); E-STOP interrupts it.
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], complete_after=10_000),
                             [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'navigating:0'), "reaches navigating"
        har.estop(True)
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.PENDING), \
            "operator E-STOP mid-nav must leave the bloom PENDING (honest accounting)"
        assert har.max_omega == 0.0
    finally:
        _teardown(runner, har, ex)


# ----------------------------------------------------------- navigator-contract robustness
def test_rejected_goal_skips_without_stale_success():
    """BasicNavigator.goToPose returns False when a goal is rejected (e.g. in collision) and leaves
    its last result untouched. The runner MUST treat that as a failure, NOT poll and reuse the prior
    goal's stale SUCCEEDED status -> spray a bloom it never reached (the near-wall silent mis-treat)."""
    # outcome would be SUCCEEDED if (wrongly) polled; accept=False makes goToPose reject the goal.
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], accept=False), [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: (har.last_state or '').startswith('complete'))
        assert _markers_by_id(har) == {0: B.SKIPPED}, "rejected goal -> skipped, never treated"
        assert har.max_omega == 0.0, "a rejected goal must NOT spray"
    finally:
        _teardown(runner, har, ex)


def test_mission_proceeds_despite_broken_wait_until_active():
    """Bug 1 root cause: on the 2nd+ mission the reused BasicNavigator's waitUntilNav2Active() runs
    (it is skipped on the lazily-created 1st run), and there it can block / republish a bad initial
    pose / throw under executor contention. The mission must NOT depend on it: a raising readiness
    check must not stop the robot from navigating."""
    nav = FakeNavigator([TaskResult.SUCCEEDED], raise_on_active=True)
    runner, har, ex = _build(nav, [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.TREATED), \
            "mission must navigate+treat even when waitUntilNav2Active() raises"
        assert nav.goals, "a goal must actually be sent to the navigator"
    finally:
        _teardown(runner, har, ex)


def test_restart_after_completion_treats_a_newly_placed_bloom():
    """Bug 1 scenario: mission completes, operator drops a NEW bloom and clicks Start again -> the
    new bloom must be navigated+treated. Uses a navigator whose readiness check raises (the real
    reused-navigator failure mode) so this also guards the fix, not just the state machine."""
    nav = FakeNavigator([TaskResult.SUCCEEDED, TaskResult.SUCCEEDED], raise_on_active=True)
    runner, har, ex = _build(nav, [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'complete' and
                           _markers_by_id(har).get(0) == B.TREATED), "first bloom treated"
        har.blooms_xy = [(0.1, 0.0), (0.4, 0.0)]          # operator drops a second bloom
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 2), "second bloom registered"
        har.send('start')                                  # restart
        assert _spin_until(ex, lambda: _markers_by_id(har).get(1) == B.TREATED), \
            "a fresh Start after completion must navigate+treat the newly placed bloom"
    finally:
        _teardown(runner, har, ex)


# ----------------------------------------------------------- goal projection (near-wall fix)
def _mi():
    return geometry.MapInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width_px=20, height_px=20)


def _wall_left_grid(wall_cols=3, w=20, h=20):
    return occupancy.Grid(w, h, tuple(c >= wall_cols for r in range(h) for c in range(w)))


def _only_one_free_grid(cell=(10, 10), w=20, h=20):
    return occupancy.Grid(w, h, tuple((c, r) == cell for r in range(h) for c in range(w)))


def test_near_wall_goal_is_projected_off_the_wall():
    """A bloom hugging a wall must be sent to a projected, reachable goal (off the wall), not its raw
    centre that sits in the costmap inflation/lethal zone."""
    grid = _wall_left_grid(wall_cols=3)
    mi = _mi()
    bx, by = geometry.pixel_to_world(3, 10, mi)            # the free cell right against the wall
    nav = FakeNavigator([TaskResult.SUCCEEDED])
    runner, har, ex = _build(nav, [(bx, by)], grid=grid)
    runner._map_info = mi
    runner.goal_clearance_m = 0.10
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: bool(nav.goals)), "a goal is sent"
        gx = nav.goals[0].pose.position.x
        assert gx > bx + 1e-6, "goal must be pushed away from the wall (greater x)"
        col, row = geometry.world_to_pixel(gx, nav.goals[0].pose.position.y, mi)
        assert occupancy.clear_radius_ok(grid, col, row, 0.10 / mi.resolution), \
            "projected goal must have the required clearance"
    finally:
        _teardown(runner, har, ex)


def test_unreachable_bloom_skips_fast_without_calling_nav():
    """A bloom with no clear cell within reach (boxed in) must be skipped IMMEDIATELY — no goal sent
    to Nav2 (so no recovery-behaviour churn / 60 s timeout that looks like 'does nothing')."""
    grid = _only_one_free_grid(cell=(10, 10))
    mi = _mi()
    bx, by = geometry.pixel_to_world(10, 10, mi)
    nav = FakeNavigator([TaskResult.SUCCEEDED])
    runner, har, ex = _build(nav, [(bx, by)], grid=grid)
    runner._map_info = mi
    runner.goal_clearance_m = 0.10
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.SKIPPED), \
            "an unreachable bloom is skipped"
        assert nav.goals == [], "no goal must be sent to Nav2 for an unreachable bloom"
        assert har.max_omega == 0.0
    finally:
        _teardown(runner, har, ex)


# ----------------------------------------------------------- spray = N full revolutions (closed-loop)
def test_spray_turns_full_revolutions_measured_by_odom():
    """Chemical spraying is a fixed number of FULL in-place spins, CLOSED-LOOP on odometry: the robot
    must actually turn ~spray_revolutions * 2*pi (the harness feeds back achieved yaw on
    /dt/odom_active). omega raised so the test is quick."""
    revs, omega = 2.0, 12.0
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)],
                             spray_revolutions=revs, spray_omega=omega)
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.TREATED, secs=12), \
            "bloom treated after a full spray"
        assert har.sim_yaw == pytest.approx(revs * 2.0 * math.pi, rel=0.15), \
            f"robot must TURN ~{revs} full revolutions, turned {har.sim_yaw / (2*math.pi):.2f}"
    finally:
        _teardown(runner, har, ex)


def test_spray_keeps_turning_until_revolutions_achieved_when_robot_is_slow():
    """The spray is closed-loop: a robot that under-rotates (achieves only a fraction of the commanded
    rate) must KEEP spinning until the ACHIEVED rotation reaches N revs — not stop after a fixed time.
    This is the actual fix for 'it isn't doing 3 full 360s'. (Open-loop timing fails this.)"""
    revs, omega = 2.0, 12.0
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)],
                             spray_revolutions=revs, spray_omega=omega)
    har.rotation_gain = 0.6        # robot only achieves 60% of the commanded spin rate
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.TREATED, secs=15), \
            "bloom treated only after the achieved rotation reaches N revs"
        assert har.sim_yaw == pytest.approx(revs * 2.0 * math.pi, rel=0.15), \
            f"closed-loop must spin to ~{revs} ACHIEVED revs despite a slow robot, " \
            f"turned {har.sim_yaw / (2*math.pi):.2f}"
    finally:
        _teardown(runner, har, ex)


def test_spray_backstop_stall_skips_without_marking_treated():
    """If odom never advances during the spray (robot wedged / odom stream stalled), the closed-loop
    spin can't reach N revs. The backstop must end it, the bloom must NOT be marked treated, and the
    mission must NOT re-spray it forever — it is SKIPPED (honest accounting, RULES §B-8). This guards
    the fix for the backstop returning success on an incomplete spin."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)],
                             spray_revolutions=1.0, spray_omega=4.0)
    har.rotation_gain = 0.0          # the robot is commanded to spin but never actually turns
    runner.spray_time_margin = 0.4   # short backstop so the stall is detected quickly
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: (har.last_state or '').startswith('complete'), secs=12), \
            "mission must end (no infinite re-spray of a stalled bloom)"
        assert _markers_by_id(har) == {0: B.SKIPPED}, "a stalled spray must skip, never mark treated"
        assert har.max_omega > 0.0, "the spin was actually attempted before the backstop fired"
    finally:
        _teardown(runner, har, ex)


def test_operator_stop_midnav_leaves_bloom_pending():
    """Operator Stop (distinct from E-STOP) mid-navigation must leave the bloom PENDING (resumable on
    the next Start), not skipped or treated — the 'stopped' accounting branch."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], complete_after=10_000),
                             [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'navigating:0'), "reaches navigating"
        har.send('stop')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.PENDING), \
            "operator Stop mid-nav must leave the bloom PENDING (resumable, honest accounting)"
        assert har.max_omega == 0.0, "a stopped nav must NOT spray"
    finally:
        _teardown(runner, har, ex)


def test_arrival_is_checked_against_the_projected_goal_not_the_raw_centre():
    """REGRESSION (near-wall false-skip): the goal of a wall-hugging bloom is projected up to
    center_tol_m away from the raw centre, and Nav2 stops within ITS tolerance of that projected
    goal. The gross-arrival check must therefore measure against the PROJECTED goal: measured
    against the raw centre, the worst case (projection + Nav2 tolerance > center_tol_m) skipped a
    bloom that navigation genuinely reached."""
    w, h = 40, 20
    grid = occupancy.Grid(w, h, tuple(c >= 3 for r in range(h) for c in range(w)))   # wall cols 0-2
    mi = geometry.MapInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width_px=w, height_px=h)
    bx, by = geometry.pixel_to_world(3, 10, mi)        # bloom in the free cell hugging the wall
    # complete_after=15 keeps Nav2 'driving' (~1.5 s of polls) so the harness has time to publish
    # the robot's parked position before the arrival check runs.
    nav = FakeNavigator([TaskResult.SUCCEEDED], complete_after=15)
    runner, har, ex = _build(nav, [(bx, by)], grid=grid)
    runner._map_info = mi
    runner.goal_clearance_m = 0.40                     # forces a ~0.40 m projection off the wall
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: bool(nav.goals)), "a goal is sent"
        gx, gy = nav.goals[0].pose.position.x, nav.goals[0].pose.position.y
        assert geometry.euclidean(gx, gy, bx, by) > 0.30, "the goal really was projected far"
        # The robot parks 0.14 m PAST the projected goal (within Nav2's own tolerance of it) —
        # which puts it > center_tol_m from the RAW centre. Must still count as arrived.
        har.robot_xy = (gx + 0.14, gy)
        assert geometry.euclidean(gx + 0.14, gy, bx, by) > runner.center_tol_m
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.TREATED, secs=12), \
            "arrival at the PROJECTED goal must treat the bloom (raw-centre check falsely skipped it)"
    finally:
        _teardown(runner, har, ex)


def test_spray_stall_watchdog_fires_independent_of_the_time_cap():
    """The PRIMARY spray guard is the odom-stall watchdog (no yaw progress for
    spray_stall_timeout_s), not the total-time cap: a frozen odom must end the spray quickly even
    when the absolute cap is still far away — and the bloom stays honestly un-treated."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)],
                             spray_revolutions=1.0, spray_omega=4.0)
    har.rotation_gain = 0.0              # commanded to spin, never actually turns
    runner.spray_stall_timeout_s = 0.4   # quick watchdog...
    runner.spray_time_margin = 60.0      # ...while the absolute cap is ~94 s away
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        t0 = time.monotonic()
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.SKIPPED, secs=10), \
            "a frozen odom must skip via the stall watchdog"
        assert time.monotonic() - t0 < 8.0, "the watchdog, not the 94 s cap, must end the spray"
    finally:
        _teardown(runner, har, ex)


def test_spray_completes_on_a_slow_sim_that_keeps_progressing():
    """REGRESSION (RTF false-skip): a robot achieving only ~25% of the commanded rate — worse than
    the documented worst-case RTF 0.5 that the old margin=2.0 backstop could not survive — must
    still complete the full count and mark the bloom TREATED, because progress never stalls."""
    revs, omega = 1.0, 12.0
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)],
                             spray_revolutions=revs, spray_omega=omega)
    har.rotation_gain = 0.25
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.TREATED, secs=15), \
            "slow-but-progressing rotation must finish the count, never be skipped by a time cap"
        assert har.sim_yaw == pytest.approx(revs * 2.0 * math.pi, rel=0.2)
    finally:
        _teardown(runner, har, ex)


def test_estop_midnav_publishes_terminal_mission_state():
    """REGRESSION: E-STOP set _running=False but published NO mission_state, so the worker broke out
    leaving /dt/mission_state stuck at 'navigating:0' forever — the GUI banner then lied that the
    mission was still running. The rising E-STOP edge must publish a terminal state ('estopped')."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], complete_after=10_000),
                             [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'navigating:0'), "reaches navigating"
        har.estop(True)
        assert _spin_until(ex, lambda: har.last_state == 'estopped'), \
            "E-STOP must publish a terminal mission_state, not leave it stuck at 'navigating:0'"
    finally:
        _teardown(runner, har, ex)


def test_nan_odom_does_not_falsely_complete_spray():
    """REGRESSION (geometry NaN): a malformed odom quaternion made yaw NaN; the worker-loop spray
    then did turned += NaN, so `turned < target` became False and the spin EXITED immediately,
    marking an un-sprayed bloom TREATED. A non-finite odom sample must be ignored: _spray_turned
    stays finite and a NaN burst can never satisfy the completion criterion on its own."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED]), [(0.1, 0.0)],
                             spray_revolutions=1.0, spray_omega=8.0)
    try:
        # drive the accumulator directly with a NaN-quaternion odom while a spray is armed
        runner._spray_active = True
        runner._spray_turned = 0.0
        runner._spray_last_yaw = None
        bad = Odometry()
        bad.pose.pose.orientation.z = float('nan')
        bad.pose.pose.orientation.w = float('nan')
        runner._on_odom(bad)
        runner._on_odom(bad)
        assert math.isfinite(runner._spray_turned) and runner._spray_turned == 0.0, \
            "a NaN odom sample must never be accumulated into the spray count"
        runner._spray_active = False
    finally:
        _teardown(runner, har, ex)


def test_worker_crash_reverts_inflight_bloom_to_pending():
    """A worker crash (e.g. the navigator throwing) must not leave the in-flight bloom stuck ACTIVE
    (blue, looks in-progress forever): it reverts to PENDING so a re-Start resumes it honestly."""
    runner, har, ex = _build(FakeNavigator([TaskResult.SUCCEEDED], raise_on_go=True), [(0.1, 0.0)])
    try:
        assert _spin_until(ex, lambda: len(_markers_by_id(har)) == 1)
        har.send('start')
        assert _spin_until(ex, lambda: har.last_state == 'idle'
                           and not (runner._worker and runner._worker.is_alive()), secs=10), \
            "the worker exits via the crash path and reports idle"
        assert _spin_until(ex, lambda: _markers_by_id(har).get(0) == B.PENDING), \
            "the in-flight bloom must be reverted to PENDING after a worker crash"
    finally:
        _teardown(runner, har, ex)
