"""Headless tests for operator_gui (PLAN T4.2). rclpy + PyQt5 (offscreen) only.

Verifies the Qt-free GuiBridge publishes the right operator commands, and that the PyQt5 window
builds + paints offscreen with the real course map loaded via the in-tree PGM parser.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import time  # noqa: E402

import pytest  # noqa: E402

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("PyQt5")

from rclpy.executors import SingleThreadedExecutor       # noqa: E402
from rclpy.node import Node                               # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,      # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import BatteryState                   # noqa: E402
from std_msgs.msg import Bool, Empty, Float64, String      # noqa: E402
from visualization_msgs.msg import MarkerArray            # noqa: E402

from algae_dt import operator_gui                          # noqa: E402
from algae_dt.operator_gui import GuiBridge                # noqa: E402


def _latched(depth=10):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Harness(Node):
    def __init__(self):
        super().__init__('gui_test_harness')
        self.last_blooms = None
        self.last_cmd = None
        self.last_estop = None
        self.resync_clicks = 0
        self.create_subscription(MarkerArray, '/dt/blooms', lambda m: setattr(self, 'last_blooms', m), _latched())
        self.create_subscription(String, '/dt/mission_cmd', lambda m: setattr(self, 'last_cmd', m.data), 10)
        self.create_subscription(Bool, '/dt/estop_cmd', lambda m: setattr(self, 'last_estop', m.data), _latched())
        self.create_subscription(Empty, '/dt/resync_cmd',
                                 lambda m: setattr(self, 'resync_clicks', self.resync_clicks + 1), 10)


def _spin_until(ex, pred, secs=6.0):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def test_bridge_publishes_operator_commands():
    rclpy.init()
    bridge = GuiBridge()
    har = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(har)
    try:
        bridge.place_bloom(0.5, -0.5)
        assert _spin_until(ex, lambda: har.last_blooms is not None and len(har.last_blooms.markers) == 1)
        m = har.last_blooms.markers[0]
        assert abs(m.pose.position.x - 0.5) < 1e-6 and abs(m.pose.position.y + 0.5) < 1e-6

        bridge.start()
        assert _spin_until(ex, lambda: har.last_cmd == 'start')

        bridge.set_estop(True)
        assert _spin_until(ex, lambda: har.last_estop is True)

        bridge.clear_blooms()
        assert _spin_until(ex, lambda: har.last_cmd == 'clear'
                           and har.last_blooms is not None and len(har.last_blooms.markers) == 0)
    finally:
        ex.shutdown()
        bridge.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def test_bridge_ingests_dt_state():
    """The named /dt/* handlers land each message on the matching bridge field (F17 — the inbound
    mapping was previously untested)."""
    rclpy.init()
    bridge = GuiBridge()
    pub = rclpy.create_node('gui_state_pub')
    p_mode = pub.create_publisher(String, '/dt/mode', _latched())
    p_sync = pub.create_publisher(Bool, '/dt/sync_ok', _latched())
    p_health = pub.create_publisher(BatteryState, '/dt/health', 10)
    p_lat = pub.create_publisher(Float64, '/dt/latency_ms', 10)
    p_safety = pub.create_publisher(Bool, '/dt/safety', _latched())
    p_mstate = pub.create_publisher(String, '/dt/mission_state', _latched())
    ex = SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(pub)
    try:
        p_mode.publish(String(data='both'))
        p_sync.publish(Bool(data=False))
        p_health.publish(BatteryState(voltage=11.5))
        p_lat.publish(Float64(data=42.0))
        p_safety.publish(Bool(data=True))
        p_mstate.publish(String(data='navigating:0'))
        ok = _spin_until(ex, lambda: bridge.mode == 'both' and bridge.sync_ok is False
                         and abs(bridge.battery_v - 11.5) < 1e-6
                         and abs(bridge.latency_ms - 42.0) < 1e-6
                         and bridge.safety_blocked is True
                         and bridge.mission_state == 'navigating:0')
        assert ok, "GuiBridge must ingest each /dt/* topic onto the matching field"
    finally:
        ex.shutdown()
        bridge.destroy_node()
        pub.destroy_node()
        rclpy.shutdown()


def test_bridge_ingests_alerts():
    """The operator console must surface /dt/alerts (latency/sync/stop-skew tolerances are a graded
    deliverable — they were previously invisible to the operator)."""
    rclpy.init()
    bridge = GuiBridge()
    pub = rclpy.create_node('gui_alert_pub')
    p_alert = pub.create_publisher(String, '/dt/alerts', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(pub)
    try:
        p_alert.publish(String(data='LATENCY 400 ms > budget 250 ms'))
        ok = _spin_until(ex, lambda: 'LATENCY' in bridge.last_alert)
        assert ok and bridge.last_alert_t > float('-inf')
    finally:
        ex.shutdown()
        bridge.destroy_node()
        pub.destroy_node()
        rclpy.shutdown()


def test_bridge_publishes_and_ingests_resync():
    """RESYNC is operator-facing both ways: request_resync() publishes /dt/resync_cmd, and an
    executed /dt/resync_event lands on the bridge for the SYNC banner note."""
    rclpy.init()
    bridge = GuiBridge()
    har = Harness()
    p_evt = har.create_publisher(String, '/dt/resync_event', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(har)
    try:
        bridge.request_resync()
        assert _spin_until(ex, lambda: har.resync_clicks == 1), \
            "request_resync() must publish one /dt/resync_cmd"
        p_evt.publish(String(data='manual dxy=0.21 dyaw=0.04 -> sim snapped to (1.00, 0.50, 0.00)'))
        assert _spin_until(ex, lambda: bridge.last_resync.startswith('manual'))
        assert bridge.last_resync_t > float('-inf')
    finally:
        ex.shutdown()
        bridge.destroy_node()
        har.destroy_node()
        rclpy.shutdown()


def test_resync_button_enabled_only_in_both_mode():
    """Only `both` has a mirror sim to snap: the RESYNC TWIN button must track /dt/mode (disabled
    in sim_only/real_only, enabled in both), and a recent resync event must surface on the SYNC
    banner."""
    from PyQt5 import QtWidgets
    rclpy.init()
    bridge = GuiBridge()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = operator_gui._make_window(bridge)
    try:
        win._spin.stop()
        win._repaint.stop()                               # drive _refresh by hand
        assert not win.buttons['RESYNC TWIN'].isEnabled(), "disabled until the mode is known"
        bridge.mode = 'sim_only'
        win._refresh()
        assert not win.buttons['RESYNC TWIN'].isEnabled()
        bridge.mode = 'both'
        win._refresh()
        assert win.buttons['RESYNC TWIN'].isEnabled()

        import time as _time
        bridge.last_resync = 'auto dxy=0.31 dyaw=0.05 -> sim snapped to (1.20, 0.45, 1.57)'
        bridge.last_resync_t = _time.monotonic()
        win._refresh()
        assert 'resynced (auto)' in win.banners['sync'].text(), \
            "a recent resync must surface on the SYNC banner"
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


def test_window_builds_and_paints_offscreen():
    from PyQt5 import QtWidgets
    rclpy.init()
    bridge = GuiBridge()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = operator_gui._make_window(bridge)
    try:
        win.resize(800, 600)
        win.show()
        app.processEvents()
        # the real 86x110 course map is loaded into the canvas
        assert win.canvas._img.width() == bridge.map_info.width_px == 86
        assert win.canvas._img.height() == bridge.map_info.height_px == 110
        # a click in the canvas centre places a bloom (screen->world->/dt/blooms list)
        w = win.canvas.screen_to_world(win.canvas.width() / 2, win.canvas.height() / 2)
        assert w is not None
        before = len(bridge._blooms)
        bridge.place_bloom(*w)
        assert len(bridge._blooms) == before + 1
        win.canvas.repaint()           # paints map + bloom (no pose/scan yet) without error
        app.processEvents()
    finally:
        win._spin.stop()
        win._repaint.stop()
        bridge.destroy_node()
        rclpy.shutdown()


def test_margin_click_is_rejected_not_a_phantom_bloom():
    """The canvas keeps aspect ratio, so it has dark margins around the map image. A click there
    used to truncate to a plausible off-map world point and publish a phantom bloom the mission
    would chase; screen_to_world must return None and place nothing."""
    from PyQt5 import QtWidgets
    rclpy.init()
    bridge = GuiBridge()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = operator_gui._make_window(bridge)
    try:
        win.resize(800, 600)        # 86x110 map scaled by ~5.45 -> wide dark margins left/right
        win.show()
        app.processEvents()
        assert win.canvas.screen_to_world(5.0, 5.0) is None, "margin click maps off the image"
        before = len(bridge._blooms)

        class _Ev:                  # minimal QMouseEvent stand-in
            def x(self): return 5
            def y(self): return 5
        win.canvas.mousePressEvent(_Ev())
        assert len(bridge._blooms) == before, "a margin click must not place a bloom"
    finally:
        win._spin.stop()
        win._repaint.stop()
        bridge.destroy_node()
        rclpy.shutdown()


def test_scan_overlay_anchors_to_the_active_robots_pose():
    """/dt/scan_active carries the ACTIVE robot's scan: the sim's in sim_only, the REAL Burger's in
    real_only/both. The overlay must anchor to the matching pose — anchoring the real scan to the
    sim pose detached the overlay exactly when real and sim diverge (the thing the twin shows)."""
    from PyQt5 import QtWidgets
    rclpy.init()
    bridge = GuiBridge()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = operator_gui._make_window(bridge)
    try:
        bridge.real_pose = (1.0, 1.0, 0.0)
        bridge.sim_pose = (2.0, 2.0, 0.0)
        bridge.mode = 'both'
        assert win.canvas._active_pose() == (1.0, 1.0, 0.0), "both: the active robot is the REAL one"
        bridge.mode = 'real_only'
        assert win.canvas._active_pose() == (1.0, 1.0, 0.0)
        bridge.mode = 'sim_only'
        assert win.canvas._active_pose() == (2.0, 2.0, 0.0), "sim_only: the active robot IS the sim"
    finally:
        win._spin.stop()
        win._repaint.stop()
        bridge.destroy_node()
        rclpy.shutdown()


def test_bridge_battery_guard_ignores_invalid_frames():
    """The real OpenCR emits voltage=0.0 / present=False frames at bringup or on a serial hiccup —
    one such frame used to paint a phantom '0.00 V' critical on a healthy pack. Invalid frames
    (non-finite or <= 0 V) must be ignored, and a later glitch must keep the last GOOD value."""
    import math
    rclpy.init()
    bridge = GuiBridge()
    pub = rclpy.create_node('gui_batt_pub')
    p = pub.create_publisher(BatteryState, '/dt/health', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(pub)
    try:
        p.publish(BatteryState(voltage=0.0))             # bringup glitch frame
        p.publish(BatteryState(voltage=float('nan')))    # serial garbage
        _spin_until(ex, lambda: False, secs=0.4)
        assert math.isnan(bridge.battery_v), "invalid frames must not land on the banner"
        assert bridge.age_s('battery') == float('inf'), "invalid frames must not count as live data"

        p.publish(BatteryState(voltage=11.5))
        assert _spin_until(ex, lambda: abs(bridge.battery_v - 11.5) < 1e-6)
        assert bridge.age_s('battery') < 5.0

        p.publish(BatteryState(voltage=0.0))             # later glitch: keep the last good value
        _spin_until(ex, lambda: False, secs=0.4)
        assert abs(bridge.battery_v - 11.5) < 1e-6
    finally:
        ex.shutdown()
        bridge.destroy_node()
        pub.destroy_node()
        rclpy.shutdown()


def test_bridge_link_liveness_tracks_dt_safety():
    """/dt/safety is the mediator heartbeat (republished every command tick): never seen -> the
    link reads STALE; a fresh message makes it live; silence flips it back. The old console kept
    every banner at its last (green) value when the mediator died — a frozen console must read
    as frozen."""
    from rclpy.parameter import Parameter
    rclpy.init()
    bridge = GuiBridge(parameter_overrides=[
        Parameter('gui_stale_after_s', Parameter.Type.DOUBLE, 0.3)])
    pub = rclpy.create_node('gui_live_pub')
    p_safety = pub.create_publisher(Bool, '/dt/safety', _latched())
    ex = SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(pub)
    try:
        assert bridge.link_stale(), "no /dt/safety yet -> the link must read STALE, not healthy"
        p_safety.publish(Bool(data=False))
        assert _spin_until(ex, lambda: not bridge.link_stale()), "heartbeat arrives -> link live"
        time.sleep(0.5)                                   # exceed the 0.3 s budget, no traffic
        assert bridge.link_stale(), "a silent mediator must flip the link back to STALE"
    finally:
        ex.shutdown()
        bridge.destroy_node()
        pub.destroy_node()
        rclpy.shutdown()


def test_refresh_paints_estop_engaged_banner():
    """The latched E-STOP state must surface on the console: with a live DT link, /dt/estop True
    paints 'E-STOP: ENGAGED' (red) and False paints 'clear'. The inbound estop -> banner path was
    never exercised."""
    from PyQt5 import QtWidgets
    from std_msgs.msg import Bool as _Bool
    rclpy.init()
    bridge = GuiBridge()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = operator_gui._make_window(bridge)
    try:
        win._spin.stop()
        win._repaint.stop()
        bridge._on_safety(_Bool(data=False))              # heartbeat -> link is live (not stale)
        bridge._on_estop(_Bool(data=True))
        win._refresh()
        assert win.banners['estop'].text() == 'E-STOP: ENGAGED', "latched E-STOP must read ENGAGED"
        bridge._on_safety(_Bool(data=False))
        bridge._on_estop(_Bool(data=False))
        win._refresh()
        assert win.banners['estop'].text() == 'E-STOP: clear'
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


def test_refresh_paints_unknown_banners_when_link_lost():
    """With the DT link stale, the SAFETY/E-STOP banners must read UNKNOWN — DT LINK LOST instead
    of the last (green) values; once the heartbeat arrives they return to the real state."""
    from PyQt5 import QtWidgets
    from rclpy.parameter import Parameter
    rclpy.init()
    bridge = GuiBridge(parameter_overrides=[
        Parameter('gui_stale_after_s', Parameter.Type.DOUBLE, 0.3)])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = operator_gui._make_window(bridge)
    try:
        win._spin.stop()
        win._repaint.stop()                               # drive _refresh by hand
        win._refresh()
        assert 'UNKNOWN' in win.banners['safety'].text(), "stale link -> SAFETY must read UNKNOWN"
        assert 'UNKNOWN' in win.banners['estop'].text(), "stale link -> E-STOP must read UNKNOWN"
        assert 'DT LINK LOST' in win.banners['mode'].text()

        bridge._on_safety(Bool(data=True))                # heartbeat arrives, gate blocked
        win._refresh()
        assert win.banners['safety'].text() == 'SAFETY: BLOCKED', "live link -> real gate state"
        assert win.banners['estop'].text() == 'E-STOP: clear'
        assert 'DT LINK LOST' not in win.banners['mode'].text()
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
