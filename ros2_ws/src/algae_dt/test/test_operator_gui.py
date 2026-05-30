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
from std_msgs.msg import Bool, String                     # noqa: E402
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
        self.create_subscription(MarkerArray, '/dt/blooms', lambda m: setattr(self, 'last_blooms', m), _latched())
        self.create_subscription(String, '/dt/mission_cmd', lambda m: setattr(self, 'last_cmd', m.data), 10)
        self.create_subscription(Bool, '/dt/estop_cmd', lambda m: setattr(self, 'last_estop', m.data), _latched())


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
        x, y = win.canvas.screen_to_world(win.canvas.width() / 2, win.canvas.height() / 2)
        before = len(bridge._blooms)
        bridge.place_bloom(x, y)
        assert len(bridge._blooms) == before + 1
        win.canvas.repaint()           # paints map + bloom (no pose/scan yet) without error
        app.processEvents()
    finally:
        win._spin.stop()
        win._repaint.stop()
        bridge.destroy_node()
        rclpy.shutdown()
