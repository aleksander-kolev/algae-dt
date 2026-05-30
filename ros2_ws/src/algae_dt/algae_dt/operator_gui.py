"""operator_gui — PyQt5 operator console. PLAN T4.2.

Two parts kept separate so the logic is testable without a display:
  * GuiBridge(Node) — the ROS side (subscribes /dt/* ONLY; publishes /dt/blooms, /dt/mission_cmd,
    /dt/estop_cmd). Qt-free and unit-testable.
  * _make_window(bridge) — builds the PyQt5 window (map canvas via the in-tree lib.pgm parser, real
    + sim pose overlay, live /dt/scan_active, bloom markers, click-to-place, Start/Stop/Clear/E-STOP,
    banners). PyQt5 is imported HERE, not at module import time, so colcon build / entry-point
    discovery never require PyQt5 (RULES/HANDOFF).

Headless: QT_QPA_PLATFORM=offscreen. The window drives rclpy via a QTimer spin_once.
"""
from __future__ import annotations

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry  # noqa: F401  (kept for parity / future odom overlay)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from algae_dt.lib import geometry, hud


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GuiBridge(Node):
    """ROS interface for the operator console. Subscribes /dt/* only; owns bloom placement."""

    def __init__(self, **kwargs) -> None:
        super().__init__('operator_gui', **kwargs)
        gp = self._declare
        self.map_info = geometry.MapInfo(
            resolution=gp('map_resolution', 0.05),
            origin_x=gp('map_origin_x', -2.051),
            origin_y=gp('map_origin_y', -4.194),
            width_px=gp('map_width_px', 86),
            height_px=gp('map_height_px', 110),
        )
        self.battery_low_v = gp('battery_low_v', 11.0)
        self.battery_critical_v = gp('battery_critical_v', 10.5)
        self.bloom_radius = gp('bloom_radius_m', 0.15)

        # latest state for the canvas/banners
        self.real_pose = None
        self.sim_pose = None
        self.scan: LaserScan | None = None
        self.markers: MarkerArray | None = None
        self.mode = '?'
        self.sync_ok = True
        self.sync_err = (0.0, 0.0, 0.0)
        self.latency_ms = float('nan')
        self.battery_v = float('nan')
        self.safety_blocked = False
        self.mission_state = 'idle'
        self.estop = False

        self._next_id = 0
        self._blooms: list[tuple[int, float, float]] = []

        self.pub_blooms = self.create_publisher(MarkerArray, '/dt/blooms', _latched(10))
        self.pub_cmd = self.create_publisher(String, '/dt/mission_cmd', 10)
        self.pub_estop = self.create_publisher(Bool, '/dt/estop_cmd', _latched())

        self.create_subscription(PoseStamped, '/dt/real_pose', lambda m: setattr(self, 'real_pose', _xyyaw(m)), 10)
        self.create_subscription(PoseStamped, '/dt/sim_pose', lambda m: setattr(self, 'sim_pose', _xyyaw(m)), 10)
        self.create_subscription(LaserScan, '/dt/scan_active', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/dt/markers', lambda m: setattr(self, 'markers', m), _latched(10))
        self.create_subscription(String, '/dt/mode', lambda m: setattr(self, 'mode', m.data), _latched())
        self.create_subscription(Bool, '/dt/sync_ok', lambda m: setattr(self, 'sync_ok', m.data), _latched())
        self.create_subscription(Vector3, '/dt/sync_error', lambda m: setattr(self, 'sync_err', (m.x, m.y, m.z)), 10)
        self.create_subscription(Float64, '/dt/latency_ms', lambda m: setattr(self, 'latency_ms', m.data), 10)
        self.create_subscription(BatteryState, '/dt/health', lambda m: setattr(self, 'battery_v', m.voltage), 10)
        self.create_subscription(Bool, '/dt/safety', lambda m: setattr(self, 'safety_blocked', m.data), _latched())
        self.create_subscription(String, '/dt/mission_state', lambda m: setattr(self, 'mission_state', m.data), _latched(10))
        self.create_subscription(Bool, '/dt/estop', lambda m: setattr(self, 'estop', m.data), _latched())

    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ---- operator actions ----
    def place_bloom(self, x: float, y: float) -> None:
        self._blooms.append((self._next_id, float(x), float(y)))
        self._next_id += 1
        self._publish_blooms()

    def clear_blooms(self) -> None:
        self._blooms = []
        self._publish_blooms()
        self.pub_cmd.publish(String(data='clear'))

    def _publish_blooms(self) -> None:
        arr = MarkerArray()
        for bid, x, y in self._blooms:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'blooms'
            m.id = bid
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.orientation.w = 1.0
            arr.markers.append(m)
        self.pub_blooms.publish(arr)

    def start(self) -> None:
        self.pub_cmd.publish(String(data='start'))

    def stop(self) -> None:
        self.pub_cmd.publish(String(data='stop'))

    def set_estop(self, on: bool) -> None:
        self.pub_estop.publish(Bool(data=bool(on)))


def _xyyaw(ps: PoseStamped):
    p, o = ps.pose.position, ps.pose.orientation
    return (p.x, p.y, geometry.yaw_from_quaternion(o.z, o.w))


def _load_map_qimage(map_info):
    """Parse maps/map.pgm with the in-tree parser into a Format_Grayscale8 QImage."""
    from PyQt5 import QtGui
    from algae_dt.lib import pgm
    path = os.path.join(get_package_share_directory('algae_dt'), 'maps', 'map.pgm')
    with open(path, 'rb') as f:
        img = pgm.parse(f.read())
    qimg = QtGui.QImage(bytes(img.pixels), img.width, img.height,
                        img.width, QtGui.QImage.Format_Grayscale8)
    return qimg.copy()   # detach from the temporary buffer


def _make_window(bridge: GuiBridge):
    """Build (do not exec) the operator window. PyQt5 imported here so import stays optional."""
    from PyQt5 import QtCore, QtGui, QtWidgets

    _STATE_QCOLOR = {
        'pending': QtGui.QColor(243, 217, 38),
        'active': QtGui.QColor(51, 140, 255),
        'treated': QtGui.QColor(38, 204, 64),
        'skipped': QtGui.QColor(140, 140, 140),
    }
    _BANNER = {'green': '#1f9d3a', 'amber': '#d98b00', 'red': '#c0392b'}

    class MapCanvas(QtWidgets.QWidget):
        def __init__(self, bridge):
            super().__init__()
            self.bridge = bridge
            self.setMinimumSize(430, 550)
            self._img = _load_map_qimage(bridge.map_info)

        # --- map<->screen transform (keep aspect, centred) ---
        def _xform(self):
            mi = self.bridge.map_info
            sc = min(self.width() / mi.width_px, self.height() / mi.height_px)
            ox = (self.width() - mi.width_px * sc) / 2.0
            oy = (self.height() - mi.height_px * sc) / 2.0
            return sc, ox, oy

        def world_to_screen(self, x, y):
            sc, ox, oy = self._xform()
            col, row = geometry.world_to_pixel(x, y, self.bridge.map_info)
            return QtCore.QPointF(ox + (col + 0.5) * sc, oy + (row + 0.5) * sc)

        def screen_to_world(self, sx, sy):
            sc, ox, oy = self._xform()
            col = (sx - ox) / sc
            row = (sy - oy) / sc
            return geometry.pixel_to_world(int(col), int(row), self.bridge.map_info)

        def mousePressEvent(self, ev):
            x, y = self.screen_to_world(ev.x(), ev.y())
            self.bridge.place_bloom(x, y)
            self.update()

        def paintEvent(self, _ev):
            qp = QtGui.QPainter(self)
            qp.fillRect(self.rect(), QtGui.QColor(25, 28, 33))
            sc, ox, oy = self._xform()
            mi = self.bridge.map_info
            target = QtCore.QRectF(ox, oy, mi.width_px * sc, mi.height_px * sc)
            qp.drawImage(target, self._img)
            self._draw_scan(qp)
            self._draw_blooms(qp, sc)
            self._draw_pose(qp, self.bridge.real_pose, QtGui.QColor(38, 204, 64))   # real = green
            self._draw_pose(qp, self.bridge.sim_pose, QtGui.QColor(51, 140, 255))   # sim = blue
            qp.end()

        def _active_pose(self):
            return self.bridge.sim_pose or self.bridge.real_pose

        def _draw_scan(self, qp):
            scan, pose = self.bridge.scan, self._active_pose()
            if scan is None or pose is None:
                return
            import math
            px, py, pyaw = pose
            qp.setPen(QtGui.QPen(QtGui.QColor(255, 80, 80, 200), 2))
            for rx, ry in hud.scan_points(list(scan.ranges), scan.angle_min, scan.angle_increment,
                                          scan.range_min or 0.12, scan.range_max or 3.5):
                wx = px + rx * math.cos(pyaw) - ry * math.sin(pyaw)
                wy = py + rx * math.sin(pyaw) + ry * math.cos(pyaw)
                qp.drawPoint(self.world_to_screen(wx, wy))

        def _draw_blooms(self, qp, sc):
            markers = self.bridge.markers
            if markers is not None and markers.markers:
                for m in markers.markers:
                    c = QtGui.QColor(int(m.color.r * 255), int(m.color.g * 255),
                                     int(m.color.b * 255)) if m.color.a > 0 else _STATE_QCOLOR['pending']
                    self._blob(qp, m.pose.position.x, m.pose.position.y, c, sc)
            else:  # not yet echoed by the mission -> show local placements as pending
                for _bid, x, y in self.bridge._blooms:
                    self._blob(qp, x, y, _STATE_QCOLOR['pending'], sc)

        def _blob(self, qp, x, y, color, sc):
            r = max(4.0, self.bridge.bloom_radius / self.bridge.map_info.resolution * sc)
            qp.setBrush(color)
            qp.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20), 1))
            qp.drawEllipse(self.world_to_screen(x, y), r, r)

        def _draw_pose(self, qp, pose, color):
            if pose is None:
                return
            import math
            x, y, yaw = pose
            c = self.world_to_screen(x, y)
            qp.setBrush(color)
            qp.setPen(QtGui.QPen(color, 2))
            qp.drawEllipse(c, 6, 6)
            qp.drawLine(c, QtCore.QPointF(c.x() + 14 * math.cos(yaw), c.y() + 14 * math.sin(yaw)))

    class OperatorWindow(QtWidgets.QWidget):
        def __init__(self, bridge):
            super().__init__()
            self.bridge = bridge
            self.setWindowTitle('algae-dt — Operator Console')
            self.canvas = MapCanvas(bridge)

            self.banners = {k: QtWidgets.QLabel(k) for k in
                            ('mode', 'mission', 'sync', 'latency', 'battery', 'safety', 'estop')}
            panel = QtWidgets.QVBoxLayout()
            for lab in self.banners.values():
                lab.setMargin(6)
                lab.setStyleSheet('color:white; background:#333; border-radius:4px;')
                panel.addWidget(lab)
            panel.addStretch(1)
            for name, slot in (('Start', bridge.start), ('Stop', bridge.stop),
                               ('Clear', bridge.clear_blooms),
                               ('E-STOP', lambda: bridge.set_estop(True)),
                               ('Resume', lambda: bridge.set_estop(False))):
                btn = QtWidgets.QPushButton(name)
                btn.clicked.connect(slot)
                panel.addWidget(btn)

            root = QtWidgets.QHBoxLayout(self)
            root.addWidget(self.canvas, 3)
            root.addLayout(panel, 1)

            self._spin = QtCore.QTimer(self)
            self._spin.timeout.connect(self._tick)
            self._spin.start(40)
            self._repaint = QtCore.QTimer(self)
            self._repaint.timeout.connect(self._refresh)
            self._repaint.start(100)

        def _tick(self):
            if rclpy.ok():
                rclpy.spin_once(self.bridge, timeout_sec=0.0)

        def _refresh(self):
            b = self.bridge
            self._set('mode', f"MODE: {b.mode}", 'green')
            self._set('mission', f"MISSION: {b.mission_state}", 'green')
            self._set('sync', f"SYNC: {hud.sync_text(b.sync_ok)}  "
                      f"dxy={b.sync_err[0]:.2f} dyaw={b.sync_err[1]:.2f}",
                      'green' if b.sync_ok else 'red')
            lat = '—' if b.latency_ms != b.latency_ms else f"{b.latency_ms:.0f} ms"
            self._set('latency', f"LATENCY: {lat}", 'green')
            bcol = 'green' if b.battery_v != b.battery_v else hud.battery_color(
                b.battery_v, b.battery_low_v, b.battery_critical_v)
            bv = '—' if b.battery_v != b.battery_v else f"{b.battery_v:.2f} V"
            self._set('battery', f"BATTERY: {bv}", bcol)
            self._set('safety', f"SAFETY: {hud.safety_text(b.safety_blocked)}",
                      'red' if b.safety_blocked else 'green')
            self._set('estop', "E-STOP: ENGAGED" if b.estop else "E-STOP: clear",
                      'red' if b.estop else 'green')
            self.canvas.update()

        def _set(self, key, text, color):
            lab = self.banners[key]
            lab.setText(text)
            lab.setStyleSheet(f'color:white; background:{_BANNER[color]}; border-radius:4px; padding:4px;')

        def keyPressEvent(self, ev):
            if ev.key() == QtCore.Qt.Key_Space:
                self.bridge.start()
            elif ev.key() == QtCore.Qt.Key_Escape:
                self.bridge.set_estop(True)

    return OperatorWindow(bridge)


def main(args=None) -> None:
    import sys
    from PyQt5 import QtWidgets
    rclpy.init(args=args)
    bridge = GuiBridge()
    app = QtWidgets.QApplication(sys.argv)
    win = _make_window(bridge)
    win.resize(720, 600)
    win.show()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
