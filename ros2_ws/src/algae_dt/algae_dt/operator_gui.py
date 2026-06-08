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

import functools
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from algae_dt.lib import geometry, hud
from algae_dt.lib.bloom_predictor import fetch_water_temperature, predict_severity
from algae_dt.lib.ros_utils import declare_get, load_package_map


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GuiBridge(Node):
    """ROS interface for the operator console. Subscribes /dt/* only; owns bloom placement."""

    def __init__(self, **kwargs) -> None:
        super().__init__('operator_gui', **kwargs)
        gp = functools.partial(declare_get, self)
        self.map_info = geometry.map_info_from_params(gp)
        self.battery_low_v = gp('battery_low_v', 11.0)
        self.battery_critical_v = gp('battery_critical_v', 10.5)
        self.bloom_radius = gp('bloom_radius_m', 0.15)
        self.range_min = gp('scan_range_min_m', 0.12)
        self.range_max = gp('scan_range_max_m', 3.5)
        self.latency_budget_ms = gp('latency_budget_ms', 250.0)
        self.stale_after_s = gp('gui_stale_after_s', 2.0)
        # sync_supervisor republishes a STILL-TRUE /dt/alerts at most every alert_repeat_s on ITS
        # clock — which is SIM time in sim_only. The GUI ages the banner on WALL time, so under a
        # throttled real-time-factor (~0.5) the wall gap between republishes (~2x alert_repeat_s)
        # exceeded a fixed 10 s window and the banner flipped to green while the condition still
        # held. Hold the banner at least 3x alert_repeat_s of wall time so a persistent alert stays
        # visible across the worst documented RTF. (A persistent issue staying RED is the safe side.)
        alert_repeat_s = gp('alert_repeat_s', 5.0)
        hold = gp('gui_alert_hold_s', 0.0)
        self.alert_hold_s = hold if hold > 0.0 else max(10.0, 3.0 * alert_repeat_s)

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
        self.last_alert = ''
        self.last_alert_t = float('-inf')   # time.monotonic() of the last /dt/alerts message
        # Arrival clock per CONTINUOUS stream (time.monotonic). /dt/safety doubles as the
        # mediator's heartbeat — it is republished every command tick — so its age is the DT-link
        # liveness. Latched change-driven topics (/dt/estop, /dt/mode, /dt/mission_state, ...) are
        # deliberately NOT aged: silence there is normal; their TRUST is gated on link_stale().
        # Without this, a dead mediator / dropped Wi-Fi froze every banner at its last value, all
        # green — the console looked healthy while the safety gate was no longer running.
        self.last_seen: dict[str, float] = {}

        self._next_id = 0
        self._blooms: list[tuple[int, float, float]] = []

        self.pub_blooms = self.create_publisher(MarkerArray, '/dt/blooms', _latched(10))
        self.pub_cmd = self.create_publisher(String, '/dt/mission_cmd', 10)
        self.pub_estop = self.create_publisher(Bool, '/dt/estop_cmd', _latched())
        self.pub_alerts = self.create_publisher(String, '/dt/alerts', 10)

        self.create_subscription(PoseStamped, '/dt/real_pose', self._on_real_pose, 10)
        self.create_subscription(PoseStamped, '/dt/sim_pose', self._on_sim_pose, 10)
        self.create_subscription(LaserScan, '/dt/scan_active', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/dt/markers', self._on_markers, _latched(10))
        self.create_subscription(String, '/dt/mode', self._on_mode, _latched())
        self.create_subscription(Bool, '/dt/sync_ok', self._on_sync_ok, _latched())
        self.create_subscription(Vector3, '/dt/sync_error', self._on_sync_err, 10)
        self.create_subscription(Float64, '/dt/latency_ms', self._on_latency, 10)
        self.create_subscription(BatteryState, '/dt/health', self._on_health, 10)
        self.create_subscription(Bool, '/dt/safety', self._on_safety, _latched())
        self.create_subscription(String, '/dt/mission_state', self._on_mission_state, _latched(10))
        self.create_subscription(Bool, '/dt/estop', self._on_estop, _latched())
        self.create_subscription(String, '/dt/alerts', self._on_alert, 10)

    # ---- inbound /dt/* state (named handlers so the mapping is unit-testable, F17) ----
    def _on_real_pose(self, m): self.real_pose = _xyyaw(m); self._seen('real_pose')
    def _on_sim_pose(self, m): self.sim_pose = _xyyaw(m); self._seen('sim_pose')
    def _on_scan(self, m): self.scan = m; self._seen('scan')
    def _on_markers(self, m): self.markers = m
    def _on_mode(self, m): self.mode = m.data
    def _on_sync_ok(self, m): self.sync_ok = m.data
    def _on_sync_err(self, m): self.sync_err = (m.x, m.y, m.z)
    def _on_latency(self, m): self.latency_ms = m.data; self._seen('latency')

    def _on_health(self, m):
        # The real OpenCR emits voltage=0.0 / present=False frames at bringup or on a serial
        # hiccup — without this guard one such frame painted a phantom "0.00 V" critical on a
        # healthy pack. Only finite positive voltages are accepted (and stamped as live data).
        if math.isfinite(m.voltage) and m.voltage > 0.0:
            self.battery_v = m.voltage
            self._seen('battery')

    def _on_safety(self, m): self.safety_blocked = m.data; self._seen('safety')
    def _on_mission_state(self, m): self.mission_state = m.data
    def _on_estop(self, m): self.estop = m.data

    # ---- stream liveness: a frozen console must visibly read as frozen ----
    def _seen(self, key: str) -> None:
        self.last_seen[key] = time.monotonic()

    def age_s(self, key: str) -> float:
        """Seconds since the last ACCEPTED message on `key` (inf when never seen)."""
        t = self.last_seen.get(key)
        return float('inf') if t is None else time.monotonic() - t

    def link_stale(self) -> bool:
        """True while the mediator's gate loop is silent (no /dt/safety for stale_after_s).
        The safety/E-STOP banners must then read UNKNOWN, not the last green."""
        return self.age_s('safety') > self.stale_after_s

    def _on_alert(self, m):
        self.last_alert = m.data
        self.last_alert_t = time.monotonic()

    # ---- operator actions ----
    def place_bloom(self, x: float, y: float) -> None:
        self._blooms.append((self._next_id, float(x), float(y)))
        self._next_id += 1
        self._publish_blooms()
        
        temperature = fetch_water_temperature()
        severity, t_harmful = predict_severity(self.bloom_radius, temperature)

        if severity == 'CRITICAL':
            self.pub_alerts.publish(String(data = "CRITICAL bloom detected! Dispatching robot immediately."))
            self.start()
        elif severity == 'HIGH':
            self.pub_alerts.publish(String(data = f"HIGH severity bloom detected! Dispatching robot in {t_harmful - 1:.2f} days."))
            delay_seconds = (t_harmful - 1) * 24 * 60 * 60
            def _dispatch():
                self.start()
                timer.cancel()
            
            timer = self.create_timer(delay_seconds, _dispatch)
            
        elif severity == 'MEDIUM':
            self.pub_alerts.publish(String(data = f"MODERATE severity bloom detected! Potentially, harmful in {t_harmful - 1:.2f} days."))
        elif severity == 'LOW':
            self.pub_alerts.publish(String(data = f"LOW severity bloom detected! Potentially, harmful in {t_harmful - 1:.2f} days."))
        else: 
            self.pub_alerts.publish(String(data = "Bloom detected. Severity: SAFE"))

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
    return geometry.pose_xyyaw(ps.pose)


def _load_map_qimage(map_info):
    """Parse maps/map.pgm with the in-tree parser into a Format_Grayscale8 QImage."""
    from PyQt5 import QtGui
    img = load_package_map()
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
            """Inverse of world_to_screen, or None for a click OUTSIDE the map image (the canvas
            keeps aspect ratio, so it has dark margins; a margin click used to truncate to a
            plausible-looking off-map world point and place a phantom bloom there)."""
            sc, ox, oy = self._xform()
            mi = self.bridge.map_info
            col = math.floor((sx - ox) / sc)
            row = math.floor((sy - oy) / sc)
            if not (0 <= col < mi.width_px and 0 <= row < mi.height_px):
                return None
            return geometry.pixel_to_world(col, row, mi)

        def mousePressEvent(self, ev):
            w = self.screen_to_world(ev.x(), ev.y())
            if w is None:
                return                       # click in the margin / outside the map: not a bloom
            self.bridge.place_bloom(*w)
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
            # a stale pose is the robot's PAST, not its position — don't draw it as live
            if self.bridge.age_s('real_pose') <= self.bridge.stale_after_s:
                self._draw_pose(qp, self.bridge.real_pose, QtGui.QColor(38, 204, 64))   # real = green
            if self.bridge.age_s('sim_pose') <= self.bridge.stale_after_s:
                self._draw_pose(qp, self.bridge.sim_pose, QtGui.QColor(51, 140, 255))   # sim = blue
            qp.end()

        def _active_pose(self):
            """Pose of the robot /dt/scan_active belongs to. The active (bare-topic) robot is the
            SIM in sim_only and the REAL Burger in real_only/both — anchoring the real robot's scan
            to the sim pose made the overlay visibly detach exactly when real and sim diverge (the
            very thing the twin is meant to show)."""
            if self.bridge.mode == 'sim_only':
                return self.bridge.sim_pose or self.bridge.real_pose
            return self.bridge.real_pose or self.bridge.sim_pose

        def _draw_scan(self, qp):
            scan, pose = self.bridge.scan, self._active_pose()
            if scan is None or pose is None or self.bridge.age_s('scan') > self.bridge.stale_after_s:
                return                       # a frozen scan overlay must vanish, not look live
            px, py, pyaw = pose
            qp.setPen(QtGui.QPen(QtGui.QColor(255, 80, 80, 200), 2))
            for rx, ry in hud.scan_points(list(scan.ranges), scan.angle_min, scan.angle_increment,
                                          scan.range_min or self.bridge.range_min,
                                          scan.range_max or self.bridge.range_max):
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
            x, y, yaw = pose
            c = self.world_to_screen(x, y)
            # Heading tip computed in WORLD metres then mapped through world_to_screen, so the
            # screen y-flip is handled consistently (raw screen-trig would mirror the arrow).
            tip = self.world_to_screen(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw))
            qp.setBrush(color)
            qp.setPen(QtGui.QPen(color, 2))
            qp.drawEllipse(c, 6, 6)
            qp.drawLine(c, tip)

    class OperatorWindow(QtWidgets.QWidget):
        def __init__(self, bridge):
            super().__init__()
            self.bridge = bridge
            self.setWindowTitle('algae-dt — Operator Console')
            self.canvas = MapCanvas(bridge)

            self.banners = {k: QtWidgets.QLabel(k) for k in
                            ('mode', 'mission', 'sync', 'latency', 'battery', 'safety', 'estop',
                             'alerts')}
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
            # Drain ALL ready work each tick (bounded): spin_once executes at most ONE callback,
            # and 12 subscriptions at pose/scan rates outrun one-callback-per-40ms — the overlay
            # then lags real time and never catches up.
            if not rclpy.ok():
                return
            for _ in range(32):
                rclpy.spin_once(self.bridge, timeout_sec=0.0)

        def _refresh(self):
            b = self.bridge
            # Honesty first: /dt/safety is the mediator heartbeat. While it is silent the console
            # must say UNKNOWN — the old behaviour kept painting the last (green) values, so a dead
            # mediator / dropped Wi-Fi looked exactly like a healthy session.
            link_lost = b.link_stale()
            shadow = "  (green = commanded shadow)" if b.mode == 'sim_only' else ""
            self._set('mode', f"MODE: {b.mode}{shadow}" + ("  — DT LINK LOST" if link_lost else ""),
                      'amber' if link_lost else 'green')
            self._set('mission', f"MISSION: {b.mission_state}", 'green')
            self._set('sync', f"SYNC: {hud.sync_text(b.sync_ok)}  "
                      f"dxy={b.sync_err[0]:.2f} dyaw={b.sync_err[1]:.2f}",
                      'green' if b.sync_ok else 'red')
            lat_age = b.age_s('latency')
            if math.isnan(b.latency_ms):
                self._set('latency', "LATENCY: —", 'green')
            elif lat_age > 10.0:
                # the sample is real but old (latency is only measured on a command->motion
                # onset) — show its age instead of an eternally-green frozen number
                self._set('latency', f"LATENCY: {b.latency_ms:.0f} ms ({lat_age:.0f}s ago)", 'amber')
            else:
                self._set('latency', f"LATENCY: {b.latency_ms:.0f} ms",
                          'green' if b.latency_ms <= b.latency_budget_ms else 'red')
            alert_age = time.monotonic() - b.last_alert_t
            if b.last_alert and alert_age < b.alert_hold_s:
                self._set('alerts', f"ALERT: {b.last_alert}", 'red')
            else:
                self._set('alerts', "ALERTS: none", 'green')
            if b.age_s('battery') > 3.0 * b.stale_after_s:
                # never seen a valid frame, or the feed died — '—' must not read as healthy green
                self._set('battery', "BATTERY: — (no data)", 'amber')
            else:
                self._set('battery', f"BATTERY: {b.battery_v:.2f} V", hud.battery_color(
                    b.battery_v, b.battery_low_v, b.battery_critical_v))
            if link_lost:
                self._set('safety', "SAFETY: UNKNOWN — DT LINK LOST", 'amber')
                self._set('estop', "E-STOP: UNKNOWN — DT LINK LOST", 'amber')
            else:
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
