"""In-process integration tests for sync_supervisor (PLAN T3.2, Rubric ②).

rclpy-only (skipped on a bare host). Feeds /dt/real_pose + /dt/sim_pose (+ /cmd_vel, /dt/odom_active)
and asserts the measured sync error, the /dt/sync_ok flip + /dt/alerts on out-of-tolerance, the
command->motion latency, and that the CSV evidence file is written with the documented columns.
"""
import glob
import math
import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3   # noqa: E402
from nav_msgs.msg import Odometry                                          # noqa: E402
from rclpy.executors import SingleThreadedExecutor                         # noqa: E402
from rclpy.node import Node                                                 # noqa: E402
from rclpy.parameter import Parameter                                       # noqa: E402
from std_msgs.msg import Bool, Float64, String                              # noqa: E402

from algae_dt.lib import geometry, metrics                                  # noqa: E402
from algae_dt.sync_supervisor import SyncSupervisor                         # noqa: E402


def _pose(x, y, yaw=0.0) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'odom'
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z, ps.pose.orientation.w = geometry.quaternion_from_yaw(yaw)
    return ps


class Harness(Node):
    def __init__(self) -> None:
        super().__init__('sync_test_harness')
        self.real = (0.0, 0.0, 0.0)
        self.sim = (0.0, 0.0, 0.0)
        self.cmd_vx = 0.0
        self.odom_vx = 0.0
        self.last_err: Vector3 | None = None
        self.last_ok: bool | None = None
        self.last_alert: str | None = None
        self.last_latency: float | None = None

        self.p_real = self.create_publisher(PoseStamped, '/dt/real_pose', 10)
        self.p_sim = self.create_publisher(PoseStamped, '/dt/sim_pose', 10)
        self.p_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.p_odom = self.create_publisher(Odometry, '/dt/odom_active', 10)
        self.create_subscription(Vector3, '/dt/sync_error', lambda m: setattr(self, 'last_err', m), 10)
        self.create_subscription(Bool, '/dt/sync_ok', lambda m: setattr(self, 'last_ok', m.data), 10)
        self.create_subscription(String, '/dt/alerts', lambda m: setattr(self, 'last_alert', m.data), 10)
        self.create_subscription(Float64, '/dt/latency_ms', lambda m: setattr(self, 'last_latency', m.data), 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _tick(self) -> None:
        self.p_real.publish(_pose(*self.real))
        self.p_sim.publish(_pose(*self.sim))
        now = self.get_clock().now().to_msg()
        c = TwistStamped()
        c.header.stamp = now
        c.twist = Twist()
        c.twist.linear.x = self.cmd_vx
        self.p_cmd.publish(c)
        od = Odometry()
        od.header.stamp = now
        od.twist.twist.linear.x = self.odom_vx
        self.p_odom.publish(od)


@pytest.fixture()
def world(tmp_path):
    rclpy.init()
    sup = SyncSupervisor(parameter_overrides=[
        Parameter('mode', Parameter.Type.STRING, 'sim_only'),
        Parameter('sync_log_dir', Parameter.Type.STRING, str(tmp_path)),
    ])
    har = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(sup)
    ex.add_node(har)
    yield har, ex, tmp_path
    ex.shutdown()
    sup.destroy_node()
    har.destroy_node()
    rclpy.shutdown()


def _spin_until(ex, pred, secs=6.0) -> bool:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if pred():
            return True
    return False


def test_in_tolerance_sets_sync_ok_true(world):
    har, ex, _ = world
    har.real = (0.0, 0.0, 0.0)
    har.sim = (0.02, 0.0, 0.0)        # 2 cm < 15 cm tolerance
    ok = _spin_until(ex, lambda: har.last_ok is True and har.last_err is not None)
    assert ok and abs(har.last_err.x - 0.02) < 1e-6


def test_out_of_tolerance_flips_and_alerts(world):
    har, ex, _ = world
    har.sim = (0.5, 0.0, 0.0)          # 50 cm >> 15 cm tolerance
    ok = _spin_until(ex, lambda: har.last_ok is False and har.last_alert is not None)
    assert ok and 'OUT-OF-TOLERANCE' in har.last_alert


def test_command_to_motion_latency_measured(world):
    har, ex, _ = world
    har.cmd_vx = 0.2
    har.odom_vx = 0.2                  # robot moves shortly after the command
    ok = _spin_until(ex, lambda: har.last_latency is not None)
    assert ok and har.last_latency >= 0.0


def test_csv_evidence_written_with_columns(world):
    har, ex, tmp_path = world
    har.sim = (0.03, 0.0, 0.0)
    _spin_until(ex, lambda: har.last_err is not None, secs=3.0)
    # let a couple of 5 Hz ticks append rows
    _spin_until(ex, lambda: False, secs=0.6)
    files = glob.glob(os.path.join(str(tmp_path), 'sync_metrics_*.csv'))
    assert files, "a sync_metrics CSV should be created"
    with open(files[0], encoding='utf-8') as f:
        lines = f.read().strip().splitlines()
    assert lines[0] == metrics.csv_header()
    assert len(lines) >= 2          # header + at least one data row
