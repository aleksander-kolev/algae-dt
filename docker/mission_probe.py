"""Live mission probe (run against a launched sim_only stack). Places one bloom, starts the
mission, and watches /odom, /dt/markers and /dt/cmd_vel_raw to confirm the robot actually navigates
and sprays and the bloom is marked treated. Prints a RESULT line. Usage: mission_probe.py [timeout]."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

TREATED_RGB = (0.15, 0.80, 0.25)


def _latched(d=10):
    return QoSProfile(depth=d, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Probe(Node):
    def __init__(self):
        super().__init__('mission_probe')
        self.states = []
        self.markers = None
        self.max_omega = 0.0
        self.odom0 = None
        self.odom_moved = 0.0
        self.pub_blooms = self.create_publisher(MarkerArray, '/dt/blooms', _latched())
        self.pub_cmd = self.create_publisher(String, '/dt/mission_cmd', 10)
        self.create_subscription(String, '/dt/mission_state', self._st, _latched())
        self.create_subscription(MarkerArray, '/dt/markers', lambda m: setattr(self, 'markers', m), _latched())
        self.create_subscription(TwistStamped, '/dt/cmd_vel_raw',
                                 lambda m: setattr(self, 'max_omega', max(self.max_omega, abs(m.twist.angular.z))), 10)
        self.create_subscription(Odometry, '/odom', self._od, 10)

    def _st(self, m):
        if not self.states or self.states[-1] != m.data:
            self.states.append(m.data)

    def _od(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.odom0 is None:
            self.odom0 = p
        else:
            self.odom_moved = max(self.odom_moved, math.hypot(p[0] - self.odom0[0], p[1] - self.odom0[1]))

    def place(self, x, y):
        a = MarkerArray()
        mk = Marker()
        mk.id = 0
        mk.pose.position.x = x
        mk.pose.position.y = y
        a.markers = [mk]
        self.pub_blooms.publish(a)

    def start(self):
        self.pub_cmd.publish(String(data='start'))


def _treated(markers):
    if not markers:
        return False
    return any(abs(m.color.r - TREATED_RGB[0]) < 0.05 and abs(m.color.g - TREATED_RGB[1]) < 0.05
               for m in markers.markers)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    p = Probe()
    spin_to = lambda secs: [rclpy.spin_once(p, timeout_sec=0.1) for _ in range(int(secs / 0.1))]

    spin_to(8)                       # let discovery settle
    bx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    p.place(bx, 0.0)                 # one bloom bx m ahead of the spawn (reachable in slow headless sim)
    spin_to(4)
    for _ in range(8):               # nudge start a few times until the mission picks it up
        p.start()
        spin_to(0.5)

    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(p, timeout_sec=0.2)
        if _treated(p.markers) or (p.states and p.states[-1] == 'complete'):
            break

    treated = _treated(p.markers)
    moved = p.odom_moved > 0.05
    sprayed = p.max_omega > 0.0
    print('STATES:', ' -> '.join(p.states))
    print('ODOM_MOVED_M: %.3f' % p.odom_moved)
    print('MAX_SPRAY_OMEGA: %.3f' % p.max_omega)
    print('TREATED:', treated)
    print('RESULT:', 'PASS' if (treated and moved and sprayed) else 'INCOMPLETE')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
