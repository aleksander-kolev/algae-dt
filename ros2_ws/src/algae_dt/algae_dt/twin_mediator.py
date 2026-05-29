"""twin_mediator — the DT Integration Node (fan-in / fan-out). PLAN.md T1.4 + T2.3.

Single command chokepoint: subscribes /dt/cmd_vel_raw + both scans, applies the 25 cm dual-LiDAR
safety gate (lib.safety, fail-safe), and fans the safe command out to /cmd_vel (TwistStamped, real)
AND /sim/cmd_vel (sim). Mirrors /odom->/dt/real_pose, sim pose->/dt/sim_pose, battery->/dt/health;
republishes the ACTIVE robot's scan/odom to /dt/scan_active + /dt/odom_active; owns the latched
/dt/estop (auto on critical battery). See RULES.md §B (TwistStamped, topic-collision, fail-safe).
"""
import rclpy
from rclpy.node import Node


class TwinMediator(Node):
    def __init__(self) -> None:
        super().__init__('twin_mediator')
        self.get_logger().warn('STUB twin_mediator — implement per PLAN.md T1.4/T2.3')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinMediator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
