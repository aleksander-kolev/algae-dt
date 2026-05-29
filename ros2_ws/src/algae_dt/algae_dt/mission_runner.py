"""mission_runner — navigate to each bloom + spray (Rubric ③ task). PLAN.md T2.2.

Uses nav2_simple_commander.BasicNavigator (node_name='mission_runner_nav', namespace='') to send a
goal to each bloom centre, then spins in place spray_seconds on /dt/cmd_vel_raw ("spraying").
Publishes /dt/markers + /dt/mission_state. Honest accounting (RULES.md §B): spray + mark-treated
ONLY on Nav2 SUCCEEDED (+ sim-only generous center_tol_m); nav-failed -> grey/skipped; aborted spray
-> stays pending. Mission loop on a worker thread. Subscribes /dt/mission_cmd ("start"/"stop") and
/dt/estop. Do NOT set name= on this node in the launch (process-wide remap trap — BEST_APPROACHES).
"""
import rclpy
from rclpy.node import Node


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__('mission_runner')
        self.get_logger().warn('STUB mission_runner — implement per PLAN.md T2.2')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
