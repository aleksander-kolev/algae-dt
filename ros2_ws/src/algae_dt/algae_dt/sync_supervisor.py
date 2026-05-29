"""sync_supervisor — Real-Time Synchronization & Tolerances (Rubric ②). PLAN.md T3.2.

Computes real<->sim pose/sensor discrepancy (lib.sync) and command->motion + scan-age latency
(lib.metrics); publishes /dt/sync_error, /dt/latency_ms, /dt/sync_ok (Bool); appends a CSV
(sync_metrics_<stamp>.csv); and publishes /dt/alerts (String) when out of the documented tolerances
in twin.yaml. This is the highest-value, most-overlooked deliverable — first-class here.
"""
import rclpy
from rclpy.node import Node


class SyncSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('sync_supervisor')
        self.get_logger().warn('STUB sync_supervisor — implement per PLAN.md T3.2')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyncSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
