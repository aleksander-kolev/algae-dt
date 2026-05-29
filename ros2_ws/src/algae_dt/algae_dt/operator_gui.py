"""operator_gui — PyQt5 operator console. PLAN.md T4.2.

Map canvas from maps/map.pgm via lib.pgm; real+sim pose overlay; live /dt/scan_active overlay; bloom
markers (pending/active/done/grey); click-to-place -> /dt/blooms; Start/Stop/Clear/E-STOP (also
Space/Esc); banners for mode/sync/latency/battery/safety/mission. Subscribes /dt/* ONLY (no TF dep).
QTimer-driven spin_once. Headless: QT_QPA_PLATFORM=offscreen. Import of PyQt5 is deferred into main()
so `colcon build` / entry-point discovery never requires PyQt5 to be present.
"""
import rclpy
from rclpy.node import Node


class OperatorGui(Node):
    def __init__(self) -> None:
        super().__init__('operator_gui')
        self.get_logger().warn('STUB operator_gui — implement per PLAN.md T4.2')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorGui()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
