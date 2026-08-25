from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node

from ci_bot_fault_injection.fault_logic import (
    MODE_NORMAL,
    apply_scan_fault,
)


class ScanFaultInjector(Node):

    def __init__(self):
        super().__init__("scan_fault_injector")

        self.declare_parameter(
            "fault_mode",
            MODE_NORMAL,
        )

        self.declare_parameter(
            "stale_offset_seconds",
            5.0,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.scan_callback,
            10,
        )

        self.publisher = self.create_publisher(
            LaserScan,
            "/scan",
            10,
        )

    def scan_callback(self, message):
        mode = self.get_parameter(
            "fault_mode"
        ).value

        stale_offset_seconds = self.get_parameter(
            "stale_offset_seconds"
        ).value

        try:
            output = apply_scan_fault(
                message,
                mode,
                stale_offset_seconds,
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        if output is not None:
            self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = ScanFaultInjector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
