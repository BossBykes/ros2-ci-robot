from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node

from ci_bot_monitor.health_evaluator import (
    STATUS_ERROR,
    STATUS_OK,
    odom_data_valid,
    scan_data_valid,
    sensor_status,
)


class SensorHealthMonitor(Node):

    def __init__(self):
        super().__init__("sensor_health_monitor")

        self.scan_timeout = self.declare_parameter(
            "scan_timeout_seconds",
            1.0,
        ).value

        self.odom_timeout = self.declare_parameter(
            "odom_timeout_seconds",
            1.0,
        ).value

        check_period = self.declare_parameter(
            "check_period_seconds",
            0.1,
        ).value

        self.last_scan_time = None
        self.last_odom_time = None

        self.scan_valid = True
        self.odom_valid = True

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )

        self.timer = self.create_timer(
            check_period,
            self.publish_diagnostics,
        )

    def scan_callback(self, message):
        self.last_scan_time = self.get_clock().now()
        self.scan_valid = scan_data_valid(message.ranges)

    def odom_callback(self, message):
        self.last_odom_time = self.get_clock().now()
        self.odom_valid = odom_data_valid(message)

    def age_seconds(self, timestamp):
        if timestamp is None:
            return float("inf")

        return (
            self.get_clock().now() - timestamp
        ).nanoseconds / 1e9

    def make_status(
        self,
        name,
        age,
        timeout,
        data_valid,
    ):
        level, reason = sensor_status(
            age,
            timeout,
            data_valid,
        )

        status = DiagnosticStatus()
        status.name = name

        if level == STATUS_OK:
            status.level = DiagnosticStatus.OK
        elif level == STATUS_ERROR:
            status.level = DiagnosticStatus.ERROR
        else:
            status.level = DiagnosticStatus.WARN

        status.message = reason

        status.values = [
            KeyValue(
                key="age_seconds",
                value=f"{age:.3f}",
            ),
            KeyValue(
                key="timeout_seconds",
                value=f"{timeout:.3f}",
            ),
        ]

        return status

    def publish_diagnostics(self):
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = (
            self.get_clock().now().to_msg()
        )

        diagnostics.status = [
            self.make_status(
                "ci_bot/scan",
                self.age_seconds(self.last_scan_time),
                self.scan_timeout,
                self.scan_valid,
            ),
            self.make_status(
                "ci_bot/odom",
                self.age_seconds(self.last_odom_time),
                self.odom_timeout,
                self.odom_valid,
            ),
        ]

        self.diagnostics_publisher.publish(diagnostics)


def main(args=None):
    rclpy.init(args=args)

    node = SensorHealthMonitor()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
