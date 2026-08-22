import time

import pytest
import rclpy

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import launch
import launch_pytest
import launch_ros.actions


pytestmark = pytest.mark.timeout(15)


@pytest.fixture(scope="function")
def monitor_node():
    return launch_ros.actions.Node(
        package="ci_bot_monitor",
        executable="sensor_health_monitor",
        name="sensor_health_monitor",
        output="screen",
        parameters=[
            {
                "scan_timeout_seconds": 0.5,
                "odom_timeout_seconds": 0.5,
                "check_period_seconds": 0.1,
            }
        ],
    )


@launch_pytest.fixture
def launch_description(monitor_node):
    return launch.LaunchDescription(
        [
            monitor_node,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.fixture
def ros_node():
    rclpy.init()

    node = rclpy.create_node(
        "sensor_health_monitor_integration_test"
    )

    yield node

    node.destroy_node()
    rclpy.shutdown()


def wait_for_diagnostics(
    node,
    received_messages,
    condition,
    timeout_sec=5.0,
):
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

        for message in received_messages:
            if condition(message):
                return message

    pytest.fail(
        "Timed out waiting for expected diagnostics"
    )


def status_by_name(message, name):
    for status in message.status:
        if status.name == name:
            return status

    return None


@pytest.mark.launch(fixture=launch_description)
def test_healthy_scan_and_odom_report_ok(ros_node):
    received_diagnostics = []

    scan_publisher = ros_node.create_publisher(
        LaserScan,
        "/scan",
        10,
    )

    odom_publisher = ros_node.create_publisher(
        Odometry,
        "/odom",
        10,
    )

    diagnostics_subscription = ros_node.create_subscription(
        DiagnosticArray,
        "/diagnostics",
        received_diagnostics.append,
        10,
    )

    scan = LaserScan()
    scan.ranges = [0.5, 1.0, 2.0]

    odom = Odometry()

    publish_deadline = time.monotonic() + 1.0

    while time.monotonic() < publish_deadline:
        scan_publisher.publish(scan)
        odom_publisher.publish(odom)

        rclpy.spin_once(
            ros_node,
            timeout_sec=0.05,
        )

    result = wait_for_diagnostics(
        ros_node,
        received_diagnostics,
        lambda message: (
            status_by_name(
                message,
                "ci_bot/scan",
            ) is not None
            and status_by_name(
                message,
                "ci_bot/odom",
            ) is not None
            and status_by_name(
                message,
                "ci_bot/scan",
            ).message == "OK"
            and status_by_name(
                message,
                "ci_bot/odom",
            ).message == "OK"
        ),
    )

    scan_status = status_by_name(
        result,
        "ci_bot/scan",
    )

    odom_status = status_by_name(
        result,
        "ci_bot/odom",
    )

    assert scan_status.message == "OK"
    assert odom_status.message == "OK"

    ros_node.destroy_subscription(
        diagnostics_subscription
    )


@pytest.mark.launch(fixture=launch_description)
def test_sensor_timeout_reports_error(ros_node):
    received_diagnostics = []

    diagnostics_subscription = ros_node.create_subscription(
        DiagnosticArray,
        "/diagnostics",
        received_diagnostics.append,
        10,
    )

    result = wait_for_diagnostics(
        ros_node,
        received_diagnostics,
        lambda message: (
            status_by_name(
                message,
                "ci_bot/scan",
            ) is not None
            and status_by_name(
                message,
                "ci_bot/scan",
            ).message == "SENSOR_TIMEOUT"
        ),
        timeout_sec=3.0,
    )

    scan_status = status_by_name(
        result,
        "ci_bot/scan",
    )

    assert scan_status.message == "SENSOR_TIMEOUT"

    ros_node.destroy_subscription(
        diagnostics_subscription
    )
