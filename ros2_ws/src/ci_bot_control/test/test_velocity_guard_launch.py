import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy

from geometry_msgs.msg import Twist
from launch_testing.asserts import assertExitCodes


def generate_test_description():
    velocity_guard_node = launch_ros.actions.Node(
        package="ci_bot_control",
        executable="velocity_guard_node",
        name="velocity_guard",
        output="screen",
        parameters=[
            {
                "max_linear_velocity": 0.5,
                "max_angular_velocity": 1.5,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                velocity_guard_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "velocity_guard_node": velocity_guard_node,
        },
    )


class TestVelocityGuardCommunication(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.node = rclpy.create_node(
            "velocity_guard_integration_test"
        )

        cls.received_messages = []

        cls.publisher = cls.node.create_publisher(
            Twist,
            "/cmd_vel_raw",
            10,
        )

        cls.subscription = cls.node.create_subscription(
            Twist,
            "/cmd_vel",
            cls.received_messages.append,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_command(
        self,
        linear_x,
        angular_z,
        timeout_sec=5.0,
    ):
        self.received_messages.clear()

        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z

        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            self.publisher.publish(command)

            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            if self.received_messages:
                return self.received_messages[-1]

        self.fail("Timed out waiting for /cmd_vel output")

    def test_valid_command_passes_through(self):
        result = self.publish_command(
            linear_x=0.3,
            angular_z=1.0,
        )

        self.assertAlmostEqual(
            result.linear.x,
            0.3,
        )

        self.assertAlmostEqual(
            result.angular.z,
            1.0,
        )

    def test_excessive_command_is_clamped(self):
        result = self.publish_command(
            linear_x=2.0,
            angular_z=-4.0,
        )

        self.assertAlmostEqual(
            result.linear.x,
            0.5,
        )

        self.assertAlmostEqual(
            result.angular.z,
            -1.5,
        )


@launch_testing.post_shutdown_test()
class TestVelocityGuardShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
