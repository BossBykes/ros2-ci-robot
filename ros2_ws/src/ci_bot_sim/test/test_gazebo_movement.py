import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing

import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def generate_test_description():
    sim_share = get_package_share_directory(
        "ci_bot_sim"
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_share,
                "launch",
                "sim.launch.py",
            )
        )
    )

    return launch.LaunchDescription(
        [
            sim_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGazeboMovement(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.node = rclpy.create_node(
            "gazebo_movement_system_test"
        )

        cls.received_odometry = []

        cls.command_publisher = cls.node.create_publisher(
            Twist,
            "/cmd_vel_raw",
            10,
        )

        cls.odom_subscription = cls.node.create_subscription(
            Odometry,
            "/odom",
            cls.received_odometry.append,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for_odometry(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            if self.received_odometry:
                return self.received_odometry[-1]

        self.fail(
            "Timed out waiting for /odom from Gazebo"
        )

    def test_robot_moves_forward_from_guarded_command(self):
        self.received_odometry.clear()

        initial_odometry = self.wait_for_odometry()

        initial_x = (
            initial_odometry.pose.pose.position.x
        )

        command = Twist()
        command.linear.x = 0.2
        command.angular.z = 0.0

        drive_deadline = time.monotonic() + 1.5

        while time.monotonic() < drive_deadline:
            self.command_publisher.publish(command)

            rclpy.spin_once(
                self.node,
                timeout_sec=0.05,
            )

        settle_deadline = time.monotonic() + 1.0

        while time.monotonic() < settle_deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.05,
            )

        self.assertTrue(
            self.received_odometry,
            "No odometry received after commanding movement",
        )

        final_odometry = self.received_odometry[-1]

        final_x = (
            final_odometry.pose.pose.position.x
        )

        displacement = final_x - initial_x

        self.assertGreater(
            displacement,
            0.15,
            (
                "Robot did not move far enough in Gazebo: "
                f"initial_x={initial_x:.3f}, "
                f"final_x={final_x:.3f}, "
                f"displacement={displacement:.3f}"
            ),
        )
