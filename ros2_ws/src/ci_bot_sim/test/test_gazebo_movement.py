import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing

import rclpy


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
            "gazebo_system_test"
        )

        cls.received_odometry = []
        cls.received_raw_scans = []
        cls.received_scans = []
        cls.received_diagnostics = []

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

        cls.raw_scan_subscription = cls.node.create_subscription(
            LaserScan,
            "/scan_raw",
            cls.received_raw_scans.append,
            10,
        )

        cls.scan_subscription = cls.node.create_subscription(
            LaserScan,
            "/scan",
            cls.received_scans.append,
            10,
        )

        cls.diagnostics_subscription = cls.node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            cls.received_diagnostics.append,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def stamp_key(message):
        return (
            message.header.stamp.sec,
            message.header.stamp.nanosec,
        )

    @staticmethod
    def diagnostic_level_value(level):
        if isinstance(level, (bytes, bytearray)):
            return level[0]

        return int(level)

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

    def find_matching_scan_pair(self):
        forwarded_by_stamp = {
            self.stamp_key(message): message
            for message in self.received_scans
        }

        for raw_scan in reversed(
            self.received_raw_scans
        ):
            forwarded_scan = forwarded_by_stamp.get(
                self.stamp_key(raw_scan)
            )

            if forwarded_scan is not None:
                return raw_scan, forwarded_scan

        return None

    def find_healthy_diagnostics(self):
        expected_ok_level = self.diagnostic_level_value(
            DiagnosticStatus.OK
        )

        for diagnostics in reversed(
            self.received_diagnostics
        ):
            statuses = {
                status.name: status
                for status in diagnostics.status
            }

            scan_status = statuses.get(
                "ci_bot/scan"
            )
            odom_status = statuses.get(
                "ci_bot/odom"
            )

            if (
                scan_status is not None
                and odom_status is not None
                and self.diagnostic_level_value(
                    scan_status.level
                )
                == expected_ok_level
                and self.diagnostic_level_value(
                    odom_status.level
                )
                == expected_ok_level
                and scan_status.message == "OK"
                and odom_status.message == "OK"
            ):
                return diagnostics

        return None

    def wait_for_sensor_pipeline(
        self,
        timeout_sec=12.0,
    ):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            matching_scans = (
                self.find_matching_scan_pair()
            )

            healthy_diagnostics = (
                self.find_healthy_diagnostics()
            )

            if (
                matching_scans is not None
                and healthy_diagnostics is not None
            ):
                return (
                    matching_scans[0],
                    matching_scans[1],
                    healthy_diagnostics,
                )

        self.fail(
            "Timed out waiting for healthy Gazebo "
            "LiDAR -> fault injector -> diagnostics pipeline"
        )

    def test_gazebo_lidar_pipeline_is_healthy(self):
        self.received_raw_scans.clear()
        self.received_scans.clear()
        self.received_diagnostics.clear()

        (
            raw_scan,
            forwarded_scan,
            diagnostics,
        ) = self.wait_for_sensor_pipeline()

        self.assertEqual(
            raw_scan.header.frame_id,
            "laser_frame",
        )

        self.assertEqual(
            forwarded_scan.header.frame_id,
            "laser_frame",
        )

        self.assertEqual(
            len(raw_scan.ranges),
            360,
            "Gazebo LiDAR did not produce 360 ranges",
        )

        self.assertEqual(
            len(forwarded_scan.ranges),
            360,
            "Forwarded /scan did not contain 360 ranges",
        )

        self.assertEqual(
            self.stamp_key(raw_scan),
            self.stamp_key(forwarded_scan),
            (
                "/scan was not matched to the same "
                "Gazebo /scan_raw sample"
            ),
        )

        self.assertEqual(
            list(raw_scan.ranges),
            list(forwarded_scan.ranges),
            (
                "Normal fault-injector mode changed "
                "the Gazebo LaserScan ranges"
            ),
        )

        finite_ranges = [
            value
            for value in raw_scan.ranges
            if math.isfinite(value)
        ]

        self.assertTrue(
            finite_ranges,
            (
                "Gazebo LiDAR produced no finite "
                "obstacle returns"
            ),
        )

        nearest_obstacle = min(finite_ranges)

        self.assertGreaterEqual(
            nearest_obstacle,
            raw_scan.range_min,
        )

        self.assertLessEqual(
            nearest_obstacle,
            raw_scan.range_max,
        )

        self.assertLess(
            nearest_obstacle,
            3.0,
            (
                "Gazebo LiDAR did not detect the "
                "lidar_test_wall within 3.0 m: "
                f"nearest={nearest_obstacle:.3f}"
            ),
        )

        statuses = {
            status.name: status
            for status in diagnostics.status
        }

        expected_ok_level = self.diagnostic_level_value(
            DiagnosticStatus.OK
        )

        for status_name in (
            "ci_bot/scan",
            "ci_bot/odom",
        ):
            self.assertIn(
                status_name,
                statuses,
            )

            status = statuses[status_name]

            self.assertEqual(
                self.diagnostic_level_value(
                    status.level
                ),
                expected_ok_level,
                (
                    f"{status_name} diagnostic level "
                    "was not OK"
                ),
            )

            self.assertEqual(
                status.message,
                "OK",
                (
                    f"{status_name} diagnostic message "
                    "was not OK"
                ),
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
