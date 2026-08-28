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
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


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

        cls.fault_parameter_client = AsyncParameterClient(
            cls.node,
            "scan_fault_injector",
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
    def stamp_nanoseconds(message):
        return (
            int(message.header.stamp.sec)
            * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    @staticmethod
    def diagnostic_level_value(level):
        if isinstance(level, (bytes, bytearray)):
            return level[0]

        return int(level)

    @staticmethod
    def status_by_name(diagnostics, name):
        for status in diagnostics.status:
            if status.name == name:
                return status

        return None

    def set_fault_mode(
        self,
        mode,
        timeout_sec=5.0,
    ):
        services_available = (
            self.fault_parameter_client.wait_for_services(
                timeout_sec=timeout_sec
            )
        )

        self.assertTrue(
            services_available,
            (
                "Timed out waiting for scan_fault_injector "
                "parameter services"
            ),
        )

        future = self.fault_parameter_client.set_parameters(
            [
                Parameter(
                    "fault_mode",
                    value=mode,
                )
            ]
        )

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=timeout_sec,
        )

        self.assertTrue(
            future.done(),
            (
                "Timed out setting scan_fault_injector "
                f"fault_mode={mode}"
            ),
        )

        response = future.result()

        self.assertIsNotNone(
            response,
            (
                "scan_fault_injector parameter service "
                "returned no response"
            ),
        )

        results = response.results

        self.assertEqual(
            len(results),
            1,
            (
                "Unexpected number of results while setting "
                "scan_fault_injector fault_mode"
            ),
        )

        self.assertTrue(
            results[0].successful,
            (
                "Failed to set scan_fault_injector "
                f"fault_mode={mode}: {results[0].reason}"
            ),
        )

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

    def find_nan_scan_pair(self):
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

            if forwarded_scan is None:
                continue

            if (
                raw_scan.ranges
                and forwarded_scan.ranges
                and not math.isnan(raw_scan.ranges[0])
                and math.isnan(
                    forwarded_scan.ranges[0]
                )
            ):
                return raw_scan, forwarded_scan

        return None

    def find_stale_scan_pair(
        self,
        stale_offset_seconds=5.0,
    ):
        offset_nanoseconds = int(
            stale_offset_seconds
            * 1_000_000_000
        )

        forwarded_by_stamp = {
            self.stamp_key(message): message
            for message in self.received_scans
        }

        for raw_scan in reversed(
            self.received_raw_scans
        ):
            raw_nanoseconds = (
                self.stamp_nanoseconds(raw_scan)
            )

            if raw_nanoseconds <= offset_nanoseconds:
                continue

            expected_nanoseconds = (
                raw_nanoseconds
                - offset_nanoseconds
            )

            expected_stamp = (
                expected_nanoseconds
                // 1_000_000_000,
                expected_nanoseconds
                % 1_000_000_000,
            )

            forwarded_scan = (
                forwarded_by_stamp.get(
                    expected_stamp
                )
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
            scan_status = self.status_by_name(
                diagnostics,
                "ci_bot/scan",
            )
            odom_status = self.status_by_name(
                diagnostics,
                "ci_bot/odom",
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

    def find_fault_diagnostics(
        self,
        expected_scan_message,
    ):
        expected_ok_level = self.diagnostic_level_value(
            DiagnosticStatus.OK
        )

        expected_error_level = (
            self.diagnostic_level_value(
                DiagnosticStatus.ERROR
            )
        )

        for diagnostics in reversed(
            self.received_diagnostics
        ):
            scan_status = self.status_by_name(
                diagnostics,
                "ci_bot/scan",
            )
            odom_status = self.status_by_name(
                diagnostics,
                "ci_bot/odom",
            )

            if (
                scan_status is not None
                and odom_status is not None
                and self.diagnostic_level_value(
                    scan_status.level
                )
                == expected_error_level
                and scan_status.message
                == expected_scan_message
                and self.diagnostic_level_value(
                    odom_status.level
                )
                == expected_ok_level
                and odom_status.message == "OK"
            ):
                return diagnostics

        return None

    def wait_for_raw_scan_after(
        self,
        minimum_stamp_seconds,
        timeout_sec=8.0,
    ):
        minimum_nanoseconds = int(
            minimum_stamp_seconds
            * 1_000_000_000
        )

        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            for raw_scan in reversed(
                self.received_raw_scans
            ):
                if (
                    self.stamp_nanoseconds(raw_scan)
                    > minimum_nanoseconds
                ):
                    return raw_scan

        self.fail(
            "Timed out waiting for Gazebo /scan_raw "
            f"timestamp > {minimum_stamp_seconds:.1f}s"
        )

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

    def wait_for_nan_fault_response(
        self,
        timeout_sec=5.0,
    ):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            scan_pair = self.find_nan_scan_pair()

            diagnostics = self.find_fault_diagnostics(
                "INVALID_DATA"
            )

            if (
                scan_pair is not None
                and diagnostics is not None
            ):
                return (
                    scan_pair[0],
                    scan_pair[1],
                    diagnostics,
                )

        self.fail(
            "Timed out waiting for NaN fault to produce "
            "corrupted /scan and INVALID_DATA diagnostics"
        )

    def wait_for_drop_fault_response(
        self,
        timeout_sec=5.0,
    ):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            diagnostics = self.find_fault_diagnostics(
                "SENSOR_TIMEOUT"
            )

            if (
                self.received_raw_scans
                and diagnostics is not None
            ):
                return diagnostics

        self.fail(
            "Timed out waiting for dropped /scan stream "
            "to produce SENSOR_TIMEOUT diagnostics"
        )

    def wait_for_stale_fault_response(
        self,
        timeout_sec=5.0,
    ):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

            scan_pair = self.find_stale_scan_pair(
                stale_offset_seconds=5.0,
            )

            diagnostics = self.find_fault_diagnostics(
                "SENSOR_TIMEOUT"
            )

            if (
                scan_pair is not None
                and diagnostics is not None
            ):
                return (
                    scan_pair[0],
                    scan_pair[1],
                    diagnostics,
                )

        self.fail(
            "Timed out waiting for stale /scan timestamp "
            "to produce SENSOR_TIMEOUT diagnostics"
        )

    def test_gazebo_lidar_pipeline_is_healthy(self):
        self.set_fault_mode("normal")

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

    def test_scan_drop_fault_reports_timeout(self):
        self.set_fault_mode("normal")

        self.received_raw_scans.clear()
        self.received_scans.clear()
        self.received_diagnostics.clear()

        try:
            self.set_fault_mode("drop")

            self.received_raw_scans.clear()
            self.received_scans.clear()
            self.received_diagnostics.clear()

            diagnostics = (
                self.wait_for_drop_fault_response()
            )

            self.assertTrue(
                self.received_raw_scans,
                (
                    "Gazebo /scan_raw stopped during "
                    "drop fault injection"
                ),
            )

            scan_status = self.status_by_name(
                diagnostics,
                "ci_bot/scan",
            )

            odom_status = self.status_by_name(
                diagnostics,
                "ci_bot/odom",
            )

            expected_error_level = (
                self.diagnostic_level_value(
                    DiagnosticStatus.ERROR
                )
            )

            expected_ok_level = (
                self.diagnostic_level_value(
                    DiagnosticStatus.OK
                )
            )

            self.assertEqual(
                self.diagnostic_level_value(
                    scan_status.level
                ),
                expected_error_level,
            )

            self.assertEqual(
                scan_status.message,
                "SENSOR_TIMEOUT",
            )

            self.assertEqual(
                self.diagnostic_level_value(
                    odom_status.level
                ),
                expected_ok_level,
            )

            self.assertEqual(
                odom_status.message,
                "OK",
            )
        finally:
            self.set_fault_mode("normal")

    def test_scan_nan_fault_reports_invalid_data(self):
        self.set_fault_mode("normal")

        self.received_raw_scans.clear()
        self.received_scans.clear()
        self.received_diagnostics.clear()

        try:
            self.set_fault_mode("nan")

            self.received_raw_scans.clear()
            self.received_scans.clear()
            self.received_diagnostics.clear()

            (
                raw_scan,
                forwarded_scan,
                diagnostics,
            ) = self.wait_for_nan_fault_response()

            self.assertEqual(
                self.stamp_key(raw_scan),
                self.stamp_key(forwarded_scan),
            )

            self.assertFalse(
                math.isnan(raw_scan.ranges[0]),
                (
                    "Gazebo /scan_raw was unexpectedly "
                    "NaN-corrupted"
                ),
            )

            self.assertTrue(
                math.isnan(
                    forwarded_scan.ranges[0]
                ),
                (
                    "NaN fault did not corrupt "
                    "/scan ranges[0]"
                ),
            )

            self.assertEqual(
                list(raw_scan.ranges[1:]),
                list(forwarded_scan.ranges[1:]),
                (
                    "NaN fault changed ranges other "
                    "than ranges[0]"
                ),
            )

            scan_status = self.status_by_name(
                diagnostics,
                "ci_bot/scan",
            )

            odom_status = self.status_by_name(
                diagnostics,
                "ci_bot/odom",
            )

            expected_error_level = (
                self.diagnostic_level_value(
                    DiagnosticStatus.ERROR
                )
            )

            expected_ok_level = (
                self.diagnostic_level_value(
                    DiagnosticStatus.OK
                )
            )

            self.assertEqual(
                self.diagnostic_level_value(
                    scan_status.level
                ),
                expected_error_level,
            )

            self.assertEqual(
                scan_status.message,
                "INVALID_DATA",
            )

            self.assertEqual(
                self.diagnostic_level_value(
                    odom_status.level
                ),
                expected_ok_level,
            )

            self.assertEqual(
                odom_status.message,
                "OK",
            )
        finally:
            self.set_fault_mode("normal")

    def test_scan_stale_fault_reports_timeout(self):
        self.set_fault_mode("normal")

        self.received_raw_scans.clear()
        self.received_scans.clear()
        self.received_diagnostics.clear()

        self.wait_for_raw_scan_after(
            minimum_stamp_seconds=5.5,
        )

        try:
            self.set_fault_mode("stale")

            self.received_raw_scans.clear()
            self.received_scans.clear()
            self.received_diagnostics.clear()

            (
                raw_scan,
                forwarded_scan,
                diagnostics,
            ) = self.wait_for_stale_fault_response()

            self.assertGreater(
                self.stamp_nanoseconds(raw_scan),
                5_000_000_000,
                (
                    "Gazebo /scan_raw timestamp was not "
                    "large enough for stale injection"
                ),
            )

            self.assertNotEqual(
                self.stamp_nanoseconds(
                    forwarded_scan
                ),
                0,
                (
                    "Stale fault unexpectedly produced "
                    "a zero timestamp"
                ),
            )

            timestamp_difference = (
                self.stamp_nanoseconds(raw_scan)
                - self.stamp_nanoseconds(
                    forwarded_scan
                )
            )

            self.assertEqual(
                timestamp_difference,
                5_000_000_000,
                (
                    "Stale fault did not shift the "
                    "LaserScan timestamp by exactly 5.0 s"
                ),
            )

            self.assertEqual(
                list(raw_scan.ranges),
                list(forwarded_scan.ranges),
                (
                    "Stale fault changed LaserScan ranges"
                ),
            )

            scan_status = self.status_by_name(
                diagnostics,
                "ci_bot/scan",
            )

            odom_status = self.status_by_name(
                diagnostics,
                "ci_bot/odom",
            )

            expected_error_level = (
                self.diagnostic_level_value(
                    DiagnosticStatus.ERROR
                )
            )

            expected_ok_level = (
                self.diagnostic_level_value(
                    DiagnosticStatus.OK
                )
            )

            self.assertEqual(
                self.diagnostic_level_value(
                    scan_status.level
                ),
                expected_error_level,
            )

            self.assertEqual(
                scan_status.message,
                "SENSOR_TIMEOUT",
            )

            self.assertEqual(
                self.diagnostic_level_value(
                    odom_status.level
                ),
                expected_ok_level,
            )

            self.assertEqual(
                odom_status.message,
                "OK",
            )
        finally:
            self.set_fault_mode("normal")
