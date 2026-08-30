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

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    @staticmethod
    def yaw_from_odometry(odometry):
        orientation = (
            odometry.pose.pose.orientation
        )

        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def publish_stop(self):
        stop_command = Twist()

        deadline = time.monotonic() + 0.25

        while time.monotonic() < deadline:
            self.command_publisher.publish(
                stop_command
            )

            rclpy.spin_once(
                self.node,
                timeout_sec=0.05,
            )

    def drive_to_point(
        self,
        target_x,
        target_y,
        timeout_sec=6.0,
        position_tolerance=0.05,
    ):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.05,
            )

            if not self.received_odometry:
                continue

            odometry = self.received_odometry[-1]

            current_x = (
                odometry.pose.pose.position.x
            )

            current_y = (
                odometry.pose.pose.position.y
            )

            delta_x = target_x - current_x
            delta_y = target_y - current_y

            distance = math.hypot(
                delta_x,
                delta_y,
            )

            if distance <= position_tolerance:
                self.publish_stop()
                return odometry

            target_heading = math.atan2(
                delta_y,
                delta_x,
            )

            current_yaw = self.yaw_from_odometry(
                odometry
            )

            heading_error = self.normalize_angle(
                target_heading - current_yaw
            )

            command = Twist()

            command.angular.z = max(
                -0.8,
                min(
                    0.8,
                    1.8 * heading_error,
                ),
            )

            if abs(heading_error) < 0.45:
                command.linear.x = min(
                    0.25,
                    0.8 * distance,
                )

            self.command_publisher.publish(
                command
            )

        self.publish_stop()

        if self.received_odometry:
            odometry = self.received_odometry[-1]

            current_x = (
                odometry.pose.pose.position.x
            )

            current_y = (
                odometry.pose.pose.position.y
            )

            remaining_distance = math.hypot(
                target_x - current_x,
                target_y - current_y,
            )
        else:
            remaining_distance = math.inf

        self.fail(
            (
                "Timed out driving to waypoint: "
                f"target=({target_x:.3f}, "
                f"{target_y:.3f}), "
                "remaining_distance="
                f"{remaining_distance:.3f}"
            )
        )

    def rotate_to_yaw(
        self,
        target_yaw,
        timeout_sec=5.0,
        yaw_tolerance=0.04,
    ):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.05,
            )

            if not self.received_odometry:
                continue

            odometry = self.received_odometry[-1]

            current_yaw = self.yaw_from_odometry(
                odometry
            )

            yaw_error = self.normalize_angle(
                target_yaw - current_yaw
            )

            if abs(yaw_error) <= yaw_tolerance:
                self.publish_stop()
                return odometry

            command = Twist()

            command.angular.z = max(
                -0.8,
                min(
                    0.8,
                    1.5 * yaw_error,
                ),
            )

            self.command_publisher.publish(
                command
            )

        self.publish_stop()

        if self.received_odometry:
            current_yaw = self.yaw_from_odometry(
                self.received_odometry[-1]
            )

            remaining_error = abs(
                self.normalize_angle(
                    target_yaw - current_yaw
                )
            )
        else:
            remaining_error = math.inf

        self.fail(
            (
                "Timed out rotating to target yaw: "
                f"target_yaw={target_yaw:.3f}, "
                "remaining_error="
                f"{remaining_error:.3f}"
            )
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

    def test_waypoint_navigation_regression(self):
        self.set_fault_mode("normal")

        self.received_odometry.clear()

        start_odometry = self.wait_for_odometry()

        start_x = (
            start_odometry.pose.pose.position.x
        )

        start_y = (
            start_odometry.pose.pose.position.y
        )

        start_yaw = self.yaw_from_odometry(
            start_odometry
        )

        leg_distance = 0.50

        first_target_x = (
            start_x
            + leg_distance * math.cos(start_yaw)
        )

        first_target_y = (
            start_y
            + leg_distance * math.sin(start_yaw)
        )

        turn_target_yaw = self.normalize_angle(
            start_yaw + math.pi / 2.0
        )

        second_target_x = (
            first_target_x
            + leg_distance
            * math.cos(turn_target_yaw)
        )

        second_target_y = (
            first_target_y
            + leg_distance
            * math.sin(turn_target_yaw)
        )

        try:
            first_odometry = self.drive_to_point(
                first_target_x,
                first_target_y,
            )

            first_x = (
                first_odometry.pose.pose.position.x
            )

            first_y = (
                first_odometry.pose.pose.position.y
            )

            first_position_error = math.hypot(
                first_target_x - first_x,
                first_target_y - first_y,
            )

            self.assertLessEqual(
                first_position_error,
                0.08,
                (
                    "Robot missed first navigation "
                    "waypoint: "
                    f"error={first_position_error:.3f} m"
                ),
            )

            turn_odometry = self.rotate_to_yaw(
                turn_target_yaw
            )

            achieved_turn_yaw = (
                self.yaw_from_odometry(
                    turn_odometry
                )
            )

            turn_error = abs(
                self.normalize_angle(
                    turn_target_yaw
                    - achieved_turn_yaw
                )
            )

            self.assertLessEqual(
                turn_error,
                0.08,
                (
                    "Robot missed navigation turn: "
                    f"error={turn_error:.3f} rad"
                ),
            )

            self.drive_to_point(
                second_target_x,
                second_target_y,
            )

            final_odometry = self.rotate_to_yaw(
                turn_target_yaw
            )

            final_x = (
                final_odometry.pose.pose.position.x
            )

            final_y = (
                final_odometry.pose.pose.position.y
            )

            final_yaw = self.yaw_from_odometry(
                final_odometry
            )

            final_position_error = math.hypot(
                second_target_x - final_x,
                second_target_y - final_y,
            )

            final_yaw_error = abs(
                self.normalize_angle(
                    turn_target_yaw - final_yaw
                )
            )

            self.assertLessEqual(
                final_position_error,
                0.08,
                (
                    "Robot missed final navigation "
                    "waypoint: "
                    f"target=({second_target_x:.3f}, "
                    f"{second_target_y:.3f}), "
                    f"actual=({final_x:.3f}, "
                    f"{final_y:.3f}), "
                    "error="
                    f"{final_position_error:.3f} m"
                ),
            )

            self.assertLessEqual(
                final_yaw_error,
                0.12,
                (
                    "Robot final navigation heading "
                    "was outside tolerance: "
                    f"target_yaw={turn_target_yaw:.3f}, "
                    f"actual_yaw={final_yaw:.3f}, "
                    f"error={final_yaw_error:.3f} rad"
                ),
            )
        finally:
            self.publish_stop()
