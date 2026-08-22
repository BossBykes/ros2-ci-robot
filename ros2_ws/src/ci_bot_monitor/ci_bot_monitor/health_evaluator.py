import math


STATUS_OK = 0
STATUS_WARN = 1
STATUS_ERROR = 2


def sensor_status(
    age_seconds,
    timeout_seconds,
    data_valid=True,
):
    if not data_valid:
        return STATUS_ERROR, "INVALID_DATA"

    if age_seconds > timeout_seconds:
        return STATUS_ERROR, "SENSOR_TIMEOUT"

    return STATUS_OK, "OK"


def scan_data_valid(ranges):
    return all(
        not math.isnan(value)
        for value in ranges
    )


def odom_data_valid(message):
    values = [
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z,
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.linear.z,
        message.twist.twist.angular.x,
        message.twist.twist.angular.y,
        message.twist.twist.angular.z,
    ]

    return all(math.isfinite(value) for value in values)
