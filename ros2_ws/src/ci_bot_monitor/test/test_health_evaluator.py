import math

from ci_bot_monitor.health_evaluator import (
    STATUS_ERROR,
    STATUS_OK,
    scan_data_valid,
    sensor_status,
)


def test_sensor_is_healthy_before_timeout():
    level, reason = sensor_status(
        age_seconds=0.2,
        timeout_seconds=1.0,
        data_valid=True,
    )

    assert level == STATUS_OK
    assert reason == "OK"


def test_sensor_timeout_is_error():
    level, reason = sensor_status(
        age_seconds=1.5,
        timeout_seconds=1.0,
        data_valid=True,
    )

    assert level == STATUS_ERROR
    assert reason == "SENSOR_TIMEOUT"


def test_invalid_data_is_error():
    level, reason = sensor_status(
        age_seconds=0.1,
        timeout_seconds=1.0,
        data_valid=False,
    )

    assert level == STATUS_ERROR
    assert reason == "INVALID_DATA"


def test_scan_accepts_infinite_range():
    assert scan_data_valid(
        [0.5, 1.0, math.inf]
    )


def test_scan_rejects_nan():
    assert not scan_data_valid(
        [0.5, math.nan, 1.0]
    )
