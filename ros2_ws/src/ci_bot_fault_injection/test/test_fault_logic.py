import math

import pytest

from sensor_msgs.msg import LaserScan

from ci_bot_fault_injection.fault_logic import (
    MODE_DROP,
    MODE_NAN,
    MODE_NORMAL,
    MODE_STALE,
    apply_scan_fault,
)


def make_scan():
    scan = LaserScan()

    scan.header.stamp.sec = 10
    scan.header.stamp.nanosec = 500_000_000

    scan.ranges = [
        0.5,
        1.0,
        2.0,
    ]

    return scan


def test_normal_mode_preserves_scan():
    scan = make_scan()

    result = apply_scan_fault(
        scan,
        MODE_NORMAL,
    )

    assert result is not None
    assert list(result.ranges) == [0.5, 1.0, 2.0]
    assert result.header.stamp.sec == 10
    assert result.header.stamp.nanosec == 500_000_000


def test_drop_mode_returns_none():
    scan = make_scan()

    result = apply_scan_fault(
        scan,
        MODE_DROP,
    )

    assert result is None


def test_nan_mode_corrupts_first_range():
    scan = make_scan()

    result = apply_scan_fault(
        scan,
        MODE_NAN,
    )

    assert result is not None
    assert math.isnan(result.ranges[0])
    assert result.ranges[1] == 1.0
    assert result.ranges[2] == 2.0


def test_stale_mode_changes_timestamp():
    scan = make_scan()

    result = apply_scan_fault(
        scan,
        MODE_STALE,
        stale_offset_seconds=3.0,
    )

    assert result is not None
    assert result.header.stamp.sec == 7
    assert result.header.stamp.nanosec == 500_000_000


def test_unknown_mode_raises_error():
    scan = make_scan()

    with pytest.raises(ValueError):
        apply_scan_fault(
            scan,
            "broken-mode",
        )
