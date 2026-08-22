import copy
import math


MODE_NORMAL = "normal"
MODE_DROP = "drop"
MODE_NAN = "nan"
MODE_STALE = "stale"

VALID_MODES = {
    MODE_NORMAL,
    MODE_DROP,
    MODE_NAN,
    MODE_STALE,
}


def subtract_seconds_from_stamp(stamp, seconds):
    total_nanoseconds = (
        int(stamp.sec) * 1_000_000_000
        + int(stamp.nanosec)
    )

    offset_nanoseconds = int(seconds * 1_000_000_000)

    result_nanoseconds = max(
        0,
        total_nanoseconds - offset_nanoseconds,
    )

    stamp.sec = result_nanoseconds // 1_000_000_000
    stamp.nanosec = result_nanoseconds % 1_000_000_000


def apply_scan_fault(
    message,
    mode,
    stale_offset_seconds=5.0,
):
    if mode not in VALID_MODES:
        raise ValueError(
            f"Unsupported fault mode: {mode}"
        )

    if mode == MODE_DROP:
        return None

    output = copy.deepcopy(message)

    if mode == MODE_NORMAL:
        return output

    if mode == MODE_NAN:
        if output.ranges:
            output.ranges[0] = math.nan

        return output

    if mode == MODE_STALE:
        subtract_seconds_from_stamp(
            output.header.stamp,
            stale_offset_seconds,
        )

        return output

    return output
