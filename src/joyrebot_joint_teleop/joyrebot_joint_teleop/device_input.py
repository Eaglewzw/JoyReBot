"""Turning raw Joy-Con readings into usable numbers."""

import numpy as np


def report_is_ready(report):
    """True once the driver has actually received an input report.

    The vendored JoyCon starts with a zero-filled buffer, and zeroed stick counts
    decode as full negative deflection -- acting on that would run two joints at
    full speed for the first few cycles after startup.
    """
    return bool(report) and bool(report[0])


def normalize_axis(raw, center, half_range):
    """Map a raw 12-bit Joy-Con stick count to [-1, 1]."""
    return float(np.clip((float(raw) - center) / max(1.0, float(half_range)), -1.0, 1.0))
