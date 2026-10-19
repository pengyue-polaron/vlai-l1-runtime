"""Pure geometry of bounded diagnostic paths; no hardware imports or operations."""

from __future__ import annotations

import math
from itertools import pairwise

from .configuration import MOTOR_NAMES

WAYPOINT_KIND = "isolated_right_follower_waypoints_v1"


def is_waypoint_policy(policy):
    return type(policy) is dict and policy.get("kind") == WAYPOINT_KIND


def path_offsets(policy):
    """Named inputs are validated by the diagnostic policy loader before use."""
    if is_waypoint_policy(policy):
        return tuple(
            tuple(point[name] for name in MOTOR_NAMES) for point in policy["waypoints_deg"]
        )
    active = MOTOR_NAMES.index(policy["active_motor"])
    return tuple(
        tuple(value if i == active else 0.0 for i in range(8))
        for value in (0.0, policy["amplitude_deg"], 0.0, -policy["amplitude_deg"], 0.0)
    )


def sweep_duration(policy):
    return policy["hold_seconds"] + policy["cycles"] * (len(path_offsets(policy)) - 1) * (
        policy["transition_seconds"] + policy["hold_seconds"]
    )


def target_offsets(policy, seconds, *, hold_only=False):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("invalid trajectory time")
    if hold_only or seconds <= policy["hold_seconds"] or seconds >= sweep_duration(policy):
        return (0.0,) * 8
    points = path_offsets(policy)
    segment, local = divmod(
        seconds - policy["hold_seconds"], policy["transition_seconds"] + policy["hold_seconds"]
    )
    leg = int(segment) % (len(points) - 1)
    u = min(1.0, local / policy["transition_seconds"])
    blend = u**3 * (10 - 15 * u + 6 * u * u)
    return tuple(a + (b - a) * blend for a, b in zip(points[leg], points[leg + 1], strict=True))


def validate_path(policy):
    """Closed, bounded, finite exact-name vectors; analytic per-segment budgets."""
    points = policy["waypoints_deg"]
    if type(points) is not list or not 3 <= len(points) <= 64:
        raise ValueError("waypoint path requires 3..64 points")
    for point in points:
        if type(point) is not dict or set(point) != set(MOTOR_NAMES):
            raise ValueError("exact named waypoint vectors required")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in point.values()):
            raise ValueError("finite waypoint values required")
        if point["gripper"] != 0:
            raise ValueError("waypoint gripper must remain fixed")
        for i, name in enumerate(MOTOR_NAMES):
            if abs(point[name]) > min(
                policy["amplitude_deg"], policy["maximum_relative_excursion_deg"][i]
            ):
                raise ValueError("waypoint exceeds excursion/amplitude envelope")
    if any(points[i][n] != 0 for i in (0, -1) for n in MOTOR_NAMES):
        raise ValueError("waypoint path must start and finish at zero offset")
    if sum(any(p[n] != 0 for p in points) for n in MOTOR_NAMES) < 2:
        raise ValueError("waypoint path must excite multiple joints")
    dt = policy["transition_seconds"]
    for a, b in pairwise(points):
        for name in MOTOR_NAMES:
            distance = abs(b[name] - a[name])
            if (
                1.875 * distance / dt > policy["maximum_reference_speed_deg_s"]
                or 10 * math.sqrt(3) / 3 * distance / dt**2
                > policy["maximum_reference_acceleration_deg_s2"]
            ):
                raise ValueError("waypoint segment exceeds speed/acceleration budget")
