"""Pure lidar obstacle-avoidance calculations."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from run_car_pkg.config import LidarConfig, ReverseConfig


@dataclass(frozen=True)
class AvoidanceResult:
    """Selected bearing and obstacle-state information."""

    bearing: Optional[float]
    obstacle_detected: bool
    fully_blocked: bool


def scan_bearings(
    angle_min: float,
    angle_increment: float,
    sample_count: int,
    invert: float,
) -> np.ndarray:
    """Convert LaserScan sample indices into normalized vehicle bearings."""

    angles = angle_min + np.arange(sample_count) * angle_increment
    return invert * (((angles + np.pi) % (2.0 * np.pi)) - np.pi)


def choose_avoidance_bearing(
    goal_bearing: float,
    betas: Optional[np.ndarray],
    ranges: Optional[np.ndarray],
    config: LidarConfig,
    bypass: bool = False,
) -> AvoidanceResult:
    """Choose a free bearing nearest to the requested goal bearing."""

    if not config.enabled:
        return AvoidanceResult(float(goal_bearing), False, False)

    clipped_goal = float(
        np.clip(goal_bearing, -config.fov, config.fov)
    )
    if betas is None or ranges is None:
        return AvoidanceResult(clipped_goal, False, False)

    mask = (
        (np.abs(betas) <= config.fov)
        & np.isfinite(ranges)
        & (ranges > config.minimum_range)
        & (ranges < config.influence_distance)
    )
    obstacle_bearings = betas[mask]
    obstacle_ranges = ranges[mask]
    if obstacle_bearings.size == 0 or bypass:
        return AvoidanceResult(clipped_goal, False, False)

    inflation_angles = np.arctan2(
        config.inflation_radius,
        np.maximum(obstacle_ranges, 0.1),
    )
    lower_bounds = obstacle_bearings - inflation_angles
    upper_bounds = obstacle_bearings + inflation_angles
    merged = _merge_intervals(lower_bounds, upper_bounds)

    if not _is_blocked(clipped_goal, merged):
        return AvoidanceResult(clipped_goal, False, False)

    free_intervals = _free_intervals(merged, config)
    if not free_intervals:
        return AvoidanceResult(None, True, True)

    best_bearing = None
    best_distance = float('inf')
    for lower, upper in free_intervals:
        if lower + config.edge_margin > upper - config.edge_margin:
            candidate = 0.5 * (lower + upper)
        else:
            candidate = min(
                max(clipped_goal, lower + config.edge_margin),
                upper - config.edge_margin,
            )
        distance = abs(candidate - clipped_goal)
        if distance < best_distance:
            best_distance = distance
            best_bearing = candidate

    return AvoidanceResult(float(best_bearing), True, False)


def choose_escape_bearing(
    betas: Optional[np.ndarray],
    ranges: Optional[np.ndarray],
    config: LidarConfig,
) -> float:
    """Choose the more open side when the complete front sector is blocked."""

    if betas is None or ranges is None:
        return 0.0

    mask = (
        (np.abs(betas) <= config.fov)
        & np.isfinite(ranges)
        & (ranges > config.minimum_range)
    )
    if not np.any(mask):
        return 0.0

    valid_betas = betas[mask]
    valid_ranges = ranges[mask]
    left_room = (
        valid_ranges[valid_betas > 0].min()
        if np.any(valid_betas > 0)
        else 0.0
    )
    right_room = (
        valid_ranges[valid_betas < 0].min()
        if np.any(valid_betas < 0)
        else 0.0
    )
    edge = config.fov - config.edge_margin
    return float(edge if left_room >= right_room else -edge)


def has_rear_obstacle(
    betas: Optional[np.ndarray],
    ranges: Optional[np.ndarray],
    lidar_config: LidarConfig,
    reverse_config: ReverseConfig,
) -> bool:
    """Return whether the rear sector contains a nearby obstacle."""

    if betas is None or ranges is None:
        return False

    mask = (
        (np.abs(betas) > (np.pi - reverse_config.rear_fov))
        & np.isfinite(ranges)
        & (ranges > lidar_config.minimum_range)
    )
    if not np.any(mask):
        return False
    return bool(
        float(np.min(ranges[mask]))
        < reverse_config.obstacle_distance
    )


def _merge_intervals(
    lower_bounds: np.ndarray,
    upper_bounds: np.ndarray,
) -> Tuple[Tuple[float, float], ...]:
    order = np.argsort(lower_bounds)
    lower_bounds = lower_bounds[order]
    upper_bounds = upper_bounds[order]

    merged = []
    current_lower = float(lower_bounds[0])
    current_upper = float(upper_bounds[0])
    for index in range(1, lower_bounds.size):
        lower = float(lower_bounds[index])
        upper = float(upper_bounds[index])
        if lower <= current_upper:
            current_upper = max(current_upper, upper)
        else:
            merged.append((current_lower, current_upper))
            current_lower, current_upper = lower, upper
    merged.append((current_lower, current_upper))
    return tuple(merged)


def _is_blocked(
    bearing: float,
    intervals: Tuple[Tuple[float, float], ...],
) -> bool:
    return any(lower <= bearing <= upper for lower, upper in intervals)


def _free_intervals(
    occupied: Tuple[Tuple[float, float], ...],
    config: LidarConfig,
) -> Tuple[Tuple[float, float], ...]:
    free = []
    cursor = -config.fov
    for lower, upper in occupied:
        if upper <= -config.fov:
            continue
        if lower >= config.fov:
            break

        clipped_lower = max(lower, -config.fov)
        clipped_upper = min(upper, config.fov)
        if clipped_lower > cursor:
            free.append((cursor, clipped_lower))
        cursor = max(cursor, clipped_upper)

    if cursor < config.fov:
        free.append((cursor, config.fov))
    return tuple(
        (lower, upper)
        for lower, upper in free
        if upper - lower >= config.minimum_gap
    )
