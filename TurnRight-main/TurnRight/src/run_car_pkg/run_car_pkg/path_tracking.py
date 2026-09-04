"""Pure path construction and tracking helpers."""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class PathProjection:
    """Nearest forward path point and its cross-track error."""

    nearest_index: int
    cross_track_error: float


@dataclass(frozen=True)
class RoutePath:
    """Precomputed dense route used by the circle controller."""

    points: np.ndarray
    arc_lengths: np.ndarray
    waypoint_indices: Tuple[int, ...]


def build_centerline(
    waypoints: Sequence[Sequence[float]],
    spacing: float = 0.02,
) -> Tuple[np.ndarray, np.ndarray]:
    """Densify ordered waypoints with an open Catmull-Rom curve."""

    if spacing <= 0.0:
        raise ValueError('spacing must be greater than zero')

    control_points = np.asarray(waypoints, dtype=float)
    if len(control_points) < 2:
        return control_points, np.zeros(len(control_points), dtype=float)

    padded = np.vstack(
        [control_points[0], control_points, control_points[-1]]
    )
    dense_points = []
    for index in range(1, len(padded) - 2):
        p0, p1 = padded[index - 1], padded[index]
        p2, p3 = padded[index + 1], padded[index + 2]
        sample_count = max(
            2,
            int(np.hypot(*(p2 - p1)) / spacing),
        )
        for t in np.linspace(0.0, 1.0, sample_count, endpoint=False):
            t2 = t * t
            t3 = t2 * t
            dense_points.append(
                0.5
                * (
                    2.0 * p1
                    + (-p0 + p2) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
                )
            )

    dense_points.append(padded[-2])
    path = np.asarray(dense_points, dtype=float)
    segment_lengths = np.hypot(
        np.diff(path[:, 0]),
        np.diff(path[:, 1]),
    )
    arc_lengths = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    return path, arc_lengths


def waypoint_indices(
    path: np.ndarray,
    waypoints: Sequence[Sequence[float]],
) -> Tuple[int, ...]:
    """Map ordered waypoints to monotonically increasing path indices."""

    if len(path) == 0:
        return ()

    indices = []
    start = 0
    for waypoint in waypoints:
        segment = path[start:]
        if len(segment) == 0:
            indices.append(len(path) - 1)
            continue
        distance_squared = (
            (segment[:, 0] - waypoint[0]) ** 2
            + (segment[:, 1] - waypoint[1]) ** 2
        )
        nearest = start + int(np.argmin(distance_squared))
        indices.append(nearest)
        start = nearest
    return tuple(indices)


def build_route(
    waypoints: np.ndarray,
    spacing: float = 0.02,
) -> RoutePath:
    """Precompute the current open route and its waypoint event indices."""

    path, arc_lengths = build_centerline(waypoints, spacing)
    event_waypoints = np.vstack([waypoints, waypoints[0]])
    indices = waypoint_indices(path, event_waypoints)
    return RoutePath(path, arc_lengths, indices)


def project_forward(
    path: np.ndarray,
    x: float,
    y: float,
    start_index: int,
    forward_window: int,
) -> PathProjection:
    """Project a vehicle position onto a forward-only path window."""

    if len(path) == 0:
        raise ValueError('cannot project onto an empty path')
    if forward_window <= 0:
        raise ValueError('forward_window must be greater than zero')

    lower = min(max(int(start_index), 0), len(path) - 1)
    upper = min(len(path), lower + int(forward_window))
    if upper <= lower:
        upper = min(len(path), lower + 1)

    segment = path[lower:upper]
    distance_squared = (
        (segment[:, 0] - x) ** 2
        + (segment[:, 1] - y) ** 2
    )
    local_index = int(np.argmin(distance_squared))
    return PathProjection(
        nearest_index=lower + local_index,
        cross_track_error=float(np.sqrt(distance_squared[local_index])),
    )


def find_lookahead_index(
    arc_lengths: np.ndarray,
    nearest_index: int,
    lookahead_distance: float,
) -> int:
    """Find the first path index at least ``lookahead_distance`` ahead."""

    if len(arc_lengths) == 0:
        raise ValueError('arc_lengths must not be empty')
    nearest_index = min(max(int(nearest_index), 0), len(arc_lengths) - 1)
    target_distance = arc_lengths[nearest_index] + lookahead_distance
    return min(
        int(np.searchsorted(arc_lengths, target_distance)),
        len(arc_lengths) - 1,
    )
