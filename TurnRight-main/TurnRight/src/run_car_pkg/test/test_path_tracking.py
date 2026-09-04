"""Unit tests for route construction and forward projection."""

import numpy as np

from run_car_pkg.path_tracking import (
    build_centerline,
    find_lookahead_index,
    project_forward,
    waypoint_indices,
)


def test_centerline_keeps_endpoints_and_monotonic_distance():
    waypoints = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]])
    path, arc_lengths = build_centerline(waypoints, spacing=0.1)

    assert np.allclose(path[0], waypoints[0])
    assert np.allclose(path[-1], waypoints[-1])
    assert np.all(np.diff(arc_lengths) >= 0.0)


def test_waypoint_indices_do_not_move_backward():
    waypoints = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    path, _ = build_centerline(waypoints, spacing=0.1)
    indices = waypoint_indices(path, waypoints)
    assert list(indices) == sorted(indices)


def test_projection_and_lookahead_move_forward():
    waypoints = np.array([[0.0, 0.0], [2.0, 0.0], [4.0, 0.0]])
    path, arc_lengths = build_centerline(waypoints, spacing=0.1)
    projection = project_forward(path, 1.0, 0.2, 0, 100)
    lookahead = find_lookahead_index(
        arc_lengths,
        projection.nearest_index,
        0.5,
    )
    assert projection.cross_track_error >= 0.0
    assert lookahead >= projection.nearest_index
