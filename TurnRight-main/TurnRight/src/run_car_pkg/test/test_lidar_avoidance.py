"""Unit tests for lidar avoidance and rear-obstacle detection."""

import numpy as np

from run_car_pkg.config import LidarConfig, ReverseConfig
from run_car_pkg.lidar_avoidance import (
    choose_avoidance_bearing,
    has_rear_obstacle,
)


def _lidar_config():
    return LidarConfig(
        enabled=True,
        fov=1.4,
        influence_distance=0.9,
        minimum_range=0.1,
        inflation_radius=0.24,
        minimum_gap=0.0,
        edge_margin=0.1,
        invert=1.0,
    )


def _reverse_config():
    return ReverseConfig(
        speed=-0.8,
        angular_speed=4.0,
        maximum_time=1.3,
        stop_yaw=1.8,
        obstacle_distance=0.38,
        rear_fov=0.5,
    )


def test_no_near_obstacle_preserves_goal_bearing():
    result = choose_avoidance_bearing(
        0.2,
        np.array([0.0]),
        np.array([2.0]),
        _lidar_config(),
    )
    assert result.bearing == 0.2
    assert not result.obstacle_detected


def test_rear_obstacle_uses_rear_sector_and_distance():
    betas = np.array([0.0, np.pi - 0.1])
    ranges = np.array([2.0, 0.2])
    assert has_rear_obstacle(
        betas,
        ranges,
        _lidar_config(),
        _reverse_config(),
    )
