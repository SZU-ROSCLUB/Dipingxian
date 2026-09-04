"""Unit tests for ROS-independent geometry helpers."""

import numpy as np
import pytest

from run_car_pkg.geometry import (
    angle_to_image_x,
    goal_bearing,
    normalize_angle,
)
from run_car_pkg.models import VehiclePose


def test_goal_bearing_straight_ahead():
    pose = VehiclePose(0.0, 0.0, 0.0)
    assert goal_bearing(pose, (1.0, 0.0)) == pytest.approx(0.0)


def test_normalize_angle_stays_in_expected_range():
    result = normalize_angle(4.0)
    assert -np.pi <= result < np.pi


def test_angle_to_image_uses_camera_center_at_zero_error():
    matrix = np.array(
        [[677.0, 0.0, 906.0], [0.0, 676.0, 235.0], [0.0, 0.0, 1.0]]
    )
    assert angle_to_image_x(0.0, matrix) == pytest.approx(906.0)
