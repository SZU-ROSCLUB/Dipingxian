"""Pure geometry helpers for steering and navigation."""

from typing import Sequence

import numpy as np

from run_car_pkg.models import VehiclePose


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the interval [-pi, pi)."""

    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def goal_bearing(pose: VehiclePose, goal: Sequence[float]) -> float:
    """Return a goal bearing relative to the vehicle heading, left positive."""

    target_angle = np.arctan2(goal[1] - pose.y, goal[0] - pose.x)
    return normalize_angle(float(target_angle) - pose.yaw)


def angle_to_image_x(angle_error: float, camera_matrix: np.ndarray) -> float:
    """Map a bearing error to the image x coordinate used by the PID loop."""

    principal_x = float(camera_matrix[0, 2])
    focal_x = float(camera_matrix[0, 0])
    if -1.5 <= angle_error <= 1.5:
        return principal_x + focal_x * float(np.tan(angle_error))
    if angle_error <= -1.5:
        return principal_x + focal_x * (200.0 * angle_error + 286.0)
    return principal_x + focal_x * (200.0 * angle_error - 286.0)


def squared_distance(pose: VehiclePose, goal: Sequence[float]) -> float:
    """Return squared planar distance without an unnecessary square root."""

    return float((pose.x - goal[0]) ** 2 + (pose.y - goal[1]) ** 2)
