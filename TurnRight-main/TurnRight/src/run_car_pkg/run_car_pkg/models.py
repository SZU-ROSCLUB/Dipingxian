"""Internal data models used by the vehicle controller.

This module intentionally has no ROS dependencies.  The ROS node converts
messages into these data classes before invoking the mission controller.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np


class MissionState(IntEnum):
    """High-level mission states.

    The numeric values are kept compatible with the existing manual task
    switch commands.
    """

    WAITING = 1
    APPROACH_QR = 2
    REVERSING = 3
    CIRCLING = 4
    RETURNING_HOME = 5
    STOPPED = 6


class RouteDirection(IntEnum):
    """Direction selected from the QR-code number."""

    UNKNOWN = -1
    COUNTERCLOCKWISE = 0
    CLOCKWISE = 1


@dataclass(frozen=True)
class VehiclePose:
    """Vehicle pose in the odometry frame."""

    x: float
    y: float
    yaw: float
    stamp_ns: int = 0


@dataclass(frozen=True)
class Detection:
    """Small, ROS-independent representation of one visual detection."""

    center_x: float
    bottom_y: float
    confidence: float


@dataclass(frozen=True)
class SensorSnapshot:
    """Sensor values consumed together by one control iteration."""

    pose: VehiclePose
    lidar_betas: Optional[np.ndarray]
    lidar_ranges: Optional[np.ndarray]
    qr_detections: Tuple[Detection, ...] = ()
    end_detections: Tuple[Detection, ...] = ()


@dataclass(frozen=True)
class ControlDecision:
    """ROS-independent result of one control iteration."""

    linear_x: float
    angular_z: float
    capture_requested: bool = False
    log_messages: Tuple[str, ...] = ()
    debug_message: Optional[str] = None
