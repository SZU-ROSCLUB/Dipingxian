"""Unit tests for mission-state entry and QR direction selection."""

import numpy as np

from run_car_pkg.config import (
    CircleConfig,
    ControllerConfig,
    LidarConfig,
    MissionConfig,
    PidConfig,
    ReverseConfig,
    SpeedConfig,
)
from run_car_pkg.mission_controller import MissionController
from run_car_pkg.models import MissionState, RouteDirection


def _controller_config():
    return ControllerConfig(
        camera_matrix=np.array(
            [
                [677.0, 0.0, 906.0],
                [0.0, 676.0, 235.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        speed=SpeedConfig(1.3, 0.85, 0.5, 1.0),
        pid=PidConfig(0.02, 3.8, 0.0, 0.01),
        lidar=LidarConfig(True, 1.4, 0.9, 0.1, 0.24, 0.0, 0.1, 1.0),
        reverse=ReverseConfig(-0.8, 4.0, 1.3, 1.8, 0.38, 0.5),
        circle=CircleConfig(0.48, 100, 1.0, 0.4, 0.25),
        mission=MissionConfig(1050, 1, 99, 2.4, 1.5),
    )


def test_qr_result_selects_direction_and_reverse_state():
    controller = MissionController(_controller_config())
    controller.handle_manual_command(1)

    assert controller.handle_qr_result('number 7', pose_x=3.0)
    assert controller.direction == RouteDirection.CLOCKWISE
    assert controller.state == MissionState.REVERSING


def test_invalid_qr_text_does_not_change_state():
    controller = MissionController(_controller_config())
    controller.handle_manual_command(1)

    assert not controller.handle_qr_result('no digits', pose_x=3.0)
    assert controller.state == MissionState.APPROACH_QR


def test_qr_result_can_arrive_after_reverse_has_started():
    controller = MissionController(_controller_config())
    controller.handle_manual_command(4)

    assert controller.handle_qr_result('number 8', pose_x=3.0)
    assert controller.direction == RouteDirection.COUNTERCLOCKWISE
    assert controller.state == MissionState.REVERSING
