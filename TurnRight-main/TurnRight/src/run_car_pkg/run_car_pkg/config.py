"""Typed controller configuration loaded from ROS parameters."""

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SpeedConfig:
    """Forward-speed and steering-output settings."""

    maximum: float
    medium: float
    minimum: float
    steering_scale: float


@dataclass(frozen=True)
class PidConfig:
    """PID settings for steering control."""

    sample_time: float
    kp: float
    ki: float
    kd: float
    output_limit: float = 4.0
    integral_limit: float = 0.5


@dataclass(frozen=True)
class LidarConfig:
    """Lidar obstacle-avoidance settings."""

    enabled: bool
    fov: float
    influence_distance: float
    minimum_range: float
    inflation_radius: float
    minimum_gap: float
    edge_margin: float
    invert: float


@dataclass(frozen=True)
class ReverseConfig:
    """Reverse manoeuvre and rear-collision settings."""

    speed: float
    angular_speed: float
    maximum_time: float
    stop_yaw: float
    obstacle_distance: float
    rear_fov: float


@dataclass(frozen=True)
class CircleConfig:
    """Closed-course path-following settings."""

    lookahead: float
    projection_window: int
    slow_steering_threshold: float
    slow_cte_threshold: float
    recovered_cte: float


@dataclass(frozen=True)
class MissionConfig:
    """Task-specific mission settings."""

    stop_end_threshold: int
    fallback_direction: int
    quit_waypoint: int
    task1_cruise_x: float
    task1_cruise_y: float


@dataclass(frozen=True)
class ControllerConfig:
    """Complete immutable configuration used by the controller."""

    camera_matrix: np.ndarray
    speed: SpeedConfig
    pid: PidConfig
    lidar: LidarConfig
    reverse: ReverseConfig
    circle: CircleConfig
    mission: MissionConfig


PARAMETER_DEFAULTS = (
    ('stop_end_threshold', 460),
    ('max_v', 1.5),
    ('mid_v', 1.5),
    ('min_v', 1.5),
    ('ctrl_ts', 0.02),
    ('Kp', 1.5),
    ('Ki', 0.0),
    ('Kd', 0.0),
    ('speed_back', -1.0),
    ('random_flag', 0),
    ('Coefficient_output', 1.0),
    ('quit_point', 1),
    ('lidar_fov', 1.4),
    ('lidar_influence', 1.2),
    ('lidar_range_min', 0.15),
    ('lidar_safe_clearance', 0.15),
    ('car_half_width', 0.10),
    ('obstacle_radius', 0.05),
    ('lidar_min_gap', 0.10),
    ('lidar_edge_margin', 0.10),
    ('lidar_invert', 1.0),
    ('back_omega', 4.0),
    ('back_max_time', 2.0),
    ('back_stop_yaw', 1.57),
    ('back_obst_dist', 0.35),
    ('back_fov', 0.6),
    ('task1_cruise_px', 2.5),
    ('task1_cruise_py', 0.9),
    ('circle_lookahead', 0.30),
    ('circle_proj_fwd', 100),
    ('circle_slow_thresh', 2.0),
    ('circle_cte_slow', 0.20),
    ('recover_cte_done', 0.12),
    ('open_lidar_bearing', 1),
)


def declare_and_load_config(node: Any) -> ControllerConfig:
    """Declare ROS parameters, read their values and validate the result."""

    node.declare_parameter('camera_matrix', [0.0] * 9)
    node.declare_parameters(namespace='', parameters=PARAMETER_DEFAULTS)

    matrix_values = node.get_parameter('camera_matrix').value
    camera_matrix = np.asarray(matrix_values, dtype=float)
    if camera_matrix.size != 9:
        raise ValueError('camera_matrix must contain exactly 9 numbers')
    camera_matrix = camera_matrix.reshape(3, 3)

    def value(name: str) -> Any:
        return node.get_parameter(name).value

    car_half_width = float(value('car_half_width'))
    obstacle_radius = float(value('obstacle_radius'))
    safe_clearance = float(value('lidar_safe_clearance'))

    config = ControllerConfig(
        camera_matrix=camera_matrix,
        speed=SpeedConfig(
            maximum=float(value('max_v')),
            medium=float(value('mid_v')),
            minimum=float(value('min_v')),
            steering_scale=float(value('Coefficient_output')),
        ),
        pid=PidConfig(
            sample_time=float(value('ctrl_ts')),
            kp=float(value('Kp')),
            ki=float(value('Ki')),
            kd=float(value('Kd')),
        ),
        lidar=LidarConfig(
            enabled=bool(int(value('open_lidar_bearing'))),
            fov=float(value('lidar_fov')),
            influence_distance=float(value('lidar_influence')),
            minimum_range=float(value('lidar_range_min')),
            inflation_radius=(
                car_half_width + obstacle_radius + safe_clearance
            ),
            minimum_gap=float(value('lidar_min_gap')),
            edge_margin=float(value('lidar_edge_margin')),
            invert=float(value('lidar_invert')),
        ),
        reverse=ReverseConfig(
            speed=float(value('speed_back')),
            angular_speed=float(value('back_omega')),
            maximum_time=float(value('back_max_time')),
            stop_yaw=float(value('back_stop_yaw')),
            obstacle_distance=float(value('back_obst_dist')),
            rear_fov=float(value('back_fov')),
        ),
        circle=CircleConfig(
            lookahead=float(value('circle_lookahead')),
            projection_window=int(value('circle_proj_fwd')),
            slow_steering_threshold=float(value('circle_slow_thresh')),
            slow_cte_threshold=float(value('circle_cte_slow')),
            recovered_cte=float(value('recover_cte_done')),
        ),
        mission=MissionConfig(
            stop_end_threshold=int(value('stop_end_threshold')),
            fallback_direction=int(value('random_flag')),
            quit_waypoint=int(value('quit_point')),
            task1_cruise_x=float(value('task1_cruise_px')),
            task1_cruise_y=float(value('task1_cruise_py')),
        ),
    )
    validate_config(config)
    return config


def validate_config(config: ControllerConfig) -> None:
    """Fail fast when parameters could produce unsafe or invalid commands."""

    if config.pid.sample_time <= 0.0:
        raise ValueError('ctrl_ts must be greater than zero')
    if not (
        0.0 < config.speed.minimum
        <= config.speed.medium
        <= config.speed.maximum
    ):
        raise ValueError('expected 0 < min_v <= mid_v <= max_v')
    if abs(float(config.camera_matrix[0, 2])) < 1e-9:
        raise ValueError('camera_matrix cx must not be zero')
    if config.lidar.minimum_range < 0.0:
        raise ValueError('lidar_range_min must not be negative')
    if config.lidar.influence_distance <= config.lidar.minimum_range:
        raise ValueError('lidar_influence must exceed lidar_range_min')
    if config.lidar.fov <= 0.0:
        raise ValueError('lidar_fov must be greater than zero')
    if config.reverse.maximum_time <= 0.0:
        raise ValueError('back_max_time must be greater than zero')
    if config.circle.projection_window <= 0:
        raise ValueError('circle_proj_fwd must be greater than zero')
    if config.mission.fallback_direction not in (0, 1):
        raise ValueError('random_flag must be 0 or 1')
