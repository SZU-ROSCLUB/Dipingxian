"""ROS-independent mission state machine and vehicle-control logic."""

from typing import Optional, Sequence, Tuple

import numpy as np

from run_car_pkg.PID.pid import PID
from run_car_pkg.config import ControllerConfig
from run_car_pkg.geometry import (
    angle_to_image_x,
    goal_bearing,
    normalize_angle,
    squared_distance,
)
from run_car_pkg.lidar_avoidance import (
    choose_avoidance_bearing,
    choose_escape_bearing,
    has_rear_obstacle,
)
from run_car_pkg.models import (
    ControlDecision,
    MissionState,
    RouteDirection,
    SensorSnapshot,
)
from run_car_pkg.path_tracking import (
    RoutePath,
    build_route,
    find_lookahead_index,
    project_forward,
)
from run_car_pkg.routes import (
    CLOCKWISE_WAYPOINTS,
    COUNTERCLOCKWISE_WAYPOINTS,
    STOP_GOAL,
    TASK1_GOAL,
    TASK3_GOAL,
)


QR_ACCEPTANCE_X = 2.5
TASK1_SLOWDOWN_X = 2.7
TASK1_GOAL_DISTANCE_SQUARED = 0.30
TASK3_STOP_DISTANCE_SQUARED = 0.25
CIRCLE_CAPTURE_WAYPOINT = 7
CIRCLE_LOOKAHEAD_BOOST_WAYPOINT = 6
CIRCLE_LOOKAHEAD_BOOST = 1.25
AVOIDANCE_BEARING_GAIN = 1.3
AVOIDANCE_ANGLE_THRESHOLD = 0.05


class MissionController:
    """Own mission state and compute control decisions from sensor snapshots."""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.state = MissionState.WAITING
        self.direction = RouteDirection.UNKNOWN

        self.path_index = 0
        self.waypoint_index = 0
        self.recovering = False
        self.reverse_start_time: Optional[float] = None
        self.bypass_obstacle_avoidance = False

        self.clockwise_route = build_route(CLOCKWISE_WAYPOINTS, spacing=0.02)
        self.counterclockwise_route = build_route(
            COUNTERCLOCKWISE_WAYPOINTS,
            spacing=0.02,
        )

        self.pid = PID(config.pid.sample_time)
        self.pid.set_kp(config.pid.kp)
        self.pid.set_ki(config.pid.ki)
        self.pid.set_kd(config.pid.kd)
        self.pid.set_output_limit(config.pid.output_limit)
        self.pid.set_integral_limit(config.pid.integral_limit)

        self._pending_logs = [
            '[中线] cw={} ccw={}'.format(
                len(self.clockwise_route.points),
                len(self.counterclockwise_route.points),
            )
        ]

    def step(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        """Run one control iteration for the active mission state."""

        if (
            self.state == MissionState.RETURNING_HOME
            and snapshot.end_detections
            and snapshot.end_detections[0].bottom_y
            > self.config.mission.stop_end_threshold
        ):
            self.transition_to(
                MissionState.STOPPED,
                '终点视觉目标达到停车阈值',
            )

        handlers = {
            MissionState.WAITING: self._run_waiting,
            MissionState.APPROACH_QR: self._run_task1,
            MissionState.REVERSING: self._run_reverse,
            MissionState.CIRCLING: self._run_circle,
            MissionState.RETURNING_HOME: self._run_return,
            MissionState.STOPPED: self._run_stopped,
        }
        return handlers[self.state](snapshot, now_seconds)

    def handle_qr_result(self, text: str, pose_x: float) -> bool:
        """Parse a QR result and select the circle direction when appropriate."""

        if (
            self.state
            not in (MissionState.APPROACH_QR, MissionState.REVERSING)
            or pose_x < QR_ACCEPTANCE_X
        ):
            return False

        digits = ''.join(character for character in text if character.isdigit())
        if not digits:
            self._pending_logs.append('二维码内容不包含数字，已忽略')
            return False

        number = int(digits)
        self.direction = (
            RouteDirection.CLOCKWISE
            if number % 2
            else RouteDirection.COUNTERCLOCKWISE
        )
        if self.state == MissionState.APPROACH_QR:
            self.transition_to(MissionState.REVERSING, '二维码解析完成')
        self._pending_logs.append(
            '二维码数字={}，路线={}'.format(number, self.direction.name)
        )
        return True

    def handle_manual_command(self, command: int) -> bool:
        """Apply the existing manual task-switch command mapping."""

        state_by_command = {
            1: MissionState.APPROACH_QR,
            2: MissionState.WAITING,
            3: MissionState.RETURNING_HOME,
            4: MissionState.REVERSING,
            5: MissionState.CIRCLING,
        }
        new_state = state_by_command.get(int(command))
        if new_state is None:
            self._pending_logs.append('未知手动任务指令: {}'.format(command))
            return False

        if (
            new_state == MissionState.CIRCLING
            and self.direction == RouteDirection.UNKNOWN
        ):
            self._set_fallback_direction()
        self.transition_to(new_state, '手动任务切换 {}'.format(command))
        return True

    def transition_to(self, new_state: MissionState, reason: str) -> None:
        """Switch mission state and perform all state-entry resets in one place."""

        new_state = MissionState(new_state)
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        if new_state == MissionState.REVERSING:
            self.reverse_start_time = None
        elif new_state == MissionState.CIRCLING:
            self.reverse_start_time = None
            self.path_index = 0
            self.waypoint_index = 0
            self.recovering = False
        elif new_state == MissionState.RETURNING_HOME:
            self.pid.reset()
            self.path_index = 0
            self.waypoint_index = 0
            self.recovering = False
            self.bypass_obstacle_avoidance = False
        elif new_state == MissionState.APPROACH_QR:
            self.pid.reset()

        self._pending_logs.append(
            '{} -> {}: {}'.format(old_state.name, new_state.name, reason)
        )

    def _run_waiting(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        del snapshot, now_seconds
        return self._decision(0.0, 0.0)

    def _run_task1(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        del now_seconds
        pose = snapshot.pose
        nav_goal: Sequence[float]
        if pose.x < self.config.mission.task1_cruise_x:
            nav_goal = (TASK1_GOAL[0], self.config.mission.task1_cruise_y)
        else:
            nav_goal = TASK1_GOAL

        visual_target = (
            snapshot.qr_detections[0].center_x
            if snapshot.qr_detections
            else None
        )
        speed, steering, beta_goal, beta_command = self._navigate_to_goal(
            snapshot,
            nav_goal,
            visual_target,
        )

        if pose.x > TASK1_SLOWDOWN_X:
            speed = self.config.speed.minimum
        if squared_distance(pose, TASK1_GOAL) < TASK1_GOAL_DISTANCE_SQUARED:
            self.transition_to(MissionState.REVERSING, '到达 Task1 目标区域')

        debug_message = (
            '[Task1] aim={} pos=({:.2f},{:.2f}) '
            'beta_goal={:.0f}deg beta_cmd={:.0f}deg'
        ).format(
            tuple(nav_goal),
            pose.x,
            pose.y,
            np.degrees(beta_goal),
            np.degrees(beta_command),
        )
        return self._decision(speed, steering, debug=debug_message)

    def _run_reverse(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        if self.reverse_start_time is None:
            self.reverse_start_time = now_seconds
            self._pending_logs.append('开始右打死倒车，调整偏航角')

        elapsed = now_seconds - self.reverse_start_time
        pose_ok = snapshot.pose.yaw > self.config.reverse.stop_yaw
        rear_blocked = has_rear_obstacle(
            snapshot.lidar_betas,
            snapshot.lidar_ranges,
            self.config.lidar,
            self.config.reverse,
        )
        timed_out = elapsed > self.config.reverse.maximum_time

        if pose_ok or rear_blocked or timed_out:
            if self.direction == RouteDirection.UNKNOWN:
                self._set_fallback_direction()
            if pose_ok:
                reason = '倒车偏航角到位'
            elif rear_blocked:
                reason = '后方检测到障碍物'
            else:
                reason = '倒车超时保护'
            self.transition_to(MissionState.CIRCLING, reason)
            return self._decision(0.0, 0.0)

        return self._decision(
            self.config.reverse.speed,
            self.config.reverse.angular_speed,
        )

    def _run_circle(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        del now_seconds
        route = self._selected_route()
        if route is None:
            self.transition_to(MissionState.STOPPED, '绕圈方向无效')
            return self._decision(0.0, 0.0)
        if len(route.points) < 2:
            self.transition_to(MissionState.STOPPED, '绕圈路径点不足')
            return self._decision(0.0, 0.0)

        projection = project_forward(
            route.points,
            snapshot.pose.x,
            snapshot.pose.y,
            self.path_index,
            self.config.circle.projection_window,
        )
        self.path_index = projection.nearest_index
        cross_track_error = projection.cross_track_error

        if self.path_index >= len(route.points) - 3:
            self.transition_to(MissionState.RETURNING_HOME, '绕圈路径完成')
            return self._decision(0.0, 0.0)

        capture_requested = self._advance_waypoint_events(
            route,
            snapshot.pose.x,
            snapshot.pose.yaw,
        )
        if self.state == MissionState.WAITING:
            return self._decision(0.0, 0.0, capture_requested)

        if self.recovering and (
            cross_track_error < self.config.circle.recovered_cte
        ):
            self.recovering = False
        was_recovering = self.recovering

        lookahead = self.config.circle.lookahead
        if self.waypoint_index == CIRCLE_LOOKAHEAD_BOOST_WAYPOINT:
            lookahead *= CIRCLE_LOOKAHEAD_BOOST

        lookahead_index = find_lookahead_index(
            route.arc_lengths,
            self.path_index,
            lookahead,
        )
        path_bearing = goal_bearing(
            snapshot.pose,
            route.points[lookahead_index],
        )

        avoidance = choose_avoidance_bearing(
            path_bearing,
            snapshot.lidar_betas,
            snapshot.lidar_ranges,
            self.config.lidar,
            bypass=self.bypass_obstacle_avoidance,
        )
        if avoidance.bearing is None:
            command_bearing = choose_escape_bearing(
                snapshot.lidar_betas,
                snapshot.lidar_ranges,
                self.config.lidar,
            )
            avoiding = True
        else:
            command_bearing = avoidance.bearing
            angle_difference = normalize_angle(command_bearing - path_bearing)
            avoiding = abs(angle_difference) > AVOIDANCE_ANGLE_THRESHOLD

        if avoiding:
            command_bearing *= AVOIDANCE_BEARING_GAIN
            self.recovering = True

        target_x = angle_to_image_x(
            -command_bearing,
            self.config.camera_matrix,
        )
        steering = self._pid_output(target_x)

        if self.waypoint_index == 0:
            speed = self.config.speed.medium
        elif (
            avoiding
            or was_recovering
            or cross_track_error > self.config.circle.slow_cte_threshold
            or abs(steering) > self.config.circle.slow_steering_threshold
        ):
            speed = self.config.speed.minimum
        else:
            speed = self.config.speed.maximum
        speed = max(speed, self.config.speed.minimum)
        steering *= self.config.speed.steering_scale

        debug_message = (
            'idx={}/{} point={} cte={:.2f} beta={:.0f}deg '
            'v={:.2f} out={:.2f} pose=({:.2f},{:.2f}){}'
        ).format(
            self.path_index,
            len(route.points),
            self.waypoint_index + 1,
            cross_track_error,
            np.degrees(command_bearing),
            speed,
            steering,
            snapshot.pose.x,
            snapshot.pose.y,
            ' [避障]' if avoiding else (' [回中]' if was_recovering else ''),
        )
        return self._decision(
            speed,
            steering,
            capture_requested,
            debug_message,
        )

    def _run_return(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        del now_seconds
        visual_target = (
            snapshot.end_detections[0].center_x
            if snapshot.end_detections
            else None
        )
        speed, steering, _, _ = self._navigate_to_goal(
            snapshot,
            TASK3_GOAL,
            visual_target,
        )

        if (
            squared_distance(snapshot.pose, STOP_GOAL)
            < TASK3_STOP_DISTANCE_SQUARED
        ):
            self.transition_to(MissionState.STOPPED, '到达最终停车区域')

        debug_message = 'Task3 goal={} pose=({:.2f},{:.2f},{:.2f}) out={:.2f}'.format(
            tuple(TASK3_GOAL),
            snapshot.pose.x,
            snapshot.pose.y,
            snapshot.pose.yaw,
            steering,
        )
        return self._decision(speed, steering, debug=debug_message)

    def _run_stopped(
        self,
        snapshot: SensorSnapshot,
        now_seconds: float,
    ) -> ControlDecision:
        del snapshot, now_seconds
        return self._decision(0.0, 0.0)

    def _navigate_to_goal(
        self,
        snapshot: SensorSnapshot,
        goal: Sequence[float],
        visual_target_x: Optional[float],
    ) -> Tuple[float, float, float, float]:
        target_bearing = goal_bearing(snapshot.pose, goal)
        avoidance = choose_avoidance_bearing(
            target_bearing,
            snapshot.lidar_betas,
            snapshot.lidar_ranges,
            self.config.lidar,
        )

        if avoidance.bearing is None:
            speed = self.config.speed.medium
            command_bearing = choose_escape_bearing(
                snapshot.lidar_betas,
                snapshot.lidar_ranges,
                self.config.lidar,
            )
            target_x = angle_to_image_x(
                -command_bearing,
                self.config.camera_matrix,
            )
        elif avoidance.obstacle_detected:
            speed = self.config.speed.minimum
            command_bearing = avoidance.bearing
            target_x = angle_to_image_x(
                -command_bearing,
                self.config.camera_matrix,
            )
        else:
            speed = self.config.speed.maximum
            command_bearing = avoidance.bearing
            target_x = (
                float(visual_target_x)
                if visual_target_x is not None
                else angle_to_image_x(
                    -command_bearing,
                    self.config.camera_matrix,
                )
            )

        steering = self._pid_output(target_x)
        return speed, steering, target_bearing, command_bearing

    def _advance_waypoint_events(
        self,
        route: RoutePath,
        pose_x: float,
        pose_yaw: float,
    ) -> bool:
        capture_requested = False
        while (
            self.waypoint_index + 1 < len(route.waypoint_indices)
            and self.path_index
            >= route.waypoint_indices[self.waypoint_index + 1]
        ):
            self.waypoint_index += 1
            self._pending_logs.append(
                '第{}个点完成'.format(self.waypoint_index)
            )

            if self.waypoint_index == self.config.mission.quit_waypoint:
                self.transition_to(MissionState.WAITING, '到达配置的退出路点')

            should_capture_clockwise = (
                self.waypoint_index == CIRCLE_CAPTURE_WAYPOINT
                and pose_x > 2.6
                and self.direction == RouteDirection.CLOCKWISE
            )
            should_capture_counterclockwise = (
                self.waypoint_index == CIRCLE_CAPTURE_WAYPOINT
                and pose_x < 2.4
                and self.direction == RouteDirection.COUNTERCLOCKWISE
            )
            if should_capture_clockwise or should_capture_counterclockwise:
                capture_requested = True
                self.bypass_obstacle_avoidance = True
                self._pending_logs.append('[图生文] 到达点8，已请求拍照')

            if (
                self.direction == RouteDirection.CLOCKWISE
                and pose_x > 3.9
                and -1.0 < pose_yaw < -0.14
            ):
                self.bypass_obstacle_avoidance = False
            elif (
                self.direction == RouteDirection.COUNTERCLOCKWISE
                and pose_x < 1.4
                and -3.14 < pose_yaw < -3.0
            ):
                self.bypass_obstacle_avoidance = False

        return capture_requested

    def _selected_route(self) -> Optional[RoutePath]:
        if self.direction == RouteDirection.CLOCKWISE:
            return self.clockwise_route
        if self.direction == RouteDirection.COUNTERCLOCKWISE:
            return self.counterclockwise_route
        return None

    def _set_fallback_direction(self) -> None:
        self.direction = RouteDirection(
            self.config.mission.fallback_direction
        )
        self._pending_logs.append(
            '未解析二维码，使用默认路线 {}'.format(self.direction.name)
        )

    def _pid_output(self, target_x: float) -> float:
        principal_x = int(self.config.camera_matrix[0, 2])
        return self.pid.update(1.0, target_x / principal_x)

    def _decision(
        self,
        linear_x: float,
        angular_z: float,
        capture_requested: bool = False,
        debug: Optional[str] = None,
    ) -> ControlDecision:
        messages = tuple(self._pending_logs)
        self._pending_logs.clear()
        return ControlDecision(
            linear_x=float(linear_x),
            angular_z=float(angular_z),
            capture_requested=capture_requested,
            log_messages=messages,
            debug_message=debug,
        )
