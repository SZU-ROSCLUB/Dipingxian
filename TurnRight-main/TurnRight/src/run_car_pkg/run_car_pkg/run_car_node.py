"""ROS 2 adapter for the vehicle mission controller."""

import threading

import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String

from origincar_msg.msg import Sign
from run_car_pkg.config import declare_and_load_config
from run_car_pkg.lidar_avoidance import scan_bearings
from run_car_pkg.mission_controller import MissionController
from run_car_pkg.models import Detection, SensorSnapshot, VehiclePose


class RunCarNode(Node):
    """Convert ROS messages to controller inputs and publish its decisions."""

    def __init__(self) -> None:
        super().__init__('run_car_node')

        self.config = declare_and_load_config(self)
        self.controller = MissionController(self.config)

        self._sensor_lock = threading.Lock()
        self._controller_lock = threading.Lock()
        self._latest_pose = VehiclePose(0.5, 0.2, 0.0, 0)
        self._latest_pose_stamp = 0
        self._lidar_betas = None
        self._lidar_ranges = None
        self._qr_detections = []
        self._end_detections = []

        self._cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._sign_publisher = self.create_publisher(Sign, 'sign_switch', 10)
        self._capture_publisher = self.create_publisher(
            Int32,
            '/capture_trigger',
            10,
        )

        self._dnn_subscription = self.create_subscription(
            PerceptionTargets,
            'hobot_dnn_detection',
            self._dnn_callback,
            10,
        )
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._lidar_callback,
            10,
        )
        self._odom_subscription = self.create_subscription(
            PoseStamped,
            'odom_stamped',
            self._odom_callback,
            10,
        )
        self._qr_subscription = self.create_subscription(
            String,
            '/qrcode_information',
            self._qr_result_callback,
            10,
        )
        self._manual_subscription = self.create_subscription(
            Int32,
            'sign4return',
            self._manual_task_callback,
            10,
        )
        self._control_timer = self.create_timer(
            self.config.pid.sample_time,
            self._control_callback,
        )
        self.get_logger().info('RunCarNode 初始化完成')

    def _control_callback(self) -> None:
        """Build one sensor snapshot and publish one controller decision."""

        with self._sensor_lock:
            snapshot = SensorSnapshot(
                pose=self._latest_pose,
                lidar_betas=self._lidar_betas,
                lidar_ranges=self._lidar_ranges,
                qr_detections=tuple(self._qr_detections),
                end_detections=tuple(self._end_detections),
            )
            self._qr_detections.clear()
            self._end_detections.clear()

        now_seconds = self.get_clock().now().nanoseconds / 1e9
        with self._controller_lock:
            decision = self.controller.step(snapshot, now_seconds)

        self._publish_velocity(decision.linear_x, decision.angular_z)
        if decision.capture_requested:
            capture_message = Int32()
            capture_message.data = 1
            self._capture_publisher.publish(capture_message)

        for message in decision.log_messages:
            self.get_logger().info(message)
        if decision.debug_message:
            self.get_logger().debug(decision.debug_message)

    def _dnn_callback(self, message: PerceptionTargets) -> None:
        """Convert DNN regions into small internal detection records."""

        qr_detections = []
        end_detections = []
        qr_detected = False

        for target in message.targets:
            if not target.rois:
                continue
            roi = target.rois[0]
            rectangle = roi.rect
            detection = Detection(
                center_x=float(rectangle.x_offset + rectangle.width / 2.0),
                bottom_y=float(rectangle.y_offset + rectangle.height),
                confidence=float(roi.confidence),
            )
            if roi.type == 'QR_code':
                qr_detected = True
                qr_detections.append(detection)
            elif roi.type == 'end':
                end_detections.append(detection)

        qr_detections.sort(key=lambda item: item.bottom_y, reverse=True)
        end_detections.sort(key=lambda item: item.bottom_y, reverse=True)
        with self._sensor_lock:
            self._qr_detections.extend(qr_detections)
            self._qr_detections.sort(
                key=lambda item: item.bottom_y,
                reverse=True,
            )
            self._end_detections.extend(end_detections)
            self._end_detections.sort(
                key=lambda item: item.bottom_y,
                reverse=True,
            )

        if qr_detected:
            self._publish_sign(0)

    def _qr_result_callback(self, message: String) -> None:
        """Forward a decoded QR string to the mission state machine."""

        with self._sensor_lock:
            pose_x = self._latest_pose.x
        with self._controller_lock:
            self.controller.handle_qr_result(message.data, pose_x)

    def _manual_task_callback(self, message: Int32) -> None:
        """Apply a manual task command while retaining the legacy mapping."""

        with self._controller_lock:
            accepted = self.controller.handle_manual_command(message.data)
        if not accepted:
            self.get_logger().warning(
                '忽略未知手动任务指令: {}'.format(message.data)
            )
            return

        self._publish_sign(message.data)
        if message.data == 2:
            self._publish_velocity(0.0, 0.0)

    def _odom_callback(self, message: PoseStamped) -> None:
        """Store the newest stamped pose and reject out-of-order messages."""

        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        orientation = message.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        pose = VehiclePose(
            x=float(message.pose.position.x),
            y=float(message.pose.position.y),
            yaw=float(np.arctan2(sin_yaw, cos_yaw)),
            stamp_ns=stamp_ns,
        )

        with self._sensor_lock:
            if stamp_ns == 0 or stamp_ns > self._latest_pose_stamp:
                self._latest_pose = pose
                if stamp_ns != 0:
                    self._latest_pose_stamp = stamp_ns

    def _lidar_callback(self, message: LaserScan) -> None:
        """Convert a LaserScan into normalized bearings and range arrays."""

        betas = scan_bearings(
            message.angle_min,
            message.angle_increment,
            len(message.ranges),
            self.config.lidar.invert,
        )
        ranges = np.asarray(message.ranges, dtype=np.float32)
        with self._sensor_lock:
            self._lidar_betas = betas
            self._lidar_ranges = ranges

    def _publish_velocity(self, linear_x: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self._cmd_publisher.publish(message)

    def _publish_sign(self, value: int) -> None:
        message = Sign()
        message.sign_data = int(value)
        self._sign_publisher.publish(message)


def main(args=None) -> None:
    """Start the ROS node."""

    rclpy.init(args=args)
    node = RunCarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
