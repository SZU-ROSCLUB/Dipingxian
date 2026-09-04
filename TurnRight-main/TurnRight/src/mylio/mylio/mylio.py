import tf2_ros
import math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from origincar_msg.msg import Sign
from geometry_msgs.msg import Pose2D, PoseStamped


class QrCodeDetection(Node):
    def __init__(self):
        super().__init__('OdometryPublisher')

        # 动态 TF：odom -> base
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        # 静态 TF：map -> odom
        self.static_tf_br = tf2_ros.StaticTransformBroadcaster(self)

        # 房间坐标系到地图坐标系的固定标定
        self.align_yaw = 0.03
        self.align_tx = -0.6
        self.align_ty = -0.23
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.current_pos = [0.0, 0.0, 0.0]      # x,y,yaw(rad)
        self.scope_angle = np.pi / 15
        self.exchange_benchmark_y = False
        self.exchange_benchmark_x = False
        self.cross_check_threshold = 0.3
        self.lpf_threshold = 0.3 # 常规滤波突变阈值（米）
        self.lpf_alpha = 0.2    # 常规低通滤波系数：20%新值 + 80%旧值
        self.prev_filtered = None

        self.pose2d_pub = self.create_publisher(Pose2D, '/odom_pose2d', 10)
        self.odom_stamped_pub = self.create_publisher(PoseStamped, 'odom_stamped', 10)

        self.is_blending = False
        self.switch_blend_start_time = None
        self.blend_duration = 1.5  # 混合持续时间（秒）
        self.smoothness = 0.2 # 混合系数：20%旧值 + 80%新值

        # 临时使用0.0；最终可改成雷达扫描平面相对base的真实高度
        self.laser_z = 0.0

        # 一次发布 map->odom 和 base->laser
        self._publish_static_transforms()

    def _estimate_yaw_from_walls(self, msg):
        """PCA墙面拟合估计yaw，返回[-π,π]，失败返回None"""
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        distances = np.array(msg.ranges, dtype=np.float32)

        valid = (
            (distances > msg.range_min) &
            (distances < msg.range_max) &
            (distances < 7.5) &
            ~np.isnan(distances) &
            ~np.isinf(distances)
        )

        px = distances * np.cos(angles)
        py = distances * np.sin(angles)

        SEG_GAP_SQ = 0.15 * 0.15
        MIN_PTS = 8
        segments = []
        seg_start = -1
        for i in range(n):
            if not valid[i]:
                if seg_start >= 0 and i - seg_start >= MIN_PTS:
                    segments.append((seg_start, i))
                seg_start = -1
                continue
            if seg_start < 0:
                seg_start = i
            elif valid[i-1]:
                dx = px[i] - px[i-1]
                dy = py[i] - py[i-1]
                if dx*dx + dy*dy > SEG_GAP_SQ:
                    if i - seg_start >= MIN_PTS:
                        segments.append((seg_start, i))
                    seg_start = i
        if seg_start >= 0 and n - seg_start >= MIN_PTS:
            segments.append((seg_start, n))

        if not segments:
            return None

        MIN_LINEARITY = 0.97
        MIN_LENGTH = 0.30
        orientations = []
        weights = []
        for s, e in segments:
            sx = px[s:e]; sy = py[s:e]
            dx_c = sx - sx.mean()
            dy_c = sy - sy.mean()
            sxx = float(np.sum(dx_c * dx_c))
            syy = float(np.sum(dy_c * dy_c))
            sxy = float(np.sum(dx_c * dy_c))

            trace = sxx + syy
            diff = np.sqrt(max(0.0, (sxx - syy)**2 + 4.0 * sxy * sxy))
            lam1 = (trace + diff) * 0.5
            lam2 = (trace - diff) * 0.5
            if lam1 <= 1e-9 or 1.0 - lam2 / lam1 < MIN_LINEARITY:
                continue

            theta = 0.5 * np.arctan2(2.0 * sxy, sxx - syy)
            proj = dx_c * np.cos(theta) + dy_c * np.sin(theta)
            seg_length = float(proj.max() - proj.min())
            if seg_length < MIN_LENGTH:
                continue

            orientations.append(theta)
            weights.append(seg_length)

        if not orientations:
            return None

        arr = np.array(orientations)
        w = np.array(weights)
        z = np.sum(w * np.exp(1j * 4.0 * arr))
        alpha_wall = np.angle(z) / 4.0

        yaw_mod = -alpha_wall
        while yaw_mod > np.pi / 4:
            yaw_mod -= np.pi / 2
        while yaw_mod <= -np.pi / 4:
            yaw_mod += np.pi / 2

        prev = self.current_pos[2]
        best, best_d = yaw_mod, np.pi
        for k in (-2, -1, 0, 1, 2):
            cand = (yaw_mod + k * np.pi / 2 + np.pi) % (2 * np.pi) - np.pi
            d = abs((cand - prev + np.pi) % (2 * np.pi) - np.pi)
            if d < best_d:
                best_d, best = d, cand
        return best

    def _compute_wall_distance(self, valid_angles, valid_distances, ref_angle, yaw):
        """计算车体到指定方向墙面距离，异常值过滤"""
        yaw_norm = yaw % (2.0 * np.pi)
        center = (ref_angle - yaw_norm) % (2.0 * np.pi)
        lower = (center - self.scope_angle) % (2.0 * np.pi)
        upper = (center + self.scope_angle) % (2.0 * np.pi)

        if lower <= upper:
            mask = (valid_angles >= lower) & (valid_angles <= upper)
        else:
            mask = (valid_angles >= lower) | (valid_angles <= upper)

        sel_angles = valid_angles[mask]
        sel_distances = valid_distances[mask]
        if len(sel_angles) == 0:
            return None, 0

        dist_mask = sel_distances <= 5.0
        sel_angles = sel_angles[dist_mask]
        sel_distances = sel_distances[dist_mask]
        if len(sel_angles) == 0:
            return None, 0

        projections = np.cos(np.abs(sel_angles - ref_angle + yaw)) * sel_distances
        mean_val = np.mean(projections)
        std_val = np.std(projections)
        inlier_mask = np.abs(projections - mean_val) <= 2.0 * std_val
        inliers = projections[inlier_mask]

        if len(inliers) > 0:
            return float(np.mean(inliers)), len(inliers)
        return None, 0

    def lidar_callback(self, msg):
        yaw_est = self._estimate_yaw_from_walls(msg)
        if yaw_est is not None:
            self.current_pos[2] = yaw_est

        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        distances = np.array(msg.ranges, dtype=np.float32)

        valid_mask = (
            (distances > msg.range_min) &
            (distances < msg.range_max) &
            ~np.isinf(distances) &
            ~np.isnan(distances)
        )
        valid_angles = angles[valid_mask]
        valid_distances = distances[valid_mask]
        if len(valid_angles) == 0:
            return

        yaw = self.current_pos[2]
        y_to_x5, n_y5 = self._compute_wall_distance(valid_angles, valid_distances, 1.5 * np.pi, yaw)
        y_to_x0, n_y0 = self._compute_wall_distance(valid_angles, valid_distances, 0.5 * np.pi, yaw)
        x_to_y0, n_x0 = self._compute_wall_distance(valid_angles, valid_distances, 1.0 * np.pi, yaw)
        x_to_y5, n_x5 = self._compute_wall_distance(valid_angles, valid_distances, 0.0 * np.pi, yaw)

        cross_y_ok = False
        if y_to_x5 is not None and y_to_x0 is not None:
            check_sum_y = y_to_x5 + y_to_x0
            cross_y_ok = abs(check_sum_y - 5.0) <= self.cross_check_threshold

        cross_x_ok = False
        if x_to_y0 is not None and x_to_y5 is not None:
            check_sum_x = x_to_y0 + x_to_y5
            cross_x_ok = abs(check_sum_x - 5.0) <= self.cross_check_threshold

        y_updated = False
        if self.exchange_benchmark_y:
            if y_to_x0 is not None:
                self.current_pos[1] = 5.0 - y_to_x0
                y_updated = True
            else:
                self.get_logger().warn('y_to_x0 unavailable, y not updated', throttle_duration_sec=2.0)
        else:
            if y_to_x5 is not None:
                self.current_pos[1] = y_to_x5
                y_updated = True
            else:
                self.get_logger().warn('y_to_x5 unavailable, y not updated', throttle_duration_sec=2.0)

        x_updated = False
        if self.exchange_benchmark_x:
            if x_to_y5 is not None:
                self.current_pos[0] = 5.0 - x_to_y5
                x_updated = True
            else:
                self.get_logger().warn('x_to_y5 unavailable, x not updated', throttle_duration_sec=2.0)
        else:
            if x_to_y0 is not None:
                self.current_pos[0] = x_to_y0
                x_updated = True
            else:
                self.get_logger().warn('x_to_y0 unavailable, x not updated', throttle_duration_sec=2.0)

        obs_x = self.current_pos[0]
        obs_y = self.current_pos[1]
        current_time = self.get_clock().now()

        if self.is_blending and self.switch_blend_start_time is not None:
            elapsed = (current_time - self.switch_blend_start_time).nanoseconds / 1e9
            if elapsed >= self.blend_duration:
                self.is_blending = False
                self.get_logger().info("混合期结束，恢复100%新观测坐标")

        final_x, final_y = obs_x, obs_y
        if self.prev_filtered is not None:
            prev_x, prev_y = self.prev_filtered
            if self.is_blending:
                final_x = self.smoothness * prev_x + (1 - self.smoothness) * obs_x
                final_y = self.smoothness * prev_y + (1 - self.smoothness) * obs_y
            else:
                if abs(obs_x - prev_x) > self.lpf_threshold:
                    final_x = self.lpf_alpha * obs_x + (1.0 - self.lpf_alpha) * prev_x
                if abs(obs_y - prev_y) > self.lpf_threshold:
                    final_y = self.lpf_alpha * obs_y + (1.0 - self.lpf_alpha) * prev_y

        self.current_pos[0] = final_x
        self.current_pos[1] = final_y
        self.prev_filtered = (final_x, final_y)

        pose_msg = Pose2D()
        pose_msg.x = final_x
        pose_msg.y = final_y
        pose_msg.theta = self.current_pos[2]
        self.pose2d_pub.publish(pose_msg)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = float(final_x)
        pose_stamped.pose.position.y = float(final_y)
        pose_stamped.pose.position.z = 0.0
        yaw = float(self.current_pos[2])
        half_yaw = 0.5 * yaw
        pose_stamped.pose.orientation.z = float(np.sin(half_yaw))
        pose_stamped.pose.orientation.w = float(np.cos(half_yaw))
        self.odom_stamped_pub.publish(pose_stamped)

        # 激光位姿对应本次 LaserScan 的采样时间
        scan_stamp = msg.header.stamp
        x, y, yaw = final_x, final_y, float(self.current_pos[2])

        t_base = TransformStamped()
        t_base.header.stamp = scan_stamp
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base'
        t_base.transform.translation.x = float(x)
        t_base.transform.translation.y = float(y)
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.z = math.sin(yaw / 2.0)
        t_base.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_br.sendTransform(t_base)

        odom_msg = Odometry()
        odom_msg.header.stamp = scan_stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom_msg)

        if x_updated or y_updated:
            self.get_logger().info(f'X={self.current_pos[0]:.3f}m Y={self.current_pos[1]:.3f}m Yaw={self.current_pos[2]:.3f}rad')

        prev_bm_x = self.exchange_benchmark_x
        prev_bm_y = self.exchange_benchmark_y

        if not self.exchange_benchmark_x and final_x > 2.4:
            self.exchange_benchmark_x = True
        elif self.exchange_benchmark_x and final_x < 2.6:
            self.exchange_benchmark_x = False

        if not self.exchange_benchmark_y and final_y > 2.5:
            self.exchange_benchmark_y = True
        elif self.exchange_benchmark_y and final_y < 2.5:
            self.exchange_benchmark_y = False

        if prev_bm_x != self.exchange_benchmark_x or prev_bm_y != self.exchange_benchmark_y:
            self.switch_blend_start_time = self.get_clock().now()
            self.is_blending = True
            self.get_logger().warn("检测到墙面切换，启动混合滤波...")

    def _publish_static_transforms(self):
        stamp = self.get_clock().now().to_msg()

        # map -> odom
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'

        t_map_odom.transform.translation.x = self.align_tx
        t_map_odom.transform.translation.y = self.align_ty
        t_map_odom.transform.translation.z = 0.0

        t_map_odom.transform.rotation.x = 0.0
        t_map_odom.transform.rotation.y = 0.0
        t_map_odom.transform.rotation.z = math.sin(self.align_yaw / 2.0)
        t_map_odom.transform.rotation.w = math.cos(self.align_yaw / 2.0)

        # base -> laser
        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = stamp
        t_base_laser.header.frame_id = 'base'
        t_base_laser.child_frame_id = 'laser'

        t_base_laser.transform.translation.x = 0.0
        t_base_laser.transform.translation.y = 0.0
        t_base_laser.transform.translation.z = self.laser_z

        t_base_laser.transform.rotation.x = 0.0
        t_base_laser.transform.rotation.y = 0.0
        t_base_laser.transform.rotation.z = 0.0
        t_base_laser.transform.rotation.w = 1.0

        self.static_tf_br.sendTransform([
            t_map_odom,
            t_base_laser,
        ])

def main(args=None):
    rclpy.init(args=args)
    qrCodeDetection = QrCodeDetection()
    rclpy.spin(qrCodeDetection)
    qrCodeDetection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
