# `run_car_node` 主控算法说明

本文说明 `run_car_pkg` 的主控算法、状态转换、路径跟踪、雷达避障、视觉融合、PID 转向和速度决策。
## 1. 算法目标

主控节点需要完成以下比赛流程：

1. 等待启动。
2. 驶向二维码区域。
3. 根据二维码数字奇偶确定绕圈方向。
4. 倒车调整车身姿态。
5. 按顺时针或逆时针路线绕行，同时进行雷达避障。
6. 到达指定路点时触发拍照。
7. 绕圈结束后返回终点。
8. 根据位置和视觉终点信息停车。

算法采用分层结构：

```text
ROS 话题与消息转换
        ↓
SensorSnapshot
        ↓
任务状态机 MissionController
        ├── 目标点导航
        ├── 路径生成与前视跟踪
        ├── 雷达空隙避障
        └── PID 转向
        ↓
ControlDecision
        ↓
cmd_vel / capture_trigger / sign_switch
```

## 2. 代码模块

| 文件 | 算法职责 |
| :--- | :--- |
| `run_car_node.py` | 接收 ROS 消息、维护最新传感器数据、调用控制器并发布结果 |
| `mission_controller.py` | 任务状态机、各任务控制流程、速度决策和事件触发 |
| `models.py` | 任务状态、路线方向、车辆位姿、视觉目标和控制决策的数据结构 |
| `config.py` | 读取并校验主控参数 |
| `routes.py` | Task1、Task3、停车点以及顺逆时针路线坐标 |
| `geometry.py` | 角度归一化、目标方位角、角度到图像坐标的转换 |
| `path_tracking.py` | Catmull-Rom 路径密化、路径投影和前视点选择 |
| `lidar_avoidance.py` | 障碍角度膨胀、空隙选择、脱困方向和倒车防撞 |
| `PID/pid.py` | 根据目标图像横坐标计算转向输出 |

低层算法模块不直接使用 ROS 消息或 Publisher，ROS 消息会先转换为内部数据结构。

## 3. 输入与输出

### 3.1 输入

| 输入 | 消息类型 | 用途 |
| :--- | :--- | :--- |
| `/odom_stamped` | `geometry_msgs/msg/PoseStamped` | 车辆 x、y、yaw |
| `/scan` | `sensor_msgs/msg/LaserScan` | 前向避障和倒车防撞 |
| `hobot_dnn_detection` | `ai_msgs/msg/PerceptionTargets` | 二维码框和终点框 |
| `/qrcode_information` | `std_msgs/msg/String` | 解析二维码数字和绕圈方向 |
| `sign4return` | `std_msgs/msg/Int32` | 手动切换任务状态 |

### 3.2 输出

| 输出 | 消息类型 | 用途 |
| :--- | :--- | :--- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | 发布线速度和角速度 |
| `sign_switch` | `origincar_msg/msg/Sign` | 通知二维码节点开始或切换工作状态 |
| `/capture_trigger` | `std_msgs/msg/Int32` | 通知 `voice.py` 保存当前画面并执行图生文 |

### 3.3 单周期传感器快照

ROS 回调把输入转换为 `SensorSnapshot`：

```text
SensorSnapshot
├── pose
│   ├── x
│   ├── y
│   ├── yaw
│   └── stamp_ns
├── lidar_betas
├── lidar_ranges
├── qr_detections
└── end_detections
```

控制定时器按照 `ctrl_ts` 周期调用一次 `MissionController.step()`。当前配置为 `0.02 s`，即理论控制频率 50 Hz。

二维码框和终点框按画面底边坐标从大到小排序。底边越靠下，一般表示目标越近，控制器优先使用列表中的第一个目标。

## 4. 任务状态机

主控包含六个状态：

```text
WAITING
   ↓
APPROACH_QR
   ↓
REVERSING
   ↓
CIRCLING
   ↓
RETURNING_HOME
   ↓
STOPPED
```

| 状态 | 数值 | 含义 |
| :--- | :---: | :--- |
| `WAITING` | 1 | 等待启动，输出零速度和零角速度 |
| `APPROACH_QR` | 2 | Task1，驶向二维码区域 |
| `REVERSING` | 3 | 倒车调整偏航角 |
| `CIRCLING` | 4 | 按二维码方向绕圈 |
| `RETURNING_HOME` | 5 | Task3，返回终点 |
| `STOPPED` | 6 | 最终停车 |

所有状态切换统一由 `transition_to()` 完成。进入新状态时会根据状态重置：

- 倒车开始时间。
- 路径投影下标。
- 已完成路点下标。
- 避障后回中标志。
- 拍照阶段的避障旁路标志。
- PID 内部积分和历史误差。

### 4.1 手动任务映射

| `sign4return` 数值 | 目标状态 |
| :---: | :--- |
| 1 | `APPROACH_QR` |
| 2 | `WAITING` |
| 3 | `RETURNING_HOME` |
| 4 | `REVERSING` |
| 5 | `CIRCLING` |

直接进入 `CIRCLING` 且尚未解析二维码时，控制器使用 `random_flag` 选择备用方向。

## 5. 位姿处理

`mylio` 发布 `PoseStamped`。主控从中读取：

```text
x = pose.position.x
y = pose.position.y
```

yaw 由四元数计算：

```text
sin_yaw = 2 × (qw × qz + qx × qy)
cos_yaw = 1 - 2 × (qy² + qz²)
yaw = atan2(sin_yaw, cos_yaw)
```

主控记录消息时间戳，只接受时间更新的位姿，防止乱序消息使车辆位置倒退。时间戳为零时仍允许更新，以兼容没有填写时间戳的调试消息。

## 6. 基础导航计算

### 6.1 目标相对方位角

车辆当前位姿为：

```text
P = (x, y, yaw)
```

目标点为：

```text
G = (gx, gy)
```

世界坐标系中的目标方向：

```text
target_angle = atan2(gy - y, gx - x)
```

相对车头的方位角：

```text
beta = normalize(target_angle - yaw)
```

其中 `normalize()` 将角度限制到 `[-π, π)`。算法规定左侧为正方向，与雷达方位角方向保持一致。

### 6.2 方位角转换为图像横坐标

PID 使用图像横坐标作为控制量。相机矩阵中的：

```text
fx = camera_matrix[0, 0]
cx = camera_matrix[0, 2]
```

当目标角度在 `[-1.5, 1.5] rad` 内时：

```text
target_x = cx + fx × tan(angle_error)
```

较大角度使用线性外推，防止 `tan()` 接近 `±π/2` 时数值快速发散：

```text
angle_error <= -1.5:
target_x = cx + fx × (200 × angle_error + 286)

angle_error > 1.5:
target_x = cx + fx × (200 × angle_error - 286)
```

调用时传入 `-beta`，用于将车辆方位角约定转换为当前图像转向约定。

## 7. PID 转向算法

PID 的参考值固定为：

```text
reference = 1
```

实际值为目标横坐标相对于相机主点的比例：

```text
actual = target_x / cx
```

因此误差为：

```text
error = 1 - target_x / cx
```

离散 PID：

```text
integral(k) = integral(k-1) + error(k) × dt
derivative(k) = [error(k) - error(k-1)] / dt

output = Kp × error
       + Ki × integral
       + Kd × derivative
```

当前控制器具有：

- 积分限幅：默认 `±0.5`。
- 输出限幅：默认 `±4.0`。
- 状态切换时按需要调用 `reset()`，清除历史误差和积分。

绕圈状态最终还会乘以 `Coefficient_output`：

```text
angular_z = PID_output × Coefficient_output
```

Task1 和 Task3 直接使用 PID 输出。

## 8. Task1：驶向二维码区域

Task1 同时使用位置目标、视觉二维码框和雷达避障。

### 8.1 分段目标点

车辆 x 小于 `task1_cruise_px` 时，使用临时巡航目标：

```text
goal = (TASK1_GOAL.x, task1_cruise_py)
```

这样可以让前半程轨迹更贴近大厅中较空旷的方向。

车辆 x 达到 `task1_cruise_px` 后，恢复真实 Task1 目标点：

```text
TASK1_GOAL = (4.0, 1.75)
```

### 8.2 转向目标优先级

1. 雷达判断前方完全堵塞：使用脱困方向。
2. 雷达正在避障：使用雷达选择的安全方向。
3. 前方无障碍且检测到二维码：使用二维码框中心横坐标。
4. 前方无障碍但没有二维码框：使用目标点方位角。

视觉二维码只在雷达认为当前方向安全时接管转向，雷达避障优先级高于视觉对准。

### 8.3 速度选择

| 条件 | 速度 |
| :--- | :--- |
| 前方完全堵塞 | `mid_v` |
| 正在绕障 | `min_v` |
| 正常导航 | `max_v` |
| 当前 x 大于 2.7 m | 强制 `min_v`，靠近二维码区域时降速 |

### 8.4 结束条件

车辆与 Task1 目标的平方距离满足：

```text
(x - goal_x)² + (y - goal_y)² < 0.30
```

主控进入 `REVERSING`。

## 9. 二维码方向选择

二维码字符串首先过滤出所有数字字符：

```text
"code: 17" -> 17
```

没有数字时忽略该消息，不改变当前状态。

只有满足以下条件才接收二维码方向：

- 当前状态为 `APPROACH_QR` 或 `REVERSING`。
- 当前车辆 x 不小于 2.5 m。

方向规则：

```text
奇数 -> CLOCKWISE        -> 顺时针
偶数 -> COUNTERCLOCKWISE -> 逆时针
```

允许在倒车已经开始后继续接收二维码结果，避免解码存在短暂延迟时丢失正确路线。

## 10. 倒车控制

进入 `REVERSING` 时记录开始时间。倒车期间固定输出：

```text
linear_x = speed_back
angular_z = back_omega
```

满足任意一个条件即停止倒车并进入 `CIRCLING`：

1. 当前 yaw 大于 `back_stop_yaw`。
2. 后方扇区发现距离小于 `back_obst_dist` 的障碍物。
3. 倒车时间超过 `back_max_time`。

### 10.1 后方防撞

雷达点满足以下条件时属于后方监视扇区：

```text
abs(beta) > π - back_fov
```

在有效距离点中取最小值：

```text
min(rear_ranges) < back_obst_dist
```

条件成立即认为后方存在障碍。

如果进入绕圈前仍没有二维码方向，则使用 `random_flag`：

```text
0 -> 逆时针
1 -> 顺时针
```

## 11. 路径生成

顺时针和逆时针路线由 `routes.py` 中的有序特征点组成。

### 11.1 Catmull-Rom 密化

相邻路线点之间使用开放 Catmull-Rom 曲线插值。每一段的采样数量根据段长和 `spacing` 计算：

```text
sample_count = max(2, segment_length / spacing)
```

当前：

```text
spacing = 0.02 m
```

插值后得到：

- `points`：稠密路径坐标。
- `arc_lengths`：从路径起点到每个点的累计弧长。
- `waypoint_indices`：原始特征点在稠密路径中的对应下标。

累计弧长用于寻找前视目标，路点下标用于触发拍照和调试退出事件。

### 11.2 路径形式

当前实现使用开放路径。事件列表额外追加第一个路点用于保持原有路点计数逻辑，但稠密路径本身不会强制用曲线连接回第一个路点。

如果后续需要完全闭合的循环曲线，应将路线闭合修改作为单独算法变更，并重新标定完成条件和路点事件。

## 12. 绕圈路径跟踪

### 12.1 选择路线

```text
CLOCKWISE        -> clockwise_route
COUNTERCLOCKWISE -> counterclockwise_route
UNKNOWN          -> 停车
```

### 12.2 前向路径投影

控制器不会在整条路径中搜索最近点，而是从上一次路径下标开始，只向前搜索 `circle_proj_fwd` 个稠密点：

```text
search_range = [path_index, path_index + circle_proj_fwd)
```

对窗口中每个路径点计算：

```text
d² = (path_x - vehicle_x)² + (path_y - vehicle_y)²
```

距离最小的点作为新投影点：

```text
path_index = argmin(d²)
cte = sqrt(min(d²))
```

只向前搜索可以避免车辆偏离路线后，最近点突然跳到已经走过的后方路径。

`cte` 是车辆相对于中心路径的横向偏差，用于回中判断和速度控制。

### 12.3 前视点

正常前视距离为：

```text
lookahead = circle_lookahead
```

目标累计弧长：

```text
target_s = arc_lengths[path_index] + lookahead
```

通过 `searchsorted()` 找到第一个累计弧长不小于 `target_s` 的路径点，作为当前跟踪目标。

到达特定路段时，前视距离放大为原来的 `1.25` 倍，使转向更平滑。

### 12.4 完成条件

路径投影下标满足：

```text
path_index >= path_length - 3
```

认为绕圈路径完成，停车一个控制周期并切换到 `RETURNING_HOME`。

## 13. 雷达避障算法

### 13.1 雷达角度归一化

每个雷达采样点角度为：

```text
angle(i) = angle_min + i × angle_increment
```

归一化到 `[-π, π)` 后乘以：

```text
lidar_invert
```

如果实测雷达左右方向相反，可将 `lidar_invert` 从 `1.0` 改为 `-1.0`。

### 13.2 障碍点筛选

只有同时满足以下条件的点参与前向避障：

```text
abs(beta) <= lidar_fov
range 为有限数
range > lidar_range_min
range < lidar_influence
```

### 13.3 障碍物角度膨胀

障碍膨胀半径：

```text
inflation_radius = car_half_width
                   + obstacle_radius
                   + lidar_safe_clearance
```

距离为 `r` 的障碍点对应膨胀半角：

```text
alpha = atan2(inflation_radius, max(r, 0.1))
```

该点占用的角度区间：

```text
[beta - alpha, beta + alpha]
```

算法将所有重叠占用区间合并，得到前方被障碍物占用的角度集合。

### 13.4 目标方向判断

目标方位角先限制在雷达前向视场：

```text
beta_goal = clip(beta_goal, -lidar_fov, lidar_fov)
```

- 目标方向不在任何占用区间中：直接使用目标方向。
- 目标方向被占用：计算可通行角度空隙。
- 没有满足最小宽度的空隙：返回“完全堵塞”。

### 13.5 自由空隙选择

自由区间需要满足：

```text
gap_width >= lidar_min_gap
```

每个自由区间内部使用 `lidar_edge_margin` 留出边缘余量，然后选择距离原目标方向最近的候选方向。

因此算法目标是：在避开膨胀障碍区的前提下，尽量少偏离原始导航目标。

### 13.6 完全堵塞时脱困

前方没有自由空隙时，分别计算左侧和右侧有效雷达点中的最近距离：

```text
left_room  = min(left_ranges)
right_room = min(right_ranges)
```

选择最近障碍距离更大的方向，并将目标角设到雷达视场边缘附近：

```text
edge = lidar_fov - lidar_edge_margin
```

## 14. 绕圈避障与回中

绕圈时首先根据路径前视点计算：

```text
beta_path
```

然后把 `beta_path` 交给雷达避障算法，得到：

```text
beta_command
```

两者角度差：

```text
delta = normalize(beta_command - beta_path)
```

当：

```text
abs(delta) > 0.05 rad
```

认为正在避障。避障方向再乘以 `1.3`，增强绕障转向：

```text
beta_command = beta_command × 1.3
```

开始避障后进入 `recovering` 状态。即使障碍已经消失，只要：

```text
cte >= recover_cte_done
```

车辆仍被认为处于回中阶段，并保持低速。只有横向误差小于 `recover_cte_done` 才退出回中状态。

## 15. 绕圈速度决策

速度选择顺序如下：

| 条件 | 速度 |
| :--- | :--- |
| 尚未完成第一个路线点 | `mid_v` |
| 正在避障 | `min_v` |
| 正在回中 | `min_v` |
| `cte > circle_cte_slow` | `min_v` |
| `abs(PID_output) > circle_slow_thresh` | `min_v` |
| 以上条件均不满足 | `max_v` |

最后确保：

```text
speed >= min_v
```

该策略的目的，是在直线路段提高速度，在大转向、避障或偏离中心线时主动降速。

## 16. 路点事件与拍照

车辆投影下标越过某个原始路线点在稠密路径中的下标时，路点计数器加一。

### 16.1 调试退出

当：

```text
waypoint_index == quit_point
```

主控切换到 `WAITING` 并停车。正常完整比赛配置可将 `quit_point` 设置为不会在路线中出现的较大值，例如当前的 `99`。

### 16.2 图生文拍照

达到指定路点并满足方向对应的位置条件时：

```text
capture_requested = True
```

ROS 节点随后向 `/capture_trigger` 发布 `Int32(data=1)`，`voice.py` 保存最新图像并调用 PC 图生文服务。

当前拍照条件：

```text
顺时针：waypoint_index == 7 且 x > 2.6
逆时针：waypoint_index == 7 且 x < 2.4
```

代码中的下标从零开始，因此日志中对应“到达点8”。

### 16.3 拍照区域避障旁路

触发拍照后会暂时设置：

```text
bypass_obstacle_avoidance = True
```

这是为了避免画面中的特定目标区域被控制策略当作需要绕行的障碍。满足方向相关的位置和 yaw 条件后恢复雷达避障。

> 该旁路会暂时降低雷达避障优先级，属于需要重点实车验证的安全策略。修改路线或拍照区域后，必须重新检查恢复条件。

## 17. Task3：返回与停车

Task3 的基础导航目标：

```text
TASK3_GOAL = (0.452, 0.355)
```

转向优先级与 Task1 相同：

1. 完全堵塞时使用脱困方向。
2. 正在避障时使用雷达方向。
3. 无障碍且检测到终点框时，使用终点框中心横坐标。
4. 没有终点框时，使用 Task3 目标方位角。

### 17.1 视觉停车

当终点框底边满足：

```text
end_detection.bottom_y > stop_end_threshold
```

主控立即切换到 `STOPPED`。

### 17.2 位置停车

停车参考点：

```text
STOP_GOAL = (0.25, 0.15)
```

平方距离满足：

```text
(x - stop_x)² + (y - stop_y)² < 0.25
```

主控切换到 `STOPPED`。

`STOPPED` 每个控制周期都输出：

```text
linear_x = 0
angular_z = 0
```

## 18. 主要参数及影响

配置文件：

```text
TurnRight/src/run_car_pkg/params/run_car_node.yaml
```

### 18.1 速度与 PID

| 参数 | 作用 | 增大后的主要影响 |
| :--- | :--- | :--- |
| `max_v` | 正常导航最高速度 | 直线更快，但留给转向和避障的时间更少 |
| `mid_v` | 脱困和初始绕圈速度 | 提高脱困速度，但可能加剧偏航 |
| `min_v` | 避障、回中和大转向速度 | 提高最低效率，但降低稳定余量 |
| `Kp` | 当前横向误差响应 | 转向更快，过大容易振荡 |
| `Ki` | 累积误差补偿 | 可消除长期偏差，过大容易积分累积 |
| `Kd` | 误差变化抑制 | 可减少过冲，过大容易放大噪声 |
| `Coefficient_output` | 绕圈转向输出倍率 | 绕圈转向更强 |
| `ctrl_ts` | 控制周期 | 必须与实际控制频率匹配 |

### 18.2 雷达避障

| 参数 | 作用 |
| :--- | :--- |
| `open_lidar_bearing` | 雷达避障总开关 |
| `lidar_fov` | 前向避障视场半角 |
| `lidar_influence` | 障碍物开始影响控制的最大距离 |
| `lidar_range_min` | 过滤过近的自身反射和异常点 |
| `car_half_width` | 车辆半宽，用于障碍膨胀 |
| `obstacle_radius` | 锥桶等障碍物的估计半径 |
| `lidar_safe_clearance` | 额外安全间隙 |
| `lidar_min_gap` | 可通行角度空隙的最小宽度 |
| `lidar_edge_margin` | 目标方向与空隙边缘的角度余量 |
| `lidar_invert` | 雷达左右方向修正 |

避障实际使用的膨胀半径是三者之和：

```text
car_half_width + obstacle_radius + lidar_safe_clearance
```

### 18.3 绕圈跟踪

| 参数 | 作用 | 调整方向 |
| :--- | :--- | :--- |
| `circle_lookahead` | 路径前视距离 | 大：平滑但切弯；小：贴线但容易振荡 |
| `circle_proj_fwd` | 前向投影搜索点数 | 必须覆盖单周期可能前进的路径长度 |
| `circle_cte_slow` | 横向偏差降速阈值 | 小：更谨慎；大：允许更大偏差 |
| `recover_cte_done` | 退出回中状态的偏差阈值 | 小：回中要求更严格 |
| `circle_slow_thresh` | 转向输出降速阈值 | 小：较轻转向就降速 |

### 18.4 倒车与任务

| 参数 | 作用 |
| :--- | :--- |
| `speed_back` | 倒车线速度，通常为负数 |
| `back_omega` | 倒车角速度 |
| `back_max_time` | 倒车超时保护 |
| `back_stop_yaw` | 倒车姿态完成阈值 |
| `back_obst_dist` | 后方障碍触发距离 |
| `back_fov` | 后方监视扇区半角 |
| `random_flag` | 未解析二维码时的备用方向 |
| `quit_point` | 调试时提前退出绕圈的路点 |
| `task1_cruise_px` | Task1 临时目标切换的 x 阈值 |
| `task1_cruise_py` | Task1 前半程临时目标 y |

## 19. 调参建议

建议严格按以下顺序调试，避免多个参数互相影响：

### 第一步：确认坐标和方向

1. 确认 `/odom_stamped` 的 x、y、yaw 方向正确。
2. 确认正角速度对应实际预期方向。
3. 确认雷达左、右方向；错误时调整 `lidar_invert`。
4. 低速测试 `angle_to_image_x` 与底盘转向符号是否一致。

### 第二步：关闭避障调路径

临时设置：

```yaml
open_lidar_bearing: 0
```

使用较低速度调试：

- 路线坐标。
- `circle_lookahead`。
- PID。
- `circle_slow_thresh`。

确保顺、逆时针都能完成路径。

### 第三步：启用避障

恢复：

```yaml
open_lidar_bearing: 1
```

依次调整：

1. `lidar_influence`。
2. `lidar_safe_clearance`。
3. `lidar_edge_margin`。
4. `lidar_min_gap`。
5. `circle_cte_slow` 和 `recover_cte_done`。

### 第四步：提高速度

路径和避障稳定后，再按以下顺序逐步增加：

1. `min_v`
2. `mid_v`
3. `max_v`

每次只改一个参数，并记录顺、逆时针的表现。

## 20. 安全与边界情况

当前算法已处理：

- 空 DNN ROI：忽略该目标。
- 二维码没有数字：忽略该消息。
- 里程计消息乱序：忽略旧位姿。
- 雷达尚未到达：按目标方向导航。
- 前方完全堵塞：选择左右更空的一侧。
- 后方存在障碍：立即结束倒车。
- 倒车时间过长：超时进入下一状态。
- 路线方向未知：使用备用方向或停车。
- 路径点不足：停车。
- 等待和最终停止：持续输出全零指令。
- 参数非法：节点启动时直接报告错误。

仍需重点关注：

1. 算法目前没有根据里程计或雷达“最后更新时间”执行传感器超时停车。
2. 拍照区域会暂时旁路雷达避障，必须通过实车确认恢复条件。
3. 路线为开放曲线，修改末端点时需要重新验证绕圈完成条件。
4. `angle_to_image_x` 的大角度线性外推参数 `200` 和 `286` 来自当前经验公式，更换相机后需要重新标定。
5. Task1 和 Task3 的位置结束条件使用平方距离阈值，调整时不要误认为参数本身就是实际米制距离。

## 21. 推荐验证顺序

1. 运行几何、路径、雷达和状态机单元测试。
2. 使用同一 rosbag 回放，比较修改前后的 `cmd_vel`。
3. 架空车轮验证速度和转向符号。
4. 使用低速参数测试 Task1 和倒车。
5. 分别测试奇数二维码和偶数二维码。
6. 单独测试顺、逆时针完整绕圈。
7. 验证第 8 个点只触发一次拍照。
8. 验证 Task3 视觉停车和位置停车。
9. 最后恢复比赛速度执行完整流程。

调试过程中可查看：

```bash
ros2 topic echo /odom_stamped
ros2 topic echo /cmd_vel
ros2 topic echo /capture_trigger
ros2 topic hz /scan
ros2 topic hz /hobot_dnn_detection
```
