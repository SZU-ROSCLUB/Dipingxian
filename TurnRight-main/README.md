# 第二十一届全国大学生智能汽车竞赛 · 智慧医疗组

> 基于 **ROS 2** 的智能小车系统，集成定位、底盘驱动、二维码解码与图生文四大功能模块，可完全脱离官方代码独立运行。

---

## 目录

- [项目简介](#项目简介)
- [核心功能](#核心功能)
- [运行环境](#运行环境)
- [项目结构](#项目结构)
- [配置说明](#配置说明)
- [编译](#编译)
- [启动](#启动)

## 项目简介

本项目面向第二十一届全国大学生智能汽车竞赛智慧医疗组，基于 ROS 2 构建，涵盖 **定位**、**小车驱动**、**二维码解码** 与 **图生文** 四大功能，可完全脱离官方提供的代码独立运行。

其中两个核心模块具有以下特点：

- **mylio**：订阅激光雷达数据并输出里程计信息，精度较高且资源占用低。
- **qrcode**：采用 WeChat 与 pyzbar 双引擎扫码，静态扫码极限距离约 1.5 m。

## 核心功能

| 功能包 | 说明 |
| :--- | :--- |
| `car_base` | 精简版底盘驱动 |
| `lidar` | N10 激光雷达驱动 |
| `mylio` | 基于激光雷达的里程计，精度较高、轻量 |
| `origincar_msg` | 官方消息（msg）定义 |
| `qrcode` | WeChat + pyzbar 双引擎二维码解码，静态极限距离约 1.5 m |
| `run_car_pkg` | 主控逻辑 |

## 运行环境

| 项目 | 说明 |
| :--- | :--- |
| 小车端 | RDK X5（ARM 开发板），运行 ROS 2 各功能节点 |
| PC 端 | 运行图生文推理服务（FastAPI，经 uvicorn 启动） |
| 中间件 | ROS 2 |
| 激光雷达 | N10 |
| 视觉模型 | YOLOv5 |
| 二维码引擎 | OpenCV WeChatQRCode + pyzbar |

> 图生文采用 **小车端采集 + PC 端推理** 的协同架构：小车端负责图像采集，PC 端运行推理服务并返回文本描述。两端需处于同一局域网。

## 项目结构

```text
TurnRight/
├── config/                     # YOLO 配置文件
│   ├── coco.list
│   └── yolov5xworkconfig.json
│
├── launch/                     # 启动文件
│   ├── bringup_launch.py
│   ├── run_car_node.launch.py
│   └── usb_display.launch.py
│
├── models/                     # YOLO 模型
│   ├── 25_08_05_yolov5.bin
│   └── horizon_x5.bin
│
├── src/
│   ├── car_base/               # 精简版底盘
│   ├── lidar/                  # N10 激光雷达驱动
│   ├── mylio/                  # 激光雷达里程计
│   ├── origincar_msg/          # 官方 msg
│   ├── qrcode/                 # 二维码节点
│   └── run_car_pkg/            # 主控逻辑
│
├── test_scripts/               # 任务切换（调试）
│   └── test_taskswitch.py
│
├── voice/                      # 存放待处理图片
│
├── run_display.sh              # 屏幕显示脚本
└── voice.py                    # 图生文节点
```

`run_car_pkg` 主控包内部按职责拆分：

- `run_car_node.py`：ROS 话题收发、消息转换与定时调度。
- `mission_controller.py`：任务状态机和各阶段控制流程。
- `config.py`：ROS 参数读取、分组和启动校验。
- `models.py`：任务状态、车辆位姿、感知快照和控制决策。
- `geometry.py`、`path_tracking.py`：目标方位角与路径跟踪计算。
- `lidar_avoidance.py`：前向避障、脱困方向和倒车防撞。
- `routes.py`：任务目标点与顺、逆时针路线。

## 配置说明

### 1. 图生文节点

- **`voice.py`**
  - `PC_URL`：修改为当前网络下 PC 的 IP 地址。
  - `WATCH_DIR`：可保持默认（不修改会自动创建）。
- **`run_display.sh`**
  - 修改其中 `voice.py` 的路径。

> **获取本机 IP**
>
> ```bash
> ip addr show | grep "inet " | grep -v 127.0.0.1
> ```

### 2. 二维码节点

- **`qrcode.py`**：将 `detector` 下的 4 个路径修改为实际模型路径。

### 3. YOLO 节点

- **`yolov5xworkconfig.json`**：将 `model_file` 与 `cls_names_list` 修改为实际路径。


## 编译

项目使用 N10 激光雷达，需提前完成编译。**推荐逐个功能包编译**，便于定位错误。

```bash
# 在工作空间根目录（TurnRight/）下执行
colcon build --packages-select <package_name>
source install/setup.bash
```

## 启动

> 以下命令中的路径以 `~/TurnRight` 为例，请按实际工作空间路径修改。

### 小车端

```bash
# 1. 底盘 / 传感器
ros2 launch ~/TurnRight/launch/bringup_launch.py

# 2. 主控逻辑
ros2 launch ~/TurnRight/launch/run_car_node.launch.py

# 3. 摄像头与扫码节点
ros2 launch ~/TurnRight/launch/usb_display.launch.py

# 4. 任务切换(初始默认任务2)
python3 ~/TurnRight/test_scripts/test_taskswitch.py

# 5. 屏幕显示
bash ~/TurnRight/run_display.sh
```

### PC 端

在 PC 端项目目录下启动图生文推理服务：

```bash
uvicorn server:app --host 0.0.0.0 --port 8000
```

> 请确保 PC 与小车处于同一局域网，且 `voice.py` 中的 `PC_URL` 已指向 PC 的 IP 地址。

### 鸣谢

感谢所有提供帮助的学长学姐及同学
