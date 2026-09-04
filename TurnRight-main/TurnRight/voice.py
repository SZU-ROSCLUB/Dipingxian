import os
import time
import threading
import queue
import requests
import signal
import tkinter as tk
import tkinter.font as tkfont
import io

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage

# PC_URL = "http://192.168.144.30:8000/describe" # 229_WIFI1
# PC_URL = "http://10.122.97.91:8000/describe" # OPPO Reno
PC_URL = "http://192.168.43.11:8988/describe" # Young_999

WATCH_DIR = "/userdata/TurnRight/voice/"
VALID_EXT = (".jpg", ".jpeg", ".png")

# 屏幕分辨率（4.3寸常见 800x480）
SCREEN_W = 800
SCREEN_H = 480

# 线程安全队列：所有要显示的内容先丢这里，由主线程取出更新屏幕
display_queue = queue.Queue()
shutdown_flag = threading.Event()

# ============== 图生文部分 ==============
def describe_image(image_path):
    # 读取本地保存的整张图片
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"无法读取图片: {image_path}")
    
    # 裁剪下半部分
    height, width = img.shape[:2]
    bottom_half = img[height // 2:, :]  # 从中间到底部，宽度全选
    
    # 将裁剪后的下半部分编码为 JPEG 格式的字节流
    success, encoded_image = cv2.imencode('.jpg', bottom_half)
    if not success:
        raise ValueError("图片编码失败")
    
    # 转换为字节流对象，方便 requests 发送
    image_bytes = io.BytesIO(encoded_image.tobytes())
    
    # 发送给 PC 端
    r = requests.post(
        PC_URL, 
        files={"file": ("image.jpg", image_bytes, "image/jpeg")}, 
        timeout=30
    )
    return r.json()["text"]

def wait_until_stable(path, check_interval=0.3, stable_rounds=2):
    last_size = -1
    stable_count = 0
    while stable_count < stable_rounds:
        try:
            size = os.path.getsize(path)
        except FileNotFoundError:
            return
        if size == last_size and size > 0:
            stable_count += 1
        else:
            stable_count = 0
            last_size = size
        time.sleep(check_interval)


def handle_image(image_path):
    print(f"[检测到新图片] {image_path}")
    wait_until_stable(image_path)
    try:
        text = describe_image(image_path)
        print("描述:", text)
        # 丢进队列，让主线程更新屏幕第二行
        display_queue.put({"type": "desc", "text": text})
    except Exception as e:
        print("图生文处理失败:", e)
        display_queue.put({"type": "desc", "text": "图像识别失败"})


class ImageHandler(FileSystemEventHandler):
    def on_created(self, event):
        if event.is_directory:
            return
        if event.src_path.lower().endswith(VALID_EXT):
            handle_image(event.src_path)

    def on_moved(self, event):
        if event.is_directory:
            return
        if event.dest_path.lower().endswith(VALID_EXT):
            handle_image(event.dest_path)


# ============== 二维码部分 ==============

class QRCodeDisplayNode(Node):
    def __init__(self):
        super().__init__('qrcode_display_node')
        self.qr_shown = False  # 只显示一次
        self.sub = self.create_subscription(
            String, '/qrcode_information', self.qr_callback, 10
        )
        self.get_logger().info("二维码显示节点已启动，监听 /qrcode_information")
        # 拍照：缓存最新帧 + 收主程序信号存图
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.create_subscription(CompressedImage, '/image', self.image_cb, 10)
        self.create_subscription(Int32, '/capture_trigger', self.capture_cb, 10)
        self.get_logger().info("拍照节点已就绪，监听 /image 与 /capture_trigger")

    def qr_callback(self, msg):
        if self.qr_shown:
            return

        raw = msg.data.strip()
        digits = ''.join(filter(str.isdigit, raw))
        if not digits:
            self.get_logger().warn(f"二维码内容无数字: {raw}")
            return

        num = int(digits)
        direction = "顺时针" if num % 2 else "逆时针"
        text = f"数字：{num}，{direction}"

        self.get_logger().info(f"[二维码] 显示: {text}")
        # 丢进队列，让主线程更新屏幕第一行
        display_queue.put({"type": "qr", "text": text})
        self.qr_shown = True

    def image_cb(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"[拍照] 解码失败: {e}")

    def capture_cb(self, msg):
        if msg.data != 1:
            return
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            self.get_logger().warn("[拍照] 收到信号但还没有图像帧")
            return
        final = os.path.join(WATCH_DIR, f"capture_{int(time.time()*1000)}.jpg")
        try:
            ok = cv2.imwrite(final, frame)         # 直接写 .jpg，imwrite 认得
            if not ok:
                self.get_logger().error("[拍照] imwrite 返回 False")
                return
            self.get_logger().info(f"[拍照] 已保存: {final}")
        except Exception as e:
            self.get_logger().error(f"[拍照] 保存失败: {e}")

# ============== 屏幕显示部分 ==============

class DisplayApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("RDK Display")
        self.root.configure(bg="black")
        # 全屏显示
        self.root.attributes("-fullscreen", True)
        self.root.geometry(f"{SCREEN_W}x{SCREEN_H}")
        # 按 Esc 退出全屏（调试用）
        self.root.bind("<Escape>", lambda e: self.root.attributes("-fullscreen", False))
        self.root.bind("q", lambda e: self.root.destroy())

        # 第一行：二维码内容，大字、醒目色
        self.qr_font = tkfont.Font(size=60, weight="bold")
        self.qr_label = tk.Label(
            self.root, text="等待二维码...", font=self.qr_font,
            fg="#00FF00", bg="black", anchor="center"
        )
        self.qr_label.pack(side="top", fill="x", pady=(20, 10))

        # 分隔线
        sep = tk.Frame(self.root, height=2, bg="#444444")
        sep.pack(fill="x", padx=20)

        # 剩余区域：图生文内容，自动换行
        self.desc_font = tkfont.Font(size=44, weight="bold")
        self.desc_label = tk.Label(
            self.root, text="等待图像识别...", font=self.desc_font,
            fg="#0C22E7", bg="black", anchor="n", justify="center",
            wraplength=SCREEN_W - 40  # 超过宽度自动换行
        )
        self.desc_label.pack(side="top", fill="both", expand=True, padx=20, pady=10)

    def poll_queue(self):
    # 先看是不是要退出了
        if shutdown_flag.is_set():
            self.root.quit()      # 让 mainloop 返回
            return
        try:
            while True:
                msg = display_queue.get_nowait()
                if msg["type"] == "qr":
                    self.qr_label.config(text=msg["text"])
                elif msg["type"] == "desc":
                    self.desc_label.config(text=msg["text"])
        except queue.Empty:
            pass
        self.root.after(100, self.poll_queue)

    def run(self):
        self.root.after(100, self.poll_queue)
        self.root.mainloop()


# ============== 主程序 ==============

def ros_spin(node):
    try:
        while rclpy.ok() and not shutdown_flag.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print("ros spin 退出:", e)


def main():
    os.makedirs(WATCH_DIR, exist_ok=True)

    # 注册 Ctrl+C 处理：按下就置退出标志
    def on_sigint(signum, frame):
        print("\n收到退出信号，正在关闭...")
        shutdown_flag.set()
    signal.signal(signal.SIGINT, on_sigint)

    # 启动文件监听
    print(f"[监听中] {WATCH_DIR}")
    observer = Observer()
    observer.schedule(ImageHandler(), WATCH_DIR, recursive=False)
    observer.start()

    # 启动 ROS2，spin 放后台线程
    rclpy.init()
    node = QRCodeDisplayNode()
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # GUI 主线程
    app = DisplayApp()
    try:
        app.run()
    finally:
        # 走到这说明 mainloop 已结束，开始收尾
        shutdown_flag.set()
        observer.stop()
        observer.join(timeout=2)        # 加超时，最多等 2 秒
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print("已退出")


if __name__ == "__main__":
    main()