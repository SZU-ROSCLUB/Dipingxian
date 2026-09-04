import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image ,CompressedImage
from std_msgs.msg import String, Int32
from origincar_msg.msg import Sign
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion
import pyzbar.pyzbar as pyzbar
import os
from datetime import datetime
from geometry_msgs.msg import Twist, TransformStamped, Pose2D

TASK1 = 1
TASK2 = 3

class QrCodeDetection(Node):
  def __init__(self):
    super().__init__('QRcodeSub')
 
    self.ImageSub = self.create_subscription(CompressedImage, '/image', self.image_callback, 10)
    self.qrcode_publisher = self.create_publisher(String, "/qrcode_information", 10)
    self.SignSwitchSub = self.create_subscription(Sign, "/sign_switch", self.sign_switch_callback, 10)
    self.OdomTestSub = self.create_subscription(Pose2D, 'odom_test', self.odom_test_callback, 10)
    try:
            self.detector = cv2.wechat_qrcode_WeChatQRCode(
                "/userdata/TurnRight/src/qrcode/qrcode/model/detect.prototxt",
                "/userdata/TurnRight/src/qrcode/qrcode/model/detect.caffemodel",
                "/userdata/TurnRight/src/qrcode/qrcode/model/sr.prototxt",
                "/userdata/TurnRight/src/qrcode/qrcode/model/sr.caffemodel"
            )
            self.get_logger().info(f"WeChat二维码模型加载成功")
    except Exception as e:
            self.get_logger().fatal(f"WeChat二维码模型加载失败: {str(e)}，请检查模型路径")
            raise RuntimeError("模型加载失败，节点无法启动") from e
    
    self.info_result = String()
    self.bridge = CvBridge()
    self.sign_msg = Sign()
    # self.node_run = False   #TODO 实际扫码极限距离小于半场，True和False都能满足需求，暂时不启用过半场限制
    # self.qr_exist_flag = False  
    # self.half_field = False 

    self.node_run = True   #TODO
    self.qr_exist_flag = True  
    self.half_field = True  
    self.task = TASK2  
    self.current_pos = [0.0, 0.0, 0.0]  # x y theta v omega8
    
    
  def sign_switch_callback(self, msg): 
    # 安全检查，确保消息格式正确，防止因消息异常导致节点崩溃
    if not hasattr(msg, 'sign_data'):
        self.get_logger().error(f"收到无效Sign消息,缺失sign_data字段: {msg}", throttle_duration_sec=1)
        return
    else:

        if msg.sign_data == 0: # yolo识别到的二维码
            self.qr_exist_flag = True
            self.get_logger().info("识别到二维码")
        if msg.sign_data == 1: #任务1
            self.task = TASK1
            self.get_logger().info("任务一中")
    
  def image_callback(self, msg):
    #上一次解码未完成时直接丢弃当前帧 
    
    # if self.decode_in_progress:
    #     return
    # self.decode_in_progress = True
    #临时变量
    current_half_field = self.half_field
    current_node_run = self.node_run
    current_qr_exist_flag = self.qr_exist_flag
    if current_half_field:  #任务要求必须过半场       
        if not (current_node_run and current_qr_exist_flag):
            return  
              
        # ROI裁剪，只处理有效区域，进一步降低计算量
        cv2_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='mono8')[155:, :]
        self.get_logger().info(f"图像尺寸: {cv2_image.shape}, 数据类型: {cv2_image.dtype}")
        # pyzbar快速解码
        barcodes = pyzbar.decode(cv2_image)
        self.get_logger().info(f"pyzbar解码结果: {len(barcodes)} barcodes found")
        if barcodes: 
            self.node_run = False
            self.qr_exist_flag = False
            for barcode in barcodes:
                qr_data = barcode.data.decode("utf-8")
                self.info_result.data = qr_data
                self.qrcode_publisher.publish(self.info_result)
                self.get_logger().info(f"\033[94m[pyzbar解码成功] 二维码内容：{qr_data}\033[0m")
                if rclpy.ok():
                    rclpy.shutdown()
            return  
        # pyzbar解码失败，启动wechat扫码
        self.get_logger().info("pyzbar解码失败,启动wechat解码", throttle_duration_sec=2)
        wechat_res = self.detector.detectAndDecode(cv2_image)[0]
        self.get_logger().info(f"wechat解码结果: {len(wechat_res)} codes found")
        if wechat_res: 
            self.node_run = False
            self.qr_exist_flag = False
            for r in wechat_res:
                self.info_result.data = str(r)
                self.qrcode_publisher.publish(self.info_result)
                self.get_logger().info(f"\033[94m[wechat解码成功] 二维码内容：{self.info_result.data}\033[0m")
                if rclpy.ok():
                    rclpy.shutdown()
            return
        
  def odom_test_callback(self, msg):
        self.current_pos[0] = msg.x
        if self.current_pos[0] > 2.4 and not self.half_field:
            self.half_field = True
            self.node_run = True
            self.get_logger().info(f"已过半场,x坐标: {self.current_pos[0]:.2f}，二维码解码已开启")
            
def main(args=None):
    rclpy.init(args=args)
    qrCodeDetection = QrCodeDetection()
    while rclpy.ok():
        rclpy.spin(qrCodeDetection) 
    qrCodeDetection.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()