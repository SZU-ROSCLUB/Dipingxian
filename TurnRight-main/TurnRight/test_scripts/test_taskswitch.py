#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class TaskSwitchNode(Node):
    def __init__(self):
        super().__init__("task_switch_node")
        # 发布任务控制信号
        self.sign_pub = self.create_publisher(Int32, "sign4return", 10)
        self.get_logger().info("="*60)
        self.get_logger().info(" RDK X5 任务切换节点启动成功！")
        self.get_logger().info(" 键盘控制任务：")
        self.get_logger().info("  1 → 任务1   2 → 任务2   3 → 任务3")
        self.get_logger().info("  4 → 倒车    5 → 绕圈")
        self.get_logger().info("="*60)

    def run_key_control(self):
        # 键盘任务控制主循环
        while True:
            try:
                task = input("输入任务号(1-5):")
                if task in ["1","2","3","4","5"]:
                    self.sign_pub.publish(Int32(data=int(task)))
                    self.get_logger().info(f" 已发送任务：{task}")
            except KeyboardInterrupt:
                self.get_logger().info(" 任务切换节点退出")
                break
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    task_node = TaskSwitchNode()
    # 运行键盘控制
    task_node.run_key_control()
    # 销毁节点
    task_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()