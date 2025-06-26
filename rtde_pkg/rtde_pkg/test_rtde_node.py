#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time


class RTDENode(Node):

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        # 连接 UR 机械臂
        rtde_c = RTDEControlInterface("192.168.1.40")  # 换成你的 UR IP

        # 定义目标关节位置（弧度）
        q1 = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        q2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        acceleration = 0.5  # 归一化加速度
        speed = 0.5        # 归一化速度
        servo_time = 0.008  # 8ms (125Hz)
        lookahead_time = 0.1  # 100ms 前瞻
        gain = 500          # 伺服增益

        # 控制机械臂移动
        # rtde_c.moveJ(q1)  # Joint space movement
        rtde_c.servoJ(q2,acceleration,speed,servo_time,lookahead_time,gain)

        # 保持位置 2 秒
        time.sleep(2)

        # 关闭连接
        rtde_c.stopScript()
        rtde_c.servoStop()

        # # 连接 UR 机械臂（替换为你的 IP 地址）
        # rtde_r = RTDEReceiveInterface("192.168.1.40")

        # # 获取当前关节角度（单位：弧度）
        # joint_positions = rtde_r.getActualQ()

        # # 打印结果
        # print("当前关节角度（rad）:", joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = RTDENode("rtde_node")
    rclpy.spin(node)
    rclpy.shutdown()
