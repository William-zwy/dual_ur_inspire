#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rtde_receive import RTDEReceiveInterface
import time


class ReceiverNode(Node):

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        
        self.timer = self.create_timer(1.0/100,self.timer_callback)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.joint_names = [
            "left_shoulder_pan_joint",
            "left_shoulder_lift_joint",
            "left_elbow_joint",
            "left_wrist_1_joint",
            "left_wrist_2_joint",
            "left_wrist_3_joint",
            "right_shoulder_pan_joint",
            "right_shoulder_lift_joint",
            "right_elbow_joint",
            "right_wrist_1_joint",
            "right_wrist_2_joint",
            "right_wrist_3_joint"
        ]

        self.rtde_left_r = RTDEReceiveInterface("192.168.1.40")
        self.rtde_right_r = RTDEReceiveInterface("192.168.1.41")
        
        
    def timer_callback(self):
        
        current_time = self.get_clock().now()
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = self.joint_names
        
        left_joint_positions = self.rtde_left_r.getActualQ()
        right_joint_positions = self.rtde_right_r.getActualQ()
        
        joint_state.position = list(left_joint_positions) + list(right_joint_positions)
        self.joint_state_pub.publish(joint_state)
        

def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode("rtde_receiver_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rtde_left_r.disconnect()
        node.rtde_right_r.disconnect()
        node.destroy_node()
        rclpy.shutdown()