#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rtde_control import RTDEControlInterface
import time


class RightCtrlNode(Node):

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        
        self.create_subscription(Float64MultiArray,'/ur_arm_right_ros2_controller/commands',self.command_callback,10)
        self.timer = self.create_timer(1.0/100,self.timer_callback)     
        self.rtde_c = RTDEControlInterface("192.168.1.41")
        
        init_cmd = [-0.51,-2.14,-1.84,0.84,2.08,0.0]
        self.cmd = init_cmd
        
        self.init_flag = False
        
        self.acceleration=0.5
        self.speed=0.5
        self.time=0.008
        self.lookahead_time=0.1
        self.gain=300

    def timer_callback(self):
        if self.init_flag == False:
            self.rtde_c.moveJ(self.cmd)
            self.init_flag = True
            
        self.rtde_c.servoJ(self.cmd,
                self.acceleration,
                self.speed,
                self.time,
                self.lookahead_time,
                self.gain
            )
        
    def command_callback(self, msg):
        if len(msg.data) == 6:
            self.cmd = msg.data
            self.get_logger().debug(f"新指令: {msg.data}", throttle_duration_sec=1)
            
    def __del__(self):
        if hasattr(self, 'rtde'):
            self.rtde.servoStop()
            self.rtde.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = RightCtrlNode("rtde_right_ctrl_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()