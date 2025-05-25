import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class DualHandMimicNode(Node):
    def __init__(self):
        super().__init__('dual_hand_mimic_node')

        # mimic_joint: (source_joint, multiplier, offset)
        self.mimic_map = {
            # Right hand
            'right_hand_R_thumb_intermediate_joint': ('right_hand_R_thumb_proximal_pitch_joint', 1.6, 0.0),
            'right_hand_R_thumb_distal_joint': ('right_hand_R_thumb_intermediate_joint', 1.6, 0.0),
            'right_hand_R_index_intermediate_joint': ('right_hand_R_index_proximal_joint', 1.0, 0.0),
            'right_hand_R_middle_intermediate_joint': ('right_hand_R_middle_proximal_joint', 1.0, 0.0),
            'right_hand_R_ring_intermediate_joint': ('right_hand_R_ring_proximal_joint', 1.0, 0.0),
            'right_hand_R_pinky_intermediate_joint': ('right_hand_R_pinky_proximal_joint', 1.0, 0.0),

            # Left hand
            'left_hand_L_thumb_intermediate_joint': ('left_hand_L_thumb_proximal_pitch_joint', 1.6, 0.0),
            'left_hand_L_thumb_distal_joint': ('left_hand_L_thumb_intermediate_joint', 1.6, 0.0),
            'left_hand_L_index_intermediate_joint': ('left_hand_L_index_proximal_joint', 1.0, 0.0),
            'left_hand_L_middle_intermediate_joint': ('left_hand_L_middle_proximal_joint', 1.0, 0.0),
            'left_hand_L_ring_intermediate_joint': ('left_hand_L_ring_proximal_joint', 1.0, 0.0),
            'left_hand_L_pinky_intermediate_joint': ('left_hand_L_pinky_proximal_joint', 1.0, 0.0),
        }

        # 分开发布
        self.left_mimic_joints = [j for j in self.mimic_map if j.startswith('L_')]
        self.right_mimic_joints = [j for j in self.mimic_map if j.startswith('R_')]

        self.left_pub = self.create_publisher(Float64MultiArray, '/left_hand_mimic_controller/commands', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/right_hand_mimic_controller/commands', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        left_data = []
        right_data = []

        for joint in self.left_mimic_joints:
            src, mult, offset = self.mimic_map[joint]
            pos = name_to_pos.get(src, None)
            if pos is not None:
                left_data.append(mult * pos + offset)
            else:
                self.get_logger().warn(f'Left: Missing source joint {src}')

        for joint in self.right_mimic_joints:
            src, mult, offset = self.mimic_map[joint]
            pos = name_to_pos.get(src, None)
            if pos is not None:
                right_data.append(mult * pos + offset)
            else:
                self.get_logger().warn(f'Right: Missing source joint {src}')

        if left_data:
            msg = Float64MultiArray()
            msg.data = left_data
            self.left_pub.publish(msg)

        if right_data:
            msg = Float64MultiArray()
            msg.data = right_data
            self.right_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualHandMimicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
