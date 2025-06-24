import sys

from inspire_hand_interfaces.msg import AngleState, ForceState
from inspire_hand_demo.inspire_api import InspireHandAPI

import rclpy
from rclpy.node import Node
import serial.serialutil

class InspireHandPub(Node):
    def __init__(self):
        super().__init__('inspire_hand_state_publisher')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('max_reconnect_attempts', 5)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.auto_reconnect = self.get_parameter('auto_reconnect').get_parameter_value().bool_value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').get_parameter_value().integer_value
        
        # Create publishers
        self.angle_pub = self.create_publisher(AngleState, 'inspire_hand_angle_state', 10)
        self.force_pub = self.create_publisher(ForceState, 'inspire_hand_force_state', 10)

        # Initialize connection
        self.hand = None
        self._initialize_connection()
        
        # Create timer
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Inspire Hand Publisher started on {self.serial_port}')
    
    def _initialize_connection(self):
        """Initialize connection to the hand."""
        try:
            self.hand = InspireHandAPI(self.serial_port, self.baudrate, self.device_id)
            self.get_logger().info(f'Successfully connected to hand on {self.serial_port}')
            return True
        except (serial.serialutil.SerialException, Exception) as e:
            self.get_logger().error(f'Failed to connect to hand: {str(e)}')
            if not self.auto_reconnect:
                raise
            return False
    
    def _reconnect_if_needed(self):
        """Attempt reconnection if the connection is lost."""
        if self.hand is None or not self.hand.is_connected():
            if self.auto_reconnect:
                self.get_logger().warn('Connection lost, attempting to reconnect...')
                return self._initialize_connection()
            return False
        return True

    def timer_callback(self):
        """Timer callback to publish hand state."""
        try:
            if not self._reconnect_if_needed():
                return
            
            angle_state = AngleState()
            force_state = ForceState()

            angle = self.hand.read6(self.device_id, 'angleAct')
            force = self.hand.read6(self.device_id, 'forceAct')
            
            if angle and len(angle) == 6:
                angle_state.id = self.device_id
                angle_state.angle1 = int(angle[0])
                angle_state.angle2 = int(angle[1])
                angle_state.angle3 = int(angle[2])
                angle_state.angle4 = int(angle[3])
                angle_state.angle5 = int(angle[4])
                angle_state.angle6 = int(angle[5])
                self.angle_pub.publish(angle_state)

            if force and len(force) == 6:
                force_state.id = self.device_id
                force_state.force1 = int(force[0])
                force_state.force2 = int(force[1])
                force_state.force3 = int(force[2])
                force_state.force4 = int(force[3])
                force_state.force5 = int(force[4])
                force_state.force6 = int(force[5])
                self.force_pub.publish(force_state)
                
        except Exception as e:
            # Don't spam logs with connection errors
            pass
    
    def destroy_node(self):
        """Clean shutdown."""
        if self.hand:
            try:
                self.hand.close()
            except:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    inspire_hand_pub = InspireHandPub()
    rclpy.spin(inspire_hand_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()