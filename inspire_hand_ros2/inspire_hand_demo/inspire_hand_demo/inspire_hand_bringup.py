import rclpy
from rclpy.node import Node

from inspire_hand_interfaces.srv import SetAngle, SetSpeed, SetForce
from inspire_hand_demo.inspire_api import InspireHandAPI
from inspire_hand_interfaces.msg import AngleState, ForceState

from rclpy.qos import qos_profile_services_default
import serial.serialutil
import time

class InspireHandSrv(Node):
    def __init__(self):
        super().__init__('inspire_hand_service_server')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('max_reconnect_attempts', 5)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.auto_reconnect = self.get_parameter('auto_reconnect').get_parameter_value().bool_value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').get_parameter_value().integer_value
        
        # Initialize hand connection
        self.hand = None
        self._initialize_connection()
        
        # Create services
        self.set_angle_srv = self.create_service(
            SetAngle, 'inspire_hand_set_angle_srv', self.set_angle_srv_callback)
        self.set_speed_srv = self.create_service(
            SetSpeed, 'inspire_hand_set_speed_srv', self.set_speed_srv_callback)
        self.set_force_srv = self.create_service(
            SetForce, 'inspire_hand_set_force_srv', self.set_force_srv_callback)

        # Create publisher for current state
        self.angle_pub = self.create_publisher(
            AngleState, 'inspire_hand_angle_state', qos_profile_services_default)
        self.force_pub = self.create_publisher(
            ForceState, 'inspire_hand_force_state', qos_profile_services_default)
        
        # Create timer for state publishing
        self.state_timer = self.create_timer(0.1, self._publish_current_state)
        
        self.get_logger().info(f'Inspire Hand Service Server started on {self.serial_port}')
    
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
    
    def _ensure_connection(self):
        """Ensure hand connection is active, attempt reconnection if needed."""
        if self.hand is None or not self.hand.is_connected():
            if self.auto_reconnect:
                self.get_logger().warn('Connection lost, attempting to reconnect...')
                return self._initialize_connection()
            return False
        return True
    
    def set_angle_srv_callback(self, request, response):
        """Handle set angle service requests."""
        try:
            if not self._ensure_connection():
                response.sum = -1
                self.get_logger().error('No connection to hand device')
                return response
            
            # Extract angles, use -1 for unchanged fingers
            angles = [
                request.angle1, request.angle2, request.angle3,
                request.angle4, request.angle5, request.angle6
            ]
            
            # Validate angles (0-1000 or -1 for unchanged)
            for i, angle in enumerate(angles):
                if angle != -1 and (angle < 0 or angle > 1000):
                    self.get_logger().error(f'Invalid angle value {angle} for finger {i+1}. Must be 0-1000 or -1.')
                    response.sum = -1
                    return response
            
            # Send command to hand
            self.hand.write6(request.id, 'angleSet', angles)
            response.sum = sum(angle for angle in angles if angle != -1)
            
            self.get_logger().info(f'Set angles: {angles}')
            
        except Exception as e:
            self.get_logger().error(f'Error setting angles: {str(e)}')
            response.sum = -1
        
        return response
    
    def set_speed_srv_callback(self, request, response):
        """Handle set speed service requests."""
        try:
            if not self._ensure_connection():
                response.sum = -1
                self.get_logger().error('No connection to hand device')
                return response
            
            speeds = [
                request.speed1, request.speed2, request.speed3,
                request.speed4, request.speed5, request.speed6
            ]
            
            # Validate speeds (0-1000 or -1 for unchanged)
            for i, speed in enumerate(speeds):
                if speed != -1 and (speed < 0 or speed > 1000):
                    self.get_logger().error(f'Invalid speed value {speed} for finger {i+1}. Must be 0-1000 or -1.')
                    response.sum = -1
                    return response
            
            # Send command to hand
            self.hand.write6(request.id, 'speedSet', speeds)
            response.sum = sum(speed for speed in speeds if speed != -1)
            
            self.get_logger().info(f'Set speeds: {speeds}')
            
        except Exception as e:
            self.get_logger().error(f'Error setting speeds: {str(e)}')
            response.sum = -1
        
        return response
    
    def set_force_srv_callback(self, request, response):
        """Handle set force service requests."""
        try:
            if not self._ensure_connection():
                response.sum = -1
                self.get_logger().error('No connection to hand device')
                return response
            
            forces = [
                request.force1, request.force2, request.force3,
                request.force4, request.force5, request.force6
            ]
            
            # Validate forces (0-1000 or -1 for unchanged)
            for i, force in enumerate(forces):
                if force != -1 and (force < 0 or force > 1000):
                    self.get_logger().error(f'Invalid force value {force} for finger {i+1}. Must be 0-1000 or -1.')
                    response.sum = -1
                    return response
            
            # Send command to hand
            self.hand.write6(request.id, 'forceSet', forces)
            response.sum = sum(force for force in forces if force != -1)
            
            self.get_logger().info(f'Set forces: {forces}')
            
        except Exception as e:
            self.get_logger().error(f'Error setting forces: {str(e)}')
            response.sum = -1
        
        return response
    
    def _publish_current_state(self):
        """Publish current hand state."""
        try:
            if not self._ensure_connection():
                return
            
            # Read current angles
            angles = self.hand.read6(self.device_id, 'angleAct')
            if angles and len(angles) >= 6:
                angle_msg = AngleState()
                angle_msg.id = self.device_id
                angle_msg.angle1 = int(angles[0])
                angle_msg.angle2 = int(angles[1])
                angle_msg.angle3 = int(angles[2])
                angle_msg.angle4 = int(angles[3])
                angle_msg.angle5 = int(angles[4])
                angle_msg.angle6 = int(angles[5])
                self.angle_pub.publish(angle_msg)
            
            # Read current forces
            forces = self.hand.read6(self.device_id, 'forceAct')
            if forces and len(forces) >= 6:
                force_msg = ForceState()
                force_msg.id = self.device_id
                force_msg.force1 = int(forces[0])
                force_msg.force2 = int(forces[1])
                force_msg.force3 = int(forces[2])
                force_msg.force4 = int(forces[3])
                force_msg.force5 = int(forces[4])
                force_msg.force6 = int(forces[5])
                self.force_pub.publish(force_msg)
                
        except Exception as e:
            # Don't log every read error to avoid spam
            pass
    
    def destroy_node(self):
        """Clean shutdown."""
        if self.hand:
            try:
                self.hand.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        inspire_hand_srv = InspireHandSrv()
        rclpy.spin(inspire_hand_srv)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'inspire_hand_srv' in locals():
            inspire_hand_srv.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()