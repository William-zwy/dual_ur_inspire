#!/usr/bin/env python3
"""
ULTIMATE TeleVision to Inspire Hand Bridge
Combines:
- Working VR hand tracking from television_inspire_bridge_final.py
- Proper ins-dex-retarget control from television_inspire_bridge_https.py
"""

import math
import numpy as np
import sys
import os
import time
from pathlib import Path
import yaml
from multiprocessing import Array, Process, shared_memory, Queue, Manager, Event, Semaphore
import threading

# Add TeleVision to path
sys.path.append('./TeleVision/teleop')
sys.path.append('./ins-dex-retarget')

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices

# ins-dex-retarget imports
from hand_retarget import HandRetarget

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from inspire_hand_interfaces.srv import SetAngle
from geometry_msgs.msg import PoseStamped

from pytransform3d import rotations

print("🎯 ULTIMATE TELEVISION TO INSPIRE HAND BRIDGE")
print("=" * 60)

class VuerTeleopUltimate:
    def __init__(self, config_file_path):
        print("✅ Initializing VuerTeleop with EXACT working pattern...")
        
        # EXACT same configuration as working test
        self.resolution = (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (
            self.resolution[0] - self.crop_size_h,
            self.resolution[1] - 2 * self.crop_size_w
        )
        
        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]
        
        # Create shared memory for images (CRITICAL for TeleVision)
        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm.buf)
        
        # Fill with dummy stereo images
        left_img = np.random.randint(0, 255, (self.img_height, self.img_width, 3), dtype=np.uint8)
        right_img = np.random.randint(0, 255, (self.img_height, self.img_width, 3), dtype=np.uint8)
        self.img_array[:] = np.hstack((left_img, right_img))
        
        image_queue = Queue()
        toggle_streaming = Event()
        
        # EXACT same OpenTeleVision initialization (DEFAULT parameters - this is KEY!)
        print("🚀 Creating OpenTeleVision with DEFAULT parameters...")
        # self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming)
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming, ngrok=True)
        
        # Initialize preprocessor
        self.processor = VuerPreprocessor()
        
        # Initialize hand retargeting (THE KEY ADDITION!)
        print("🔧 Initializing ins-dex-retarget...")
        self.hand_retarget = HandRetarget()
        
        # Start continuous image update thread (CRITICAL)
        self.image_thread_running = True
        self.image_thread = threading.Thread(target=self.update_images_continuously, daemon=True)
        self.image_thread.start()
        
        print("✅ VuerTeleop initialization complete!")
        
    def update_images_continuously(self):
        """Continuously update the image array (required for TeleVision to work)."""
        while self.image_thread_running:
            try:
                # Create new random stereo images each frame
                left_img = np.random.randint(0, 255, (self.img_height, self.img_width, 3), dtype=np.uint8)
                right_img = np.random.randint(0, 255, (self.img_height, self.img_width, 3), dtype=np.uint8)
                
                # Update the shared image array (following teleop_hand.py pattern)
                np.copyto(self.img_array, np.hstack((left_img, right_img)))
                time.sleep(1.0/60.0)  # 60 FPS
            except Exception as e:
                print(f"Error updating images: {e}")
                time.sleep(0.1)
    
    def calculate_pinch_distance(self, finger_landmarks):
        """Calculate distance between thumb tip and index finger tip."""
        if finger_landmarks is None or len(finger_landmarks) < 25:
            return 0.1  # Default distance
            
        # Thumb tip is index 4, index finger tip is index 8 in the landmarks
        thumb_tip = finger_landmarks[4]
        index_tip = finger_landmarks[8]
        
        distance = np.linalg.norm(thumb_tip - index_tip)
        return distance
    
    def create_finger_frames_from_landmarks(self, landmarks, wrist_mat):
        """Convert landmarks to finger frames format expected by retargeting."""
        if landmarks is None or len(landmarks) < 25:
            # Return zero frames if no valid data
            return np.zeros((25, 4, 4))
        
        frames = np.zeros((25, 4, 4))
        
        # Set identity matrices as base
        for i in range(25):
            frames[i] = np.eye(4)
            if i < len(landmarks):
                frames[i][:3, 3] = landmarks[i]
        
        return frames
    
    def step(self):
        """Main step function - processes hand data and returns retargeted angles."""
        try:
            # Get preprocessed hand data (following teleop_hand.py pattern)
            # head_mat, left_wrist_mat, right_wrist_mat, left_hand_landmarks, right_hand_landmarks = self.processor.process(self.tv)
            left_wrist_mat, right_wrist_mat, left_hand_landmarks, right_hand_landmarks = self.processor.process_for_controller(self.tv)
            
            # Check if we have valid hand data
            landmarks_valid = False
            if (hasattr(self.tv, 'left_landmarks') and hasattr(self.tv, 'right_landmarks')):
                left_nonzero = np.any(self.tv.left_landmarks != 0)
                right_nonzero = np.any(self.tv.right_landmarks != 0)
                landmarks_valid = left_nonzero or right_nonzero
            
            if landmarks_valid:
                # Calculate pinch distances
                left_pinch_distance = self.calculate_pinch_distance(left_hand_landmarks)
                right_pinch_distance = self.calculate_pinch_distance(right_hand_landmarks)
                
                # Create finger frames for retargeting
                left_finger_frames = self.create_finger_frames_from_landmarks(left_hand_landmarks, left_wrist_mat)
                right_finger_frames = self.create_finger_frames_from_landmarks(right_hand_landmarks, right_wrist_mat)
                
                # Create data structure expected by hand_retarget
                retarget_data = {
                    "left_fingers": left_finger_frames,
                    "right_fingers": right_finger_frames,
                    "left_pinch_distance": left_pinch_distance,
                    "right_pinch_distance": right_pinch_distance
                }
                
                # Get retargeted angles using ins-dex-retarget
                left_angles, right_angles = self.hand_retarget.solve_fingers_angles(retarget_data)
                
                return {
                    'valid': True,
                    'left_angles': left_angles,
                    'right_angles': right_angles,
                    'left_landmarks': self.tv.left_landmarks,
                    'right_landmarks': self.tv.right_landmarks,
                    'left_hand_landmarks': left_hand_landmarks,
                    'right_hand_landmarks': right_hand_landmarks,
                    'left_wrist_mat':left_wrist_mat,
                    'right_wrist_mat':right_wrist_mat
                }
            else:
                return {
                    'valid': False,
                    'left_angles': [500, 500, 500, 500, 500, 500],  # Neutral position
                    'right_angles': [500, 500, 500, 500, 500, 500],
                    'left_landmarks': self.tv.left_landmarks if hasattr(self.tv, 'left_landmarks') else np.zeros((25, 3)),
                    'right_landmarks': self.tv.right_landmarks if hasattr(self.tv, 'right_landmarks') else np.zeros((25, 3)),
                    'left_wrist_mat': np.eye(4),
                    'right_wrist_mat': np.eye(4)
                }
                
        except Exception as e:
            print(f"Error in step: {e}")
            return {
                'valid': False,
                'left_angles': [500, 500, 500, 500, 500, 500],
                'right_angles': [500, 500, 500, 500, 500, 500],
                'left_landmarks': np.zeros((25, 3)),
                'right_landmarks': np.zeros((25, 3)),
                'left_wrist_mat': np.eye(4),
                'right_wrist_mat': np.eye(4)
            }
    
    def cleanup(self):
        """Clean up resources."""
        try:
            self.image_thread_running = False
            if hasattr(self, 'image_thread'):
                self.image_thread.join(timeout=1.0)
            
            if hasattr(self, 'shm'):
                self.shm.close()
                self.shm.unlink()
        except Exception as e:
            print(f"Error during cleanup: {e}")


class InspireHandController(Node):
    def __init__(self):
        super().__init__('inspire_hand_controller')
        
        # ROS2 service clients for controlling inspire hands
        self.left_hand_client = self.create_client(SetAngle, '/inspire_hand_left/inspire_hand_set_angle_srv')
        self.right_hand_client = self.create_client(SetAngle, '/inspire_hand_right/inspire_hand_set_angle_srv')
        
        self.left_wrist_pose_publisher = self.create_publisher(PoseStamped,"/left/wrist_pose",qos_profile_sensor_data)
        self.right_wrist_pose_publisher = self.create_publisher(PoseStamped,"/right/wrist_pose",qos_profile_sensor_data)
        
        # Wait for services to be available
        print("🔌 Waiting for inspire hand services...")
        self.left_hand_client.wait_for_service(timeout_sec=10.0)
        self.right_hand_client.wait_for_service(timeout_sec=10.0)
        print("✅ Inspire hand services are available!")
        
        # Control parameters
        self.left_device_id = 2
        self.right_device_id = 1
    
    def send_hand_commands(self, left_angles, right_angles):
        """Send angle commands to both inspire hands."""
        try:
            # DEBUG: Print the raw retargeted angles
            # print(f"🔧 DEBUG - Raw retargeted angles:")
            # print(f"   Left:  {left_angles}")
            # print(f"   Right: {right_angles}")
            
            # Create service requests
            # CORRECTED MAPPING:
            # ins-dex-retarget outputs: [little, ring, middle, index, thumb_bend, thumb_rotation]
            # inspire hand expects: [angle1, angle2, angle3, angle4, angle5, angle6]
            # Need to figure out the correct mapping based on user feedback
            left_request = SetAngle.Request()
            left_request.id = 2  # Left hand device ID
            left_request.angle1 = int(left_angles[0])  # angle1 = little finger (from retarget[0])
            left_request.angle2 = int(left_angles[1])  # angle2 = ring finger (from retarget[1])
            left_request.angle3 = int(left_angles[2])  # angle3 = middle finger (from retarget[2])
            left_request.angle4 = int(left_angles[3])  # angle4 = index finger (from retarget[3])
            left_request.angle5 = int(left_angles[4])  # angle5 = thumb bend (from retarget[4])
            left_request.angle6 = int(left_angles[5])  # angle6 = thumb rotation (from retarget[5])
            
            right_request = SetAngle.Request()
            right_request.id = 1  # Right hand device ID
            right_request.angle1 = int(right_angles[0])  # angle1 = little finger (from retarget[0])
            right_request.angle2 = int(right_angles[1])  # angle2 = ring finger (from retarget[1])
            right_request.angle3 = int(right_angles[2])  # angle3 = middle finger (from retarget[2])
            right_request.angle4 = int(right_angles[3])  # angle4 = index finger (from retarget[3])
            right_request.angle5 = int(right_angles[4])  # angle5 = thumb bend (from retarget[4])
            right_request.angle6 = int(right_angles[5])  # angle6 = thumb rotation (from retarget[5])
            
            # DEBUG: Print the exact service requests being sent
            # print(f"🚀 DEBUG - Service requests:")
            # print(f"   LEFT:  id={left_request.id}, angles=[{left_request.angle1}, {left_request.angle2}, {left_request.angle3}, {left_request.angle4}, {left_request.angle5}, {left_request.angle6}]")
            # print(f"   RIGHT: id={right_request.id}, angles=[{right_request.angle1}, {right_request.angle2}, {right_request.angle3}, {right_request.angle4}, {right_request.angle5}, {right_request.angle6}]")
            
            # Send async requests (non-blocking)
            left_future = self.left_hand_client.call_async(left_request)
            right_future = self.right_hand_client.call_async(right_request)
            
            print(f"📡 DEBUG - Service calls sent!")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error sending hand commands: {str(e)}')
            print(f"❌ ERROR in send_hand_commands: {e}")
            return False
        
    def send_wrist_pose(self, left_wrist_mat, right_wrist_mat):
        
        left_msg = PoseStamped()
        right_msg = PoseStamped()
        
        left_msg = self.wrist_pose_to_msg(left_wrist_mat)
        right_msg = self.wrist_pose_to_msg(right_wrist_mat)
        
        self.left_wrist_pose_publisher.publish(left_msg)
        self.right_wrist_pose_publisher.publish(right_msg)
    
    def wrist_pose_to_msg(self, pose_mat):
        
        msg = PoseStamped()
        
        pose = np.concatenate([pose_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
                            rotations.quaternion_from_matrix(pose_mat[:3, :3])[[1, 2, 3, 0]]])
        
        pose[0] = self.clamp(pose[0],-0.6,-0.05)
        pose[2] = self.clamp(pose[2],0.4,1.5)
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        
        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]
        
        return msg
    
    def clamp(self, x, lower_limit, upper_limit):
        if lower_limit > upper_limit:
            lower_limit,upper_limit = upper_limit,lower_limit
        return max(lower_limit, min(x,upper_limit))
        


def main():
    print("🚀 Starting Ultimate TeleVision Inspire Bridge...")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create inspire hand controller
    hand_controller = InspireHandController()
    
    # Create VuerTeleop with exact working pattern
    config_path = Path("./TeleVision/teleop/inspire_hand.yml")
    teleoperator = VuerTeleopUltimate(config_path)
    
    print("\n🌐 TeleVision running with DEFAULT settings")
    print("📱 It will show vuer.ai URL, but access via: https://10.60.72.189?ws=://10.60.72.189")
    print("🎯 This is the SAME URL pattern that worked before!")
    print("👋 Move your hands in VR to control Inspire robotic hands")
    print("🛑 Press Ctrl+C to stop")
    print("=" * 60)
    
    frame_count = 0
    last_angles_left = None
    last_angles_right = None
    
    try:
        while True:
            # Get hand data and retargeted angles
            result = teleoperator.step()
            
            frame_count += 1
            
            # Print status every 60 frames (1 second at 60 FPS)
            if frame_count % 60 == 0:
                print(f"\n📊 Frame {frame_count}:")
                if result['valid']:
                    print("🎉 SUCCESS! Hand tracking data received!")
                    print("🤖 Controlling Inspire robotic hands with ins-dex-retarget...")
                    
                    # Check if landmarks changed
                    left_changed = np.any(result['left_landmarks'] != 0)
                    right_changed = np.any(result['right_landmarks'] != 0)
                    
                    if left_changed:
                        print("🔄 LEFT hand landmarks changed!")
                        print(f"👈 Left landmarks sample: {result['left_landmarks'][:3]}")
                        
                    if right_changed:
                        print("🔄 RIGHT hand landmarks changed!")
                        print(f"👉 Right landmarks sample: {result['right_landmarks'][:3]}")
                    
                    print(f"🎮 Left retargeted angles: {result['left_angles']}")
                    print(f"🎮 Right retargeted angles: {result['right_angles']}")
                    
                else:
                    print("⚠️  All landmarks are zeros - WebXR not sending data")
                    print(f"👈 Left landmarks: {result['left_landmarks'][:3]}")
                    print(f"👉 Right landmarks: {result['right_landmarks'][:3]}")
            
            # Send commands to inspire hands (every frame for responsiveness)
            if result['valid']:
                hand_controller.send_wrist_pose(result['left_wrist_mat'], result['right_wrist_mat'])
                # Only send if angles changed significantly (to reduce ROS2 traffic)
                angle_threshold = 10  # Only send if change > 10 units
                
                send_left = True
                send_right = True
                
                if last_angles_left is not None:
                    left_diff = np.abs(np.array(result['left_angles']) - np.array(last_angles_left))
                    send_left = np.any(left_diff > angle_threshold)
                    if frame_count % 300 == 0:  # Debug every 5 seconds
                        print(f"🔍 DEBUG - Left angle change check:")
                        # print(f"   Current: {result['left_angles']}")
                        # print(f"   Last:    {last_angles_left}")
                        # print(f"   Diff:    {left_diff}")
                        # print(f"   Send:    {send_left}")
                
                if last_angles_right is not None:
                    right_diff = np.abs(np.array(result['right_angles']) - np.array(last_angles_right))
                    send_right = np.any(right_diff > angle_threshold)
                    if frame_count % 300 == 0:  # Debug every 5 seconds
                        print(f"🔍 DEBUG - Right angle change check:")
                        # print(f"   Current: {result['right_angles']}")
                        # print(f"   Last:    {last_angles_right}")
                        # print(f"   Diff:    {right_diff}")
                        # print(f"   Send:    {send_right}")
                
                if send_left or send_right:
                    # print(f"🎯 SENDING COMMANDS - Left: {send_left}, Right: {send_right}")
                    success = hand_controller.send_hand_commands(result['left_angles'], result['right_angles'])
                    if success:
                        last_angles_left = result['left_angles'].copy()
                        last_angles_right = result['right_angles'].copy()
                        print(f"✅ Commands sent successfully!")
                    else:
                        print(f"❌ Failed to send commands!")
                else:
                    if frame_count % 300 == 0:  # Debug every 5 seconds
                        print(f"⏸️  No significant angle changes - not sending commands")                        
            
            # Process ROS2 callbacks
            rclpy.spin_once(hand_controller, timeout_sec=0.001)
            
            # Small sleep to prevent excessive CPU usage
            time.sleep(1.0/60.0)  # 60 FPS
            
    except KeyboardInterrupt:
        print("\n🛑 Bridge stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        print("🧹 Cleaning up...")
        teleoperator.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 