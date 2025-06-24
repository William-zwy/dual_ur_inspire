# TeleVision to Inspire Hand Bridge

This system creates a complete pipeline from VR hand tracking through TeleVision to physical Inspire Hand control via ROS2.

## System Overview

```
[VR Headset] → [TeleVision] → [ins-dex-retarget] → [ROS2] → [Inspire Hands]
```

**Components:**
1. **TeleVision**: VR hand tracking system that captures hand movements
2. **ins-dex-retarget**: Converts hand tracking data to Inspire Hand control signals
3. **ROS2 Driver**: Controls physical Inspire Hands through serial communication
4. **Bridge Script**: Connects all components together

## Hardware Requirements

- **VR Headset**: Meta Quest 3, Apple Vision Pro, or similar with hand tracking
- **Inspire Hands**: 2x Inspire Hand devices with USB serial connections
- **Computer**: Ubuntu system with ROS2 Humble
- **Network**: WiFi network for VR headset communication

## Installation

### 1. Run Setup Script

```bash
# Make setup script executable
chmod +x setup_television_bridge.sh

# Run setup (will install all dependencies)
./setup_television_bridge.sh
```

### 2. User Permissions

Add your user to the dialout group for serial access:

```bash
sudo usermod -a -G dialout $USER
# Logout and login again to apply changes
```

### 3. Connect Hardware

- Connect left Inspire Hand to `/dev/ttyUSB0`
- Connect right Inspire Hand to `/dev/ttyUSB1`
- Verify connections: `ls /dev/ttyUSB*`

## Usage

### Step 1: Launch ROS2 Hand Drivers

```bash
# Terminal 1: Start the dual hand ROS2 drivers
./launch_inspire_hands.sh
```

This starts both left and right hand ROS2 services:
- `/inspire_hand_left/inspire_hand_set_angle_srv`
- `/inspire_hand_right/inspire_hand_set_angle_srv`

### Step 2: Start the Bridge

```bash
# Terminal 2: Start the TeleVision bridge
./run_television_bridge.sh
```

This will show an ngrok URL like: `https://xxxxx.ngrok.io`

### Step 3: Connect VR Headset

1. **Copy the ngrok URL** from the terminal output
2. **Open VR browser** on your headset
3. **Navigate to the ngrok URL**
4. **Click "Enter VR"** and allow hand tracking permissions
5. **Start moving your hands!**

## System Architecture

### TeleVision Component
- Captures hand tracking data from VR headset
- Provides 25-point hand landmarks per hand
- Calculates wrist poses and finger positions
- Streams data via WebRTC or image streaming

### ins-dex-retarget Component
- Converts hand landmarks to joint angles
- Calculates finger bend angles from 3D positions
- Handles pinch detection for thumb control
- Maps human hand motion to 6-DOF Inspire Hand

### ROS2 Bridge
- Receives processed hand data at 30Hz
- Converts to ROS2 service calls
- Controls both hands independently
- Provides error handling and reconnection

## Configuration

### Hand Parameters (ins-dex-retarget)

Edit these parameters in the bridge script for your hand size:

```python
# Finger angle limits (degrees)
four_fingers_limits = (40.0, 170.0)    # Grip closed to open
thumb_bending_limits = (15.0, 30.0)     # Thumb bend range
thumb_rotation_limits = (80.0, 150.0)   # Thumb rotation range
```

### Bridge Settings

```python
# Update rate (Hz)
update_rate = 30.0

# Device IDs
left_device_id = 1
right_device_id = 2
```

## Troubleshooting

### Common Issues

**1. No USB devices found**
```bash
# Check connections
ls /dev/ttyUSB* /dev/ttyACM*

# Fix permissions
sudo chmod 666 /dev/ttyUSB*
```

**2. ROS2 services not available**
```bash
# Check if nodes are running
ros2 node list

# Check service availability
ros2 service list | grep inspire_hand
```

**3. VR connection issues**
- Ensure VR headset is on same WiFi network
- Try local IP instead of ngrok: `https://YOUR_LOCAL_IP:8012`
- Check SSL certificate installation on headset

**4. Hand tracking not working**
- Enable hand tracking in VR headset settings
- Ensure good lighting conditions
- Keep hands in view of headset cameras

### Debug Commands

```bash
# Monitor hand angles
ros2 topic echo /inspire_hand_left/inspire_hand_angle_state
ros2 topic echo /inspire_hand_right/inspire_hand_angle_state

# Test individual hand control
ros2 service call /inspire_hand_left/inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \
  "{id: 1, angle1: 500, angle2: 500, angle3: 500, angle4: 500, angle5: 500, angle6: 500}"

# Check bridge logs
ros2 node info /television_inspire_bridge
```

## Advanced Usage

### Custom Hand Mapping

Create your own retargeting by modifying the bridge script:

```python
def custom_retarget_mapping(left_landmarks, right_landmarks):
    # Your custom mapping logic here
    left_angles = your_left_mapping(left_landmarks)
    right_angles = your_right_mapping(right_landmarks)
    return left_angles, right_angles
```

### Recording and Playback

```python
# Add to bridge script for recording
import json
import time

# Record data
recorded_data = []
def record_hand_data(left_angles, right_angles):
    timestamp = time.time()
    recorded_data.append({
        'timestamp': timestamp,
        'left_angles': left_angles.tolist(),
        'right_angles': right_angles.tolist()
    })

# Save recording
with open('hand_recording.json', 'w') as f:
    json.dump(recorded_data, f)
```

### Multiple Hand Pairs

Launch additional hand pairs with different namespaces:

```bash
ros2 launch inspire_hand_demo inspire_hand_dual.launch.py \
    left_hand_port:=/dev/ttyUSB2 \
    right_hand_port:=/dev/ttyUSB3 \
    ROS_NAMESPACE:=inspire_hand_pair_2
```

## Performance Optimization

### Reduce Latency
- Increase update rate: `update_rate = 60.0`
- Use local streaming instead of ngrok
- Optimize retargeting calculations

### Improve Stability
- Add smoothing filters to angle commands
- Implement predictive control
- Add safety limits and collision detection

## File Structure

```
├── television_inspire_bridge.py      # Main bridge script
├── setup_television_bridge.sh        # Setup script
├── launch_inspire_hands.sh           # ROS2 launcher
├── run_television_bridge.sh          # Bridge launcher
├── inspire_hand_demo/
│   └── launch/
│       └── inspire_hand_dual.launch.py  # Dual hand launch file
├── TeleVision/                        # Hand tracking system
├── ins-dex-retarget/                  # Retargeting algorithms
└── inspire_hand_interfaces/           # ROS2 message definitions
```

## Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature-name`
3. Test with your hardware setup
4. Submit pull request with detailed description

## License

This project combines multiple components with different licenses:
- TeleVision: Check TeleVision/LICENSE
- ins-dex-retarget: Check ins-dex-retarget/LICENSE  
- inspire_hand_demo: ROS2 standard license

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review component-specific documentation
3. Create GitHub issue with:
   - Hardware setup description
   - Error logs and screenshots
   - Steps to reproduce the issue

## Acknowledgments

- TeleVision team for VR hand tracking system
- ins-dex-retarget developers for retargeting algorithms
- Inspire Hand team for hardware and ROS2 drivers 