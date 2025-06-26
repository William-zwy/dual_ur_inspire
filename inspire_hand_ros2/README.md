# Inspire Hand ROS2 with VR Hand Tracking

This project provides ROS2 control for Inspire Hand robotic hands with **VR hand tracking integration**. Control your robotic hands naturally using VR hand gestures through Meta Quest headsets!

## 🌟 Features

- **VR Hand Tracking**: Control robotic hands using Meta Quest VR headset hand tracking
- **Real-time Control**: Live hand gesture recognition and robotic hand response  
- **Natural Retargeting**: Uses ins-dex-retarget for natural human-to-robot hand mapping
- **Dual Hand Support**: Control both left and right robotic hands simultaneously
- **ROS2 Integration**: Full ROS2 service-based communication
- **WebXR Support**: Browser-based VR interface with SSL/HTTPS support

## 🎯 System Overview

```
VR Headset (Quest) → WebXR Browser → TeleVision → ins-dex-retarget → ROS2 → Inspire Hands
```

## 📋 Prerequisites

### Hardware Requirements
- **Meta Quest 2/3/Pro** VR headset
- **Inspire Hand robotic hands** (left and/or right)
- **Linux computer** with ROS2 Humble
- **Network connection** between VR headset and computer

### Software Requirements
- **ROS2 Humble** 
- **Python 3.10+**
- **Node.js and npm** (for SSL certificates)
- **mkcert** (for SSL certificate generation)

## 🚀 Installation

### 1. Clone and Build ROS2 Package

```bash
# Clone the repository
git clone https://github.com/haoyan-ts/inspire_hand_ros2.git
cd inspire_hand_ros2

# Build the ROS2 package
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Install VR Dependencies

```bash
# Install mkcert for SSL certificates
sudo apt install libnss3-tools
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert

# Generate SSL certificates (required for WebXR)
./create_ssl_certs.sh
```

### 3. Install Python Dependencies

The TeleVision system and ins-dex-retarget dependencies are included in the `TeleVision/` and `ins-dex-retarget/` directories.

## 🎮 Usage

### Option 1: VR Hand Tracking Control (Recommended)

**Step 1: Start ROS2 Inspire Hand Services**
```bash
# Terminal 1: Start the inspire hand services
source install/setup.bash
./launch_inspire_hands.sh
```

**Step 2: Launch VR Bridge**
```bash
# Terminal 2: Start the VR hand tracking bridge
source install/setup.bash
python3 television_inspire_bridge_ultimate.py
```

**Step 3: Connect VR Headset**
1. Put on your **Meta Quest headset**
2. Open the **browser** in VR
3. Navigate to: `https://YOUR_COMPUTER_IP?ws=://YOUR_COMPUTER_IP`
   - Replace `YOUR_COMPUTER_IP` with your computer's IP address (e.g., `192.168.1.100`)
4. **Accept the SSL certificate** warning (this is expected for self-signed certificates)
5. **Enable hand tracking** in the VR interface
6. **Move your hands** - the robotic hands should follow your gestures!

### Option 2: Manual ROS2 Control

**Start Services:**
```bash
source install/setup.bash
ros2 run inspire_hand_demo inspire_hand_bringup
```

**Control Commands:**

**Open Hand:**
```bash
ros2 service call /inspire_hand_left/inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \
  "{id: 2, angle1: 1000, angle2: 1000, angle3: 1000, angle4: 1000, angle5: 1000, angle6: 1000}"
```

**Close Hand:**
```bash
ros2 service call /inspire_hand_left/inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \
  "{id: 2, angle1: 0, angle2: 0, angle3: 0, angle4: 0, angle5: 0, angle6: 0}"
```

**Pinch Gesture:**
```bash
ros2 service call /inspire_hand_left/inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \
  "{id: 2, angle1: 1000, angle2: 1000, angle3: 1000, angle4: 500, angle5: 500, angle6: 100}"
```

## 🤖 System Configuration

### Finger Mapping
- **angle1**: Little finger
- **angle2**: Ring finger  
- **angle3**: Middle finger
- **angle4**: Index finger
- **angle5**: Thumb bend
- **angle6**: Thumb yaw/rotation

### Hand IDs
- **Left Hand**: `id: 2`
- **Right Hand**: `id: 1`

### Angle Values
- **Range**: 0-1000
- **0**: Fully closed
- **1000**: Fully open

## 🔧 Troubleshooting

### VR Connection Issues

**Problem**: "Connection refused" or "Site can't be reached"
**Solution**: 
- Check your computer's IP address: `ip addr show`
- Ensure both devices are on the same network
- Try accessing `https://YOUR_IP` from VR browser first

**Problem**: "SSL Certificate Error" 
**Solution**:
- This is expected with self-signed certificates
- Click "Advanced" → "Proceed to site" in VR browser
- Make sure you generated certificates with `./create_ssl_certs.sh`

**Problem**: "VR Unsupported" message
**Solution**:
- Ensure you're using HTTPS (not HTTP)
- Enable hand tracking in Quest settings
- Try refreshing the page in VR browser

### ROS2 Issues

**Problem**: "Service not available"
**Solution**:
```bash
# Check if services are running
ros2 service list | grep inspire_hand

# Restart the inspire hand service
source install/setup.bash
./launch_inspire_hands.sh
```

**Problem**: Robotic hands not responding
**Solution**:
- Check USB/serial connections to robotic hands
- Verify hand IDs (left=2, right=1)
- Test with manual ROS2 commands first

## 📁 Project Structure

```
inspire_hand_ros2/
├── inspire_hand_demo/              # ROS2 package for hand control
├── inspire_hand_interfaces/        # ROS2 message/service definitions  
├── TeleVision/                     # VR hand tracking system
├── ins-dex-retarget/              # Hand gesture retargeting
├── television_inspire_bridge_ultimate.py  # Main VR bridge
├── launch_inspire_hands.sh        # Launch script for ROS2 services
├── create_ssl_certs.sh            # SSL certificate generation
├── run_television_bridge.sh       # Alternative launch script
└── README.md                      # This file
```

## 🎯 Advanced Usage

### Custom Hand Calibration

Edit the retargeting parameters in `ins-dex-retarget/hand_retarget.py`:

```python
# Adjust these values based on your hand size
self.four_fingers_limits = (40.0, 170.0)  # Finger bend angles
self.thumb_bending_limits = (15.0, 30.0)  # Thumb bend range
self.thumb_rotation_limits = (80.0, 150.0) # Thumb rotation range
```

### Network Configuration

For different network setups, modify the IP address in the bridge:
```python
# In television_inspire_bridge_ultimate.py
tv = OpenTeleVision(port=8080)  # Default port
```

## 🤝 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## 📄 License

This project is licensed under the MIT License. See the LICENSE file for details.

## 🙏 Acknowledgments

- **TeleVision**: VR hand tracking framework
- **ins-dex-retarget**: Hand gesture retargeting system  
- **Inspire Robotics**: Robotic hand hardware
- **ROS2**: Robot Operating System

## 📞 Support

For issues and questions:
1. Check the troubleshooting section above
2. Review existing GitHub issues
3. Create a new issue with detailed description

---

**Happy VR Hand Tracking! 🤖✋**



