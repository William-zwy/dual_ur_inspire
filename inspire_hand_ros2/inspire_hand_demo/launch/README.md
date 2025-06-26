# Inspire Hand Launch Files

This directory contains various launch files for the Inspire Hand ROS2 package. Each launch file is designed for different use cases and scenarios.

## Available Launch Files

### 1. `inspire_hand_bringup.launch.py` (Main Launch File)
**Purpose**: Complete system launch with all nodes and configurable options.

**Features**:
- Service server for hand control
- State publisher for real-time feedback
- Diagnostics monitoring
- Configurable parameters
- Delayed node startup for proper initialization

**Usage**:
```bash
# Basic usage with default settings
ros2 launch inspire_hand_demo inspire_hand_bringup.launch.py

# Custom serial port
ros2 launch inspire_hand_demo inspire_hand_bringup.launch.py serial_port:=/dev/ttyACM0

# Disable diagnostics
ros2 launch inspire_hand_demo inspire_hand_bringup.launch.py enable_diagnostics:=false

# Custom publish rate
ros2 launch inspire_hand_demo inspire_hand_bringup.launch.py publish_rate:=20.0
```

**Parameters**:
- `serial_port`: Serial port (default: `/dev/ttyUSB0`)
- `baudrate`: Communication baud rate (default: `115200`)
- `device_id`: Hand device ID (default: `1`)
- `publish_rate`: State publishing frequency in Hz (default: `10.0`)
- `enable_diagnostics`: Enable diagnostic node (default: `true`)
- `enable_publisher`: Enable state publisher (default: `true`)
- `auto_reconnect`: Enable auto-reconnection (default: `true`)
- `max_reconnect_attempts`: Max reconnection attempts (default: `5`)

### 2. `inspire_hand_simple.launch.py` (Minimal Launch)
**Purpose**: Minimal setup with only the service server for basic control.

**Usage**:
```bash
ros2 launch inspire_hand_demo inspire_hand_simple.launch.py
```

**When to use**: When you only need basic control services without state publishing or diagnostics.

### 3. `inspire_hand_multi_device.launch.py` (Multiple Devices)
**Purpose**: Control multiple Inspire Hand devices simultaneously.

**Features**:
- Each device in its own namespace (`inspire_hand_1`, `inspire_hand_2`)
- Independent service servers and publishers
- Separate configuration for each device

**Usage**:
```bash
# Launch with default ports
ros2 launch inspire_hand_demo inspire_hand_multi_device.launch.py

# Custom ports for both devices
ros2 launch inspire_hand_demo inspire_hand_multi_device.launch.py \
    device1_port:=/dev/ttyUSB0 \
    device2_port:=/dev/ttyUSB1
```

**Service Access**:
```bash
# Control device 1
ros2 service call /inspire_hand_1/inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle "{id: 1, angle1: 500, angle2: -1, angle3: -1, angle4: -1, angle5: -1, angle6: -1}"

# Control device 2
ros2 service call /inspire_hand_2/inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle "{id: 2, angle1: 800, angle2: -1, angle3: -1, angle4: -1, angle5: -1, angle6: -1}"
```

### 4. `inspire_hand_demo.launch.py` (Demo and Tutorial)
**Purpose**: Educational launch with built-in examples and command demonstrations.

**Features**:
- Displays example commands on startup
- Optional demo sequence execution
- Educational output for learning

**Usage**:
```bash
# Launch with command examples
ros2 launch inspire_hand_demo inspire_hand_demo.launch.py

# Launch with automatic demo
ros2 launch inspire_hand_demo inspire_hand_demo.launch.py run_demo:=true
```

## Common Usage Patterns

### Basic Hand Control
```bash
# 1. Launch the system
ros2 launch inspire_hand_demo inspire_hand_bringup.launch.py

# 2. Control all fingers
ros2 service call /inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \
  "{id: 1, angle1: 1000, angle2: 1000, angle3: 1000, angle4: 1000, angle5: 1000, angle6: 1000}"

# 3. Control individual finger
ros2 service call /inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \
  "{id: 1, angle1: 500, angle2: -1, angle3: -1, angle4: -1, angle5: -1, angle6: -1}"
```

### Monitoring Hand State
```bash
# Watch angle states
ros2 topic echo /inspire_hand_angle_state

# Watch force states
ros2 topic echo /inspire_hand_force_state
```

### Setting Movement Parameters
```bash
# Set movement speeds
ros2 service call /inspire_hand_set_speed_srv inspire_hand_interfaces/srv/SetSpeed \
  "{id: 1, speed1: 200, speed2: 200, speed3: 200, speed4: 200, speed5: 200, speed6: 200}"

# Set grip forces
ros2 service call /inspire_hand_set_force_srv inspire_hand_interfaces/srv/SetForce \
  "{id: 1, force1: 500, force2: 500, force3: 500, force4: 500, force5: 500, force6: 500}"
```

## Troubleshooting

### Serial Port Issues
- Check if your serial port exists: `ls /dev/tty*`
- Ensure proper permissions: `sudo chmod 666 /dev/ttyUSB0`
- For persistent permissions, add user to dialout group: `sudo usermod -a -G dialout $USER`

### Connection Problems
- Verify baud rate matches your device (usually 115200)
- Check physical connections
- Use auto-reconnect feature for unstable connections

### Multiple Device Setup
- Ensure each device has a unique serial port
- Verify device IDs are different
- Check that all devices are properly powered

## Example Integration in Your Package

```python
# In your own launch file
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

include_inspire_hand = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('inspire_hand_demo'),
        '/launch/inspire_hand_bringup.launch.py'
    ]),
    launch_arguments={
        'serial_port': '/dev/ttyUSB0',
        'device_id': '1'
    }.items()
)
```

## Advanced Configuration

### Custom Node Names and Namespaces
You can modify the launch files to use custom namespaces for integration with larger robotic systems:

```python
# Add to any launch file
from launch_ros.actions import PushRosNamespace

# Wrap nodes in namespace
PushRosNamespace('robot_left_hand')
```

### Parameter Files
For complex configurations, consider using parameter files:

```yaml
# config/hand_params.yaml
inspire_hand_service_server:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baudrate: 115200
    device_id: 1
    auto_reconnect: true
    max_reconnect_attempts: 10
``` 