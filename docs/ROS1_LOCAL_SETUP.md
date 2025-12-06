# Local ROS 1.4 (Melodic) Setup for Scout Development

**Goal**: Full local ROS 1.4 environment for developing and testing Scout without physical hardware.

## 🎯 Overview

This guide sets up a complete ROS 1.4 (Melodic) development environment using Docker, allowing you to:
- Build and test Scout SDK locally
- Run ROS nodes without physical hardware
- Test rosbridge communication
- Develop robotics-mcp integration
- Exercise Scout functionality before hardware arrives

## 🐳 Docker Setup (Recommended)

### Prerequisites

- Docker Desktop for Windows (with WSL2)
- Git
- PowerShell (for Windows scripts)

### Quick Start

```powershell
# Navigate to robotics-mcp
cd D:\Dev\repos\robotics-mcp

# Build and start ROS 1.4 container
docker-compose -f docker/docker-compose.ros1.yml up -d

# Setup workspace
.\docker\scripts\setup-ros1-workspace.ps1 -Build

# Enter container
docker exec -it robotics-ros1-dev bash
```

### Inside Container

```bash
# Source ROS environment
source /opt/ros/melodic/setup.bash
source /ros_ws/devel/setup.bash

# Start rosbridge (for robotics-mcp connection)
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090

# In another terminal, test Scout SDK
rosrun roller_eye <node_name>
```

## 📋 Detailed Setup

### Step 1: Build Docker Container

```powershell
cd robotics-mcp
docker-compose -f docker/docker-compose.ros1.yml build
```

### Step 2: Start Container

```powershell
docker-compose -f docker/docker-compose.ros1.yml up -d
```

### Step 3: Setup ROS Workspace

```powershell
# Automatic setup
.\docker\scripts\setup-ros1-workspace.ps1 -Build

# Or manual setup inside container
docker exec -it robotics-ros1-dev bash
# Then run: /ros_ws/setup-ros1-workspace.sh
```

### Step 4: Build Scout SDK

```bash
# Inside container
cd /ros_ws
source /opt/ros/melodic/setup.bash
cd src/roller_eye
./build.sh
cd /ros_ws
catkin_make
source devel/setup.bash
```

## 🔧 Configuration

### Scout SDK Location

The Scout SDK is mounted from:
```
external/moorebot-scout-sdk/roller_eye → /ros_ws/src/roller_eye
```

### ROS Workspace Structure

```
/ros_ws/
├── src/
│   ├── roller_eye/          # Scout SDK (mounted)
│   └── robotics_mcp/        # robotics-mcp ROS nodes (mounted)
├── build/                   # Build artifacts
└── devel/                   # Development setup files
```

### Ports

- **9090**: rosbridge WebSocket (for robotics-mcp)
- **11311**: ROS master
- **11312-11320**: ROS communication

## 🚀 Usage Examples

### Start rosbridge

```bash
# Inside container
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

### Test ROS Topics

```bash
# List topics
rostopic list

# Echo a topic
rostopic echo /CoreNode/h264

# Publish to cmd_vel (mock movement)
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Run Scout Nodes (Mock Mode)

```bash
# Start Scout nodes in mock mode
rosrun roller_eye <node_name> _mock_mode:=true
```

### Connect from robotics-mcp

```python
# In robotics-mcp, connect to rosbridge
from robotics_mcp.clients.ros_bridge import ROSBridgeClient

client = ROSBridgeClient(host="localhost", port=9090)
await client.connect()

# Subscribe to topics
await client.subscribe("/CoreNode/h264", "roller_eye/Frame")

# Publish commands
await client.publish("/cmd_vel", "geometry_msgs/Twist", {
    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
})
```

## 🧪 Testing Without Hardware

### Mock Mode

Scout SDK supports mock mode for testing without hardware:

```bash
# Set mock mode parameter
rosparam set /mock_mode true

# Or in launch file
roslaunch roller_eye start.launch mock_mode:=true
```

### Mock Data Publishers

Create mock publishers for testing:

```python
# mock_scout_publisher.py
import rospy
from sensor_msgs.msg import Range, Imu, Illuminance
from geometry_msgs.msg import Twist

rospy.init_node('mock_scout_publisher')

# Mock TOF sensor
tof_pub = rospy.Publisher('/SensorNode/tof', Range, queue_size=10)
# Mock IMU
imu_pub = rospy.Publisher('/SensorNode/imu', Imu, queue_size=10)
# Mock light sensor
light_pub = rospy.Publisher('/SensorNode/light', Illuminance, queue_size=10)

rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # Publish mock data
    # ... (implement mock data generation)
    rate.sleep()
```

## 📚 Scout SDK Development

### Build Scout SDK

```bash
cd /ros_ws/src/roller_eye
./build.sh
```

### Run Scout Nodes

```bash
# Start all nodes
roslaunch roller_eye start.launch

# Run individual nodes
rosrun roller_eye sensors_node
rosrun roller_eye motor_node
rosrun roller_eye ui_node
```

### Python API

```python
# Use Scout Python API
import sys
sys.path.append("/usr/local/lib")
from rollereye import *

rollereye.start()

# Control Scout
rollereye.set_translationSpeed(0.3)
rollereye.set_rotationSpeed(100)
rollereye.set_translate_rotate(2, 270)  # Move 2m, rotate 270°

rollereye.stop()
```

## 🔗 Integration with robotics-mcp

### Connect via rosbridge

```python
# In robotics-mcp server
from robotics_mcp.clients.ros_bridge import ROSBridgeClient

class ScoutClient:
    def __init__(self):
        self.ros = ROSBridgeClient(host="localhost", port=9090)
    
    async def connect(self):
        await self.ros.connect()
    
    async def move(self, linear, angular):
        await self.ros.publish("/cmd_vel", "geometry_msgs/Twist", {
            "linear": {"x": linear, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular}
        })
```

## 🐛 Troubleshooting

### Container won't start
- Check Docker Desktop is running
- Verify WSL2 is enabled
- Check ports 9090, 11311 are not in use

### Build fails
- Ensure Scout SDK is mounted correctly
- Check ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Verify Ubuntu 18.04 compatibility

### rosbridge connection fails
- Verify rosbridge is running: `rosnode list | grep rosbridge`
- Check port 9090 is accessible
- Test with: `curl http://localhost:9090`

### X11/GUI issues
- Set DISPLAY environment variable
- Install X11 server on Windows (VcXsrv, Xming)
- Check X11 forwarding in docker-compose.yml

## 📖 Resources

- **ROS 1.4 Melodic**: http://wiki.ros.org/melodic
- **rosbridge_suite**: http://wiki.ros.org/rosbridge_suite
- **Scout SDK**: `external/moorebot-scout-sdk/README.md`
- **Pilot Labs Binary Container**: https://github.com/Pilot-Labs-Dev/binary.git

## 🎯 Next Steps

1. ✅ Setup ROS 1.4 Docker environment
2. ✅ Build Scout SDK
3. ✅ Start rosbridge
4. ✅ Test ROS topics/services
5. ✅ Connect robotics-mcp via rosbridge
6. ✅ Develop and test Scout control
7. ✅ Ready for physical hardware!

---

**Status**: Full local ROS 1.4 environment ready for Scout development! 🚀

