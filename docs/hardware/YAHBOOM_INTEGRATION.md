# Yahboom Robotics Integration Guide

## Overview

This guide covers integrating Yahboom robotics platforms into the robotics-mcp server. Yahboom offers a comprehensive range of ROS2-based robots that are ideal for expanding the MCP server's capabilities beyond the current Moorebot Scout integration.

## Supported Yahboom Platforms

### ROSMASTER Series (Recommended Starting Point)

#### ROSMASTER M1 - Multimodal AI Robot Car
- **Price**: €282 (~$300)
- **Perfect for**: AI research, autonomous navigation, multimodal applications
- **Key Features**:
  - Raspberry Pi 5 (8GB RAM)
  - Mecanum wheels (omni-directional)
  - Multimodal AI (vision + voice + text)
  - ROS2 Humble + Large Language Models
  - SLAM navigation + autonomous driving
  - Depth camera + LiDAR

### DOGZILLA Series - Quadruped Robots

#### DOGZILLA S1/S2 - 12DOF Robot Dog
- **Price**: $660
- **Features**: Advanced gait control, AI integration, ROS2
- **Use Case**: Quadruped locomotion research, AI-powered behaviors

### DOFBOT Series - Robotic Arms

#### DOFBOT Pro - AI Vision Robotic Arm
- **Price**: $499
- **Features**: 3D depth vision, AI recognition, ROS2 manipulation
- **Use Case**: Pick & place, industrial automation

## Integration Architecture

### ROS2 Bridge Pattern (Recommended)

```python
# src/robotics_mcp/tools/yahboom_bridge.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from yahboom_sdk import YahboomRobot

class YahboomROS2Bridge(Node):
    def __init__(self):
        super().__init__('yahboom_bridge')

        # Initialize Yahboom robot
        self.robot = YahboomRobot(model='rosmaster_m1')

        # ROS2 publishers/subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/yahboom/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/yahboom/camera', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/yahboom/scan', 10)

        # Control subscribers
        self.create_subscription(Twist, '/yahboom/cmd_vel', self.cmd_vel_callback, 10)

        # Timer for sensor publishing
        self.timer = self.create_timer(0.1, self.publish_sensors)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.robot.move(linear_x, angular_z)

    def publish_sensors(self):
        """Publish sensor data"""
        # Camera
        image = self.robot.get_camera_image()
        if image:
            self.image_pub.publish(image)

        # LiDAR
        scan = self.robot.get_lidar_scan()
        if scan:
            self.scan_pub.publish(scan)
```

### MCP Tool Integration

```python
# src/robotics_mcp/tools/yahboom_tools.py
from mcp import Tool
from yahboom_sdk import YahboomRobot, YahboomAI

class YahboomTools:
    def __init__(self):
        self.robot = YahboomRobot()
        self.ai = YahboomAI()

    @Tool()
    async def yahboom_status(self) -> dict:
        """Get Yahboom robot status"""
        return {
            "connected": self.robot.is_connected(),
            "battery": self.robot.get_battery_level(),
            "position": self.robot.get_position(),
            "ai_status": self.ai.get_status()
        }

    @Tool()
    async def yahboom_move(self, direction: str, distance: float = 1.0) -> str:
        """Move Yahboom robot"""
        if direction == "forward":
            await self.robot.move_forward(distance)
        elif direction == "backward":
            await self.robot.move_backward(distance)
        elif direction == "left":
            await self.robot.turn_left(90)
        elif direction == "right":
            await self.robot.turn_right(90)

        return f"Robot moved {direction} by {distance} units"

    @Tool()
    async def yahboom_ai_query(self, query: str) -> str:
        """Query Yahboom's AI system"""
        return await self.ai.process_query(query)

    @Tool()
    async def yahboom_navigate(self, x: float, y: float) -> bool:
        """Navigate to coordinates"""
        return await self.robot.navigate_to(x, y)

    @Tool()
    async def yahboom_analyze_scene(self) -> dict:
        """Analyze current scene with multimodal AI"""
        visual = await self.robot.get_camera_image()
        spatial = await self.robot.get_lidar_scan()
        audio = await self.robot.get_audio_sample()

        analysis = await self.ai.analyze_multimodal({
            "visual": visual,
            "spatial": spatial,
            "audio": audio
        })

        return {
            "objects": analysis["objects"],
            "layout": analysis["layout"],
            "confidence": analysis["confidence"]
        }
```

## Setup Instructions

### 1. Hardware Requirements

#### ROSMASTER M1 Setup
```bash
# Required components:
# - ROSMASTER M1 robot (€282)
# - Raspberry Pi 5 (8GB) - optional, can be purchased separately
# - WiFi network for communication
```

### 2. Software Dependencies

```bash
# Install ROS2 Humble on Ubuntu 22.04
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete

# Install Yahboom packages
pip install yahboom-sdk yahboom-ai yahboom-ros2
```

### 3. Configuration

```yaml
# config/yahboom_config.yaml
yahboom:
  model: "rosmaster_m1"
  connection:
    ip: "192.168.1.100"  # Robot's IP address
    port: 9090
  ros2:
    namespace: "/yahboom"
    domain_id: 42
  ai:
    multimodal_enabled: true
    llm_provider: "openai"  # or "iFlytek"
```

### 4. Integration with Existing Tools

```python
# src/robotics_mcp/server.py - Add to existing server
from .tools.yahboom_tools import YahboomTools

class RoboticsMCPServer:
    def __init__(self):
        # Existing tools...
        self.moorebot_tools = MoorebotTools()
        self.virtual_tools = VirtualTools()

        # Add Yahboom tools
        self.yahboom_tools = YahboomTools()

    async def setup_tools(self):
        # Register Yahboom tools
        tools = [
            # Existing tools...
            self.moorebot_tools.get_status,
            self.moorebot_tools.move_robot,

            # Yahboom tools
            self.yahboom_tools.yahboom_status,
            self.yahboom_tools.yahboom_move,
            self.yahboom_tools.yahboom_ai_query,
            self.yahboom_tools.yahboom_navigate,
            self.yahboom_tools.yahboom_analyze_scene,
        ]
        return tools
```

## ROS2 Launch Integration

### Yahboom ROS2 Launch File
```xml
<!-- launch/yahboom.launch.py -->
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    robot_ip = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.100',
        description='Yahboom robot IP address'
    )

    # Yahboom bridge node
    yahboom_bridge = Node(
        package='yahboom_bridge',
        executable='yahboom_bridge_node',
        name='yahboom_bridge',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_sim_time': False
        }]
    )

    # Navigation stack
    nav2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': '/path/to/yahboom_nav_params.yaml'
        }.items()
    )

    return LaunchDescription([
        robot_ip,
        yahboom_bridge,
        nav2
    ])
```

## Multimodal AI Integration

### AI Workflow Integration
```python
# Integrate with existing LLM workflows
@Tool()
async def multimodal_robot_task(self, task_description: str) -> dict:
    """Execute complex task using Yahboom's multimodal AI"""

    # Get current scene analysis
    scene = await self.yahboom_tools.yahboom_analyze_scene()

    # Use LLM to plan task
    llm_plan = await self.llm_service.generate_plan(
        task_description, scene
    )

    # Execute plan with Yahboom robot
    result = await self.yahboom_tools.execute_plan(llm_plan)

    return {
        "task": task_description,
        "scene_analysis": scene,
        "plan": llm_plan,
        "execution_result": result
    }
```

## Testing and Validation

### Unit Tests
```python
# tests/unit/test_yahboom_tools.py
import pytest
from unittest.mock import AsyncMock, MagicMock
from robotics_mcp.tools.yahboom_tools import YahboomTools

class TestYahboomTools:
    @pytest.fixture
    def yahboom_tools(self):
        tools = YahboomTools()
        tools.robot = AsyncMock()
        tools.ai = AsyncMock()
        return tools

    @pytest.mark.asyncio
    async def test_yahboom_status(self, yahboom_tools):
        yahboom_tools.robot.is_connected.return_value = True
        yahboom_tools.robot.get_battery_level.return_value = 85

        status = await yahboom_tools.yahboom_status()

        assert status["connected"] is True
        assert status["battery"] == 85

    @pytest.mark.asyncio
    async def test_yahboom_move(self, yahboom_tools):
        result = await yahboom_tools.yahboom_move("forward", 2.0)

        yahboom_tools.robot.move_forward.assert_called_once_with(2.0)
        assert "forward" in result
```

### Integration Tests
```python
# tests/integration/test_yahboom_integration.py
import pytest
import rclpy
from robotics_mcp.tools.yahboom_bridge import YahboomROS2Bridge

class TestYahboomIntegration:
    @pytest.fixture
    def ros_context(self):
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_ros2_bridge_initialization(self, ros_context):
        bridge = YahboomROS2Bridge()
        assert bridge.robot is not None
        assert bridge.ai is not None

    def test_sensor_publishing(self, ros_context):
        # Test that sensors publish data correctly
        pass

    def test_command_execution(self, ros_context):
        # Test that ROS2 commands are executed on robot
        pass
```

## Deployment

### Docker Integration
```dockerfile
# docker/Dockerfile.yahboom
FROM ros:humble-ros-base

# Install Yahboom dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-navigation2 \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Yahboom packages
RUN pip3 install yahboom-sdk yahboom-ai

# Copy application
COPY . /app
WORKDIR /app

# Run with ROS2
CMD ["ros2", "launch", "yahboom_bridge", "yahboom.launch.py"]
```

### Docker Compose
```yaml
# docker/docker-compose.yahboom.yml
version: '3.8'
services:
  yahboom-bridge:
    build:
      context: ..
      dockerfile: docker/Dockerfile.yahboom
    environment:
      - ROBOT_IP=192.168.1.100
      - ROS_DOMAIN_ID=42
    networks:
      - robotics-network

  robotics-mcp:
    depends_on:
      - yahboom-bridge
    environment:
      - YAHBOOM_BRIDGE_IP=yahboom-bridge
    networks:
      - robotics-network

networks:
  robotics-network:
    driver: bridge
```

## Troubleshooting

### Common Issues

#### 1. Connection Problems
```bash
# Check network connectivity
ping <robot_ip>

# Verify ROS2 communication
ros2 topic list | grep yahboom

# Check Yahboom SDK logs
tail -f /var/log/yahboom/sdk.log
```

#### 2. ROS2 Communication Issues
```bash
# Set correct domain ID
export ROS_DOMAIN_ID=42

# Check ROS2 nodes
ros2 node list

# Debug topic communication
ros2 topic echo /yahboom/cmd_vel
```

#### 3. AI Integration Issues
```bash
# Check AI service status
ros2 service call /yahboom/ai/status

# Verify multimodal capabilities
ros2 topic echo /yahboom/ai/analysis
```

## Performance Optimization

### Real-time Settings
```bash
# Enable real-time kernel (if available)
sudo apt install linux-lowlatency

# Configure CPU isolation for ROS2
echo "isolcpus=1-3" >> /boot/cmdline.txt

# Set ROS2 thread priorities
export ROS2_SCHEDULER=RPS
```

### AI Model Optimization
```python
# Use TensorRT for faster inference
from yahboom_ai import optimize_model

# Convert to optimized format
optimized_model = optimize_model('yolov8n.pt', 'tensorrt')
optimized_model.save('/opt/yahboom/models/yolov8n.engine')
```

## Security Considerations

### Network Security
```yaml
# Secure configuration
yahboom:
  security:
    authentication: true
    encryption: true
    allowed_ips: ["192.168.1.0/24"]
  monitoring:
    log_level: "INFO"
    audit_trail: true
```

### Access Control
```python
# Implement access control for MCP tools
class SecureYahboomTools(YahboomTools):
    def __init__(self, access_control):
        super().__init__()
        self.access_control = access_control

    async def check_permission(self, user, action):
        return await self.access_control.check(user, action)

    @Tool()
    async def secure_yahboom_move(self, user: str, direction: str, distance: float) -> str:
        if not await self.check_permission(user, "move_robot"):
            raise PermissionError("Insufficient permissions")

        return await self.yahboom_move(direction, distance)
```

## Future Expansion

### Multi-Robot Coordination
```python
# Swarm coordination with multiple Yahboom robots
class YahboomSwarm:
    def __init__(self, robots):
        self.robots = robots
        self.coordinator = SwarmCoordinator()

    async def coordinate_task(self, task):
        # Distribute task across robots
        assignments = self.coordinator.assign_tasks(task, self.robots)

        # Execute coordinated actions
        results = await asyncio.gather(*[
            robot.execute_task(assignment)
            for robot, assignment in zip(self.robots, assignments)
        ])

        return results
```

This integration guide provides a complete pathway for adding Yahboom robotics platforms to the robotics-mcp server, expanding capabilities beyond the current Moorebot Scout integration with modern ROS2 and multimodal AI features.