# ROS Setup and Development Tools

## Installation

### Ubuntu/Debian
```bash
# Add ROS repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Windows
```powershell
# Using Chocolatey
choco install ros-humble-desktop
```

## Workspace Setup

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Initialize workspace
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Essential Tools

### Development Tools
- **colcon**: Build system for ROS 2
- **rosdep**: Dependency management
- **vcstool**: Version control for workspaces

### Visualization and Debugging
- **RViz**: 3D visualization tool
- **rqt**: Qt-based tools for ROS
- **ros2 bag**: Data recording and playback
- **ros2 topic**: Topic monitoring and publishing

### Common Commands
```bash
# Check ROS installation
ros2 --help

# List active nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# Get topic info
ros2 topic info /topic_name

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```