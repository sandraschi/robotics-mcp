#!/bin/bash
# Setup ROS 1.4 workspace for Scout development

set -e

echo "🔧 Setting up ROS 1.4 (Melodic) workspace for Scout..."

# Source ROS
source /opt/ros/melodic/setup.bash

# Navigate to workspace
cd /ros_ws

# Initialize workspace if needed
if [ ! -f "src/CMakeLists.txt" ]; then
    echo "📦 Initializing catkin workspace..."
    catkin_init_workspace src
fi

# Copy Scout SDK if not already present
if [ ! -d "src/roller_eye" ]; then
    echo "📥 Copying Scout SDK..."
    if [ -d "/ros_ws/src/roller_eye" ]; then
        echo "✅ Scout SDK already mounted"
    else
        echo "⚠️  Scout SDK not found. Please mount it in docker-compose.yml"
    fi
fi

# Install dependencies
echo "📚 Installing dependencies..."
rosdep update
if [ -d "src/roller_eye" ]; then
    rosdep install --from-paths src --ignore-src -r -y || true
fi

# Build workspace
echo "🔨 Building workspace..."
catkin_make

# Source workspace
source devel/setup.bash

echo "✅ ROS 1.4 workspace setup complete!"
echo ""
echo "Next steps:"
echo "  1. Start rosbridge: roslaunch rosbridge_server rosbridge_websocket.launch port:=9090"
echo "  2. Test Scout SDK: rosrun roller_eye <node_name>"
echo "  3. Use robotics-mcp to connect via rosbridge"

