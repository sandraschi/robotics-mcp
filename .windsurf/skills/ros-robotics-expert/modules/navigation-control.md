# ROS Navigation and Control

## Navigation Stack

The ROS Navigation Stack provides localization, mapping, and path planning capabilities.

### Key Components

- **move_base**: Core navigation server
- **amcl**: Adaptive Monte Carlo Localization
- **map_server**: Map loading and serving
- **global_planner**: Global path planning
- **local_planner**: Local trajectory planning
- **costmap_2d**: 2D costmap for obstacle avoidance

### Basic Navigation Setup

```python
# Simple navigation goal
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()

# Set initial pose
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.orientation.w = 1.0
navigator.setInitialPose(initial_pose)

# Set goal pose
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 0.0
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)

# Wait for navigation to complete
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f'Estimated time: {feedback.estimated_time_remaining}')

result = navigator.getResult()
if result == NavigationResult.SUCCEEDED:
    print('Navigation succeeded!')
elif result == NavigationResult.CANCELED:
    print('Navigation canceled')
elif result == NavigationResult.FAILED:
    print('Navigation failed')
```

## Control Systems

### Joint Control
```python
# Joint trajectory control
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

trajectory = JointTrajectory()
trajectory.joint_names = ['joint1', 'joint2', 'joint3']

point = JointTrajectoryPoint()
point.positions = [1.0, 0.5, 0.0]
point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()

trajectory.points.append(point)
publisher.publish(trajectory)
```

### Velocity Control
```python
# Differential drive velocity control
from geometry_msgs.msg import Twist

twist = Twist()
twist.linear.x = 0.5  # Forward velocity
twist.angular.z = 0.2  # Rotational velocity

cmd_vel_publisher.publish(twist)
```

## SLAM (Simultaneous Localization and Mapping)

ROS provides several SLAM implementations:
- **gmapping**: Laser-based SLAM
- **slam_toolbox**: Enhanced SLAM with loop closure
- **cartographer**: Google's laser-based SLAM

```bash
# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py
```

## Path Planning

### Global Planners
- **navfn**: Dijkstra's algorithm
- **global_planner**: A* algorithm
- **sbpl**: Search-based planning library

### Local Planners
- **dwa_local_planner**: Dynamic Window Approach
- **base_local_planner**: Trajectory rollout
- **teb_local_planner**: Timed Elastic Band