# ROS Perception and Sensors

## Sensor Integration

### LIDAR Integration
```python
# LIDAR data processing
import rclpy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Process LIDAR scan data
    ranges = msg.ranges
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment

    # Find closest obstacle
    min_range = min(ranges)
    min_index = ranges.index(min_range)
    angle = angle_min + min_index * angle_increment

    print(f"Closest obstacle at {min_range:.2f}m, angle {angle:.2f}rad")

# Subscribe to LIDAR topic
scan_subscriber = node.create_subscription(
    LaserScan,
    '/scan',
    scan_callback,
    10
)
```

### Camera Integration
```python
# Camera image processing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_callback(msg):
    # Convert ROS image to OpenCV
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Process image (example: edge detection)
    edges = cv2.Canny(cv_image, 100, 200)

    # Display result
    cv2.imshow("Edges", edges)
    cv2.waitKey(1)

image_subscriber = node.create_subscription(
    Image,
    '/camera/image_raw',
    image_callback,
    10
)
```

### IMU Integration
```python
# IMU data processing
from sensor_msgs.msg import Imu

def imu_callback(msg):
    # Extract orientation (quaternion)
    orientation = msg.orientation

    # Extract angular velocity
    angular_velocity = msg.angular_velocity

    # Extract linear acceleration
    linear_acceleration = msg.linear_acceleration

    # Use data for robot state estimation
    print(f"Orientation: {orientation.w:.2f}")

imu_subscriber = node.create_subscription(
    Imu,
    '/imu/data',
    imu_callback,
    10
)
```

## Computer Vision in ROS

### OpenCV Integration
```python
# OpenCV with ROS
import cv2
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/vision/processed', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Apply computer vision processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Convert back to ROS image
        ros_image = self.bridge.cv2_to_imgmsg(edges, "mono8")
        self.image_pub.publish(ros_image)
```

## Point Cloud Processing

### PCL Integration
```python
# Point cloud processing
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

def pointcloud_callback(msg):
    # Read points from PointCloud2 message
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Process point cloud data
    for point in points:
        x, y, z = point
        # Apply filtering, segmentation, etc.

pointcloud_subscriber = node.create_subscription(
    PointCloud2,
    '/camera/depth/color/points',
    pointcloud_callback,
    10
)
```

## Sensor Fusion

### Kalman Filtering
```python
# Simple Kalman filter for sensor fusion
import numpy as np

class KalmanFilter:
    def __init__(self):
        self.state = np.zeros(4)  # [x, y, vx, vy]
        self.covariance = np.eye(4) * 0.1

    def predict(self, dt):
        # Prediction step
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T

    def update(self, measurement):
        # Update step with measurement
        # Implementation depends on sensor characteristics
        pass
```

## ROS Perception Packages

- **vision_opencv**: OpenCV integration
- **image_transport**: Image compression and transport
- **camera_calibration**: Camera calibration tools
- **depth_image_proc**: Depth image processing
- **laser_filters**: LIDAR data filtering
- **imu_filter_madgwick**: IMU orientation estimation