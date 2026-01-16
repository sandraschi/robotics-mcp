# ROS Core Concepts

## Architecture Overview

ROS (Robot Operating System) is a flexible framework for writing robot software. It provides tools and libraries to help software developers create robot applications.

### Key Components

- **Nodes**: Executable processes that perform computation
- **Topics**: Named buses for data streams (publish/subscribe pattern)
- **Services**: Synchronous remote procedure calls
- **Actions**: Asynchronous remote procedure calls with feedback
- **Parameters**: Configuration values stored on Parameter Server
- **Messages**: Data structures for communication between nodes

### ROS Graph

The ROS computation graph is a peer-to-peer network of processes that are loosely coupled using the ROS communication infrastructure.

## Communication Patterns

### Topics (Publish/Subscribe)
```python
# Publisher example
import rclpy
from std_msgs.msg import String

def timer_callback():
    msg = String()
    msg.data = 'Hello World'
    publisher.publish(msg)

# Subscriber example
def listener_callback(msg):
    print('I heard: "%s"' % msg.data)
```

### Services (Request/Response)
```python
# Service server
def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    return response

# Service client
client = node.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 1
request.b = 2
future = client.call_async(request)
```

### Actions (Long-running Tasks)
Actions are used for tasks that take time to complete and provide feedback during execution.

## ROS 2 vs ROS 1

- **ROS 2**: Modern version with improved performance, real-time capabilities, and cross-platform support
- **ROS 1**: Legacy version still widely used

This project uses ROS 2 for all robotics applications.