---
sidebar_position: 1
title: ROS 2 Basics Tutorial
---

# ROS 2 Basics Tutorial

Learn the fundamentals of ROS 2 (Robot Operating System 2) for robotic control.

## Prerequisites

- Ubuntu 22.04 LTS installed
- Basic Python knowledge
- Understanding of command line

## Installing ROS 2

```bash
# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Setup sources
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

> Note: The commands above are for Ubuntu-based systems (recommended for ROS 2 development). On other OSes follow the ROS 2 installation guide for your platform.

## Tutorial Overview

This tutorial converts conceptual ROS 2 primitives into a runnable lab. You'll create a workspace, a Python package with publisher/subscriber, a simple service, and a launch file. We'll also demonstrate using QoS, `MultiThreadedExecutor`, and simple unit tests.

### Learning Outcomes

- Install and verify a ROS 2 environment
- Create a reproducible workspace and package
- Implement publisher/subscriber, service, and action patterns
- Use QoS profiles and executors for robust systems
- Build, run, and test with `colcon`

---

## Step 1 — Create a workspace and package (repeatable)

```bash
# create a workspace
export ROS2_WS=~/ros2_ws
mkdir -p $ROS2_WS/src && cd $ROS2_WS/src

# create ament_python package with dependencies
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy std_msgs example_interfaces

# Inspect the generated layout
tree my_robot_pkg -L 2
```

Create the package entrypoint scripts under `my_robot_pkg/my_robot_pkg/` as shown below.

---

## Step 2 — Publisher & Subscriber (with QoS)

Save as `my_robot_pkg/my_robot_pkg/publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(String, 'chatter', qos)
        self.timer = self.create_timer(0.5, self.send_message)
        self.count = 0

    def send_message(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Subscriber (save as `my_robot_pkg/my_robot_pkg/subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(String, 'chatter', self.callback, qos)

    def callback(self, msg: String):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Line notes:

- We set `QoSProfile` with `RELIABLE` to ensure messages are delivered for this tutorial's demo; replace with `BEST_EFFORT` for higher-throughput sensor streams.
- Use exact topic names; ROS 2 topic names are case-sensitive and must match across nodes.

---

## Step 3 — Simple Service (server & client)

Service server `my_robot_pkg/my_robot_pkg/add_server.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Adding {request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    node = AddServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Client `my_robot_pkg/my_robot_pkg/add_client.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = AddClient()
    res = node.send_request(5, 7)
    node.get_logger().info(f'Result: {res.sum}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 4 — Launching the demo

Create `launch/demo_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_robot_pkg', executable='publisher', name='pub'),
        Node(package='my_robot_pkg', executable='subscriber', name='sub'),
        Node(package='my_robot_pkg', executable='add_server', name='add_server'),
    ])
```

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
ros2 launch my_robot_pkg demo_launch.py
```

To call the service from a new terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

---

## Step 5 — Tests and CI

Add a unit test for the `AddServer` logic using `pytest` in `test/test_add.py` that imports the addition function (factor out logic to a pure function for testability). Use `launch_testing` for small integration tests that launch the package and assert topics/services become available.

CI suggestion (GitHub Actions): run `colcon build`, `pytest`, and `ros2 run <sanity checks>` on Ubuntu runners.

---

## Troubleshooting & Tips

- If `ros2` commands fail, ensure you sourced both ROS 2 and your workspace: `source /opt/ros/humble/setup.bash && source install/setup.bash`.
- Use `ros2 topic echo /chatter` and `ros2 topic hz /chatter` to inspect messages and frequency.
- If topics exist but no data appears, confirm matching QoS profiles between publisher and subscriber.
- Use `ros2 doctor` for environment sanity checks (available in newer ROS 2 distros).

## Exercises

1. Convert the publisher to publish `sensor_msgs/msg/Imu` with realistic timestamping. Record messages to a bag file and replay.
2. Replace the `RELIABLE` QoS with `BEST_EFFORT` and simulate packet loss using `tc qdisc` to observe message drops.
3. Add an action server that accepts a goal to compute moving average over N imu samples and return the result.

## What's next

Proceed to [Gazebo Simulation](/docs/tutorials/gazebo-simulation) to integrate these nodes with a simulated robot.

## Understanding ROS 2 Concepts

### Nodes

A **node** is a process that performs a single task. Nodes communicate with each other.

```python
# my_first_node.py
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my first ROS 2 node!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

**Topics** are named buses for nodes to exchange messages asynchronously.

```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# subscriber_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Services

**Services** are synchronous request-response interactions.

```python
# service_node.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Actions

**Actions** are asynchronous goal-based tasks with feedback.

```python
# action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class NavigateActionServer(Node):
    def __init__(self):
        super().__init__('navigate_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = NavigateToPose.Feedback()

        # Simulate navigation with feedback
        for i in range(10):
            feedback_msg.current_pose.pose.position.x = float(i) / 10.0
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigateActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a ROS 2 Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create package
ros2 pkg create --build-type ament_python my_robot_pkg

# Package structure
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   └── my_node.py
├── test/
├── launch/
├── resource/
├── package.xml
├── setup.py
├── setup.cfg
└── Colcon.toml
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py

```python
from setuptools import setup
import os

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='My first ROS 2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
        ],
    },
)
```

## Launch Files

```python
# launch/my_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='my_node',
            name='my_custom_node_name',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
    ])
```

## Running Your Package

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash

# Run a node
ros2 run my_robot_pkg my_node

# Run with launch file
ros2 launch my_robot_pkg my_launch.py

# List running nodes
ros2 node list

# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `command not found: ros2` | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| Package not found | Build package: `colcon build --packages-select my_pkg` |
| No subscriber receiving messages | Check topic names match exactly |
| Service call fails | Verify service server is running |
| Permissions denied on launch file | `chmod +x launch/my_launch.py` |

## Next Steps

- Explore [Gazebo Simulation](/docs/tutorials/gazebo-simulation)
- Learn [NVIDIA Isaac Integration](/docs/tutorials/isaac-perception)
- Review [Hardware Setup](/docs/hardware/requirements)
