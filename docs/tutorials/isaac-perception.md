---
sidebar_position: 3
title: NVIDIA Isaac Perception Tutorial
---

# NVIDIA Isaac Perception Tutorial

Build AI-powered perception pipelines using NVIDIA Isaac Sim and Isaac ROS.

## Prerequisites

- RTX GPU (12GB+ VRAM recommended)
- Ubuntu 22.04 LTS
- CUDA 12.2+ installed
- ROS 2 Humble installed

## Installing NVIDIA Isaac

### Isaac Sim (via Omniverse)

1. Download **NVIDIA Omniverse Launcher** from NVIDIA website
2. Install Isaac Sim from the Launcher
3. Verify installation:

```bash
# Test Isaac Sim
cd ~/.local/share/ov/pkg/isaac_sim-*/bob
./python.sh -c "print('Isaac Sim installed successfully')"
```

### Isaac ROS Packages

```bash
# Install Isaac ROS dependencies
sudo apt install -y \
    cmake \
    g++ \
    git \
    libeigen3-dev \
    libopencv-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep

# Install Isaac ROS packages
sudo apt install -y \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-image-proc \
    ros-humble-isaac-ros-dnn-vision \
    ros-humble-isaac-ros-visual-slam

# Install Isaac ROS NITROS
sudo apt install -y \
    ros-humble-isaac-ros-nitros \
    ros-humble-isaac-ros-nitros-image-type

# Install VSLAM dependencies
sudo apt install -y \
    librealsense2-dev \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description
```

## Isaac ROS Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Isaac ROS Pipeline                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────────┐   │
│  │ Camera   │───▶│ Image Pre-   │───▶│ DNN Inference   │   │
│  │ Input    │    │ processing   │    │ (TensorRT)      │   │
│  └──────────┘    └──────────────┘    └─────────────────┘   │
│                                               │             │
│                                               ▼             │
│                                        ┌─────────────┐     │
│                                        │   Output    │     │
│                                        │   (Boxes,   │     │
│                                        │  Segments)  │     │
│                                        └─────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

## VSLAM Implementation

### Visual SLAM with Isaac ROS

```python
# vlam_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class VSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_vlam_node')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)

        # Isaac ROS VSLAM parameters
        self.declare_parameters(
            '',
            [
                ('vocab_path', '/path/to/ORBvoc.txt'),
                ('params_path', '/path/to/isaac_params.yaml'),
            ]
        )

        # Initialize VSLAM (placeholder for actual API)
        self.vslam = self._init_vslam()

    def _init_vslam(self):
        # In production, use Isaac ROS VSLAM API
        self.get_logger().info('Initializing VSLAM with Isaac ROS')
        return True

    def image_callback(self, msg):
        # Process image for VSLAM
        self.get_logger().info(f'Processing frame {msg.header.frame_id}')

        # Run VSLAM tracking
        pose = self._track_frame(msg)

        if pose:
            # Publish odometry
            odom = Odometry()
            odom.header = msg.header
            odom.pose.pose = pose
            self.odom_pub.publish(odom)

            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = pose
            self.pose_pub.publish(pose_msg)

    def _track_frame(self, image_msg):
        # Placeholder: actual VSLAM tracking
        return None  # Return pose if tracking successful

def main(args=None):
    rclpy.init(args=args)
    node = VSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Object Detection Pipeline

### DNN Inference with TensorRT

```python
# object_detection_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from std_msgs.msg import Header

import numpy as np
import torch

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.detect_callback,
            10
        )

        self.bbox_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        # Load TensorRT model (placeholder)
        self.model = self._load_model()

    def _load_model(self):
        self.get_logger().info('Loading TensorRT detection model...')
        # In production: torch.load('model.trt')
        return None

    def detect_callback(self, msg):
        # Convert ROS Image to numpy array
        if msg.encoding == 'rgb8':
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )[:, :, ::-1]  # RGB to BGR for OpenCV
        else:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
            return

        # Run inference
        detections = self._inference(image)

        # Publish detections
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = detections
        self.bbox_pub.publish(detection_array)

    def _inference(self, image):
        # Placeholder: actual TensorRT inference
        return []  # Return list of Detection2D

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Point Cloud Processing

```python
# pointcloud_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

import numpy as np

class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/pointcloud',
            10
        )

        self.latest_color = None
        self.latest_depth = None

    def color_callback(self, msg):
        self.latest_color = msg

    def depth_callback(self, msg):
        self.latest_depth = msg
        if self.latest_color is not None:
            self._generate_pointcloud()

    def _generate_pointcloud(self):
        # Convert depth and color to point cloud
        color = np.frombuffer(self.latest_color.data, dtype=np.uint8).reshape(
            self.latest_color.height, self.latest_color.width, 3
        )
        depth = np.frombuffer(self.latest_depth.data, dtype=np.uint16).reshape(
            self.latest_depth.height, self.latest_depth.width
        )

        # Camera intrinsics (example values - replace with actual)
        fx, fy = 616.0, 616.0
        cx, cy = 424.0, 240.0

        # Generate point cloud
        points = []
        for v in range(depth.shape[0]):
            for u in range(depth.shape[1]):
                z = depth[v, u] / 1000.0  # Convert mm to m
                if z > 0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])

        self.get_logger().info(f'Generated {len(points)} points')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Nav2 Integration

```python
# navigation_node.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import IsPathValid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import actionlib
import rclpy.action

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.client = actionlib.SimpleActionClient(
            'navigate_to_pose',
            NavigateToPose
        )

        self.get_logger().info('Waiting for Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('Connected to Nav2')

    def navigate_to(self, x, y, theta):
        goal = NavigateToPose.Goal()
        goal.pose.header = Header(frame_id='map')
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = theta

        self.client.send_goal(goal)
        self.client.wait_for_result()

        return self.client.get_result()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    # Navigate to position (2.0, 3.0) with 0 rad rotation
    result = node.navigate_to(2.0, 3.0, 0.0)

    if result == actionlib.GoalStatus.SUCCEEDED:
        node.get_logger().info('Navigation successful!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### GPU Memory Management

```python
# optimize_pipeline.py
import torch

def optimize_for_inference():
    # Use TensorRT for optimized inference
    if torch.cuda.is_available():
        # Allocate GPU memory
        torch.cuda.empty_cache()

        # Use mixed precision
        with torch.cuda.amp.autocast():
            # Inference code here
            pass

        # Profile memory
        print(f'GPU Memory: {torch.cuda.memory_allocated() / 1024**3:.2f} GB')
```

### Parallel Processing

## Expanded: Practical Tips, Docker, CI and Exercises

This section adds concrete steps to containerize Isaac ROS stacks, perform headless CI tests, and practical debugging patterns.

### Containerizing Isaac ROS

Use NVIDIA Container Toolkit to run GPU-enabled containers. Example `Dockerfile` (development):

```dockerfile
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04
RUN apt-get update && apt-get install -y python3-pip python3-colcon-common-extensions
RUN pip3 install rclpy numpy
COPY . /workspace
WORKDIR /workspace
CMD ["bash"]
```

Use `docker run --gpus all` with `--network=host` for ROS 2 networking ease on local machines.

### Headless CI Smoke Test

1. Start a headless Isaac Sim session in the container and run a short scene script for 10 seconds.
2. Start ROS 2 nodes inside the container and assert that `/camera/color/image_raw` and `/pointcloud` topics appear.

Example `ci_smoke.sh`:

```bash
#!/bin/bash
set -e
# Start Isaac headless (background)
python -m omni.isaac.kit --headless &
ISAAC_PID=$!
sleep 5
ros2 topic list | grep camera || (echo 'camera topic missing' && exit 1)
kill $ISAAC_PID
```

### Debugging Tips

- Use `ros2 topic echo` and `ros2 topic hz` to validate message rates.
- Use `nvidia-smi` or `tegrastats` on Jetson to inspect GPU usage and throttling.
- When inference outputs are inconsistent, check model preprocessing (normalization, color order) to ensure parity between training and runtime.

### Exercises

1. Containerize the `ObjectDetectionNode`, build a CI workflow that runs the `ci_smoke.sh` script on push, and fail the build if topics are missing.
2. Create a small dataset with Omniverse/Isaac Replicator and run a quick fine-tune on a small model; evaluate inference speed before and after TensorRT conversion.

---

If you want, I can scaffold the `Dockerfile` and `ci_smoke.sh` under `examples/isaac/` in the repo and add a GitHub Actions YAML that runs the smoke test. Proceed? 

```python
# parallel_pipeline.py
from concurrent.futures import ThreadPoolExecutor

class ParallelPipeline:
    def __init__(self, num_workers=4):
        self.executor = ThreadPoolExecutor(max_workers=num_workers)
        self.futures = []

    def process_frame(self, frame):
        # Submit frame processing
        future = self.executor.submit(self._process_single, frame)
        self.futures.append(future)

    def _process_single(self, frame):
        # Process a single frame
        return {
            'detection': self._detect(frame),
            'depth': self._depth_process(frame),
            'segmentation': self._segment(frame),
        }

    def wait_for_results(self):
        import concurrent.futures
        results = []
        for future in concurrent.futures.as_completed(self.futures):
            results.append(future.result())
        return results
```

## Testing the Pipeline

**Before running the full pipeline:**

```
Sensors: Camera connected, publishing images
GPU: CUDA available, sufficient VRAM
ROS 2: All nodes communicating
```

**After successful setup:**

```
VSLAM: Tracking camera motion, publishing odometry
Detection: Bounding boxes on objects
Navigation: Path planning and execution
Performance: >15 FPS processing
```

## Common Issues

| Issue | Solution |
|-------|----------|
| CUDA out of memory | Reduce batch size, free GPU cache |
| No detections | Check model input size, preprocess correctly |
| Slow VSLAM | Reduce image resolution, use GPU acceleration |
| Nav2 failing | Check costmaps, verify map is loaded |

## Next Steps

- Review [Capstone Requirements](/docs/assessments)
- Explore [Hardware Setup](/docs/hardware/edge-kit)
- Learn [ROS 2 Basics](/docs/tutorials/ros2-basics)
