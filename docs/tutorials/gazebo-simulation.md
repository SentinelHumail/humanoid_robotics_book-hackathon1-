---
sidebar_position: 2
title: Gazebo Simulation Tutorial
---

# Gazebo Simulation Tutorial

Learn to simulate robots with physics, sensors, and realistic environments.

## Prerequisites

- ROS 2 installed and configured
- Basic understanding of URDF/XACRO
- Gazebo installed (`sudo apt install gazebo`)

## Installing Gazebo and ROS 2 Integration

```bash
# Install Gazebo
sudo apt install -y gazebo libgazebo11-dev

# Install ROS 2 Gazebo packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install Ignition Gazebo (newer version)
sudo apt install -y ignition-fortress

# Test installation
gz sim
```

> Note: There are two major families: classic Gazebo (`gazebo`, `gz`) and Ignition/Carillon (Ignition Fortress, `ign` / `ignition`). Distro compatibility matters — use the versions matching your ROS 2 distro. For heavy GPU rendering, ensure drivers are installed and headless options are configured for CI.

## Tutorial Overview

This tutorial walks through creating a minimal simulated robot, adding sensors (LIDAR, camera, IMU), spawning it into a world, controlling it via ROS 2 topics, and visualizing sensor output in RViz. We'll also create launch files for repeatable experiments and cover common plugin configuration pitfalls.

### Learning Outcomes

- Create a URDF/xacro robot usable by Gazebo
- Configure sensor plugins and ROS 2 topics
- Spawn models into Gazebo worlds via ROS 2
- Record sensor data, visualize in RViz, and control robot via `cmd_vel`
- Troubleshoot common simulation stability issues

---

## Step 1 — Create URDF/xacro with sensor plugins

Create `robot.urdf.xacro` with simplified links and attach Gazebo sensor plugins.

Key tips:

- Use `xacro:macro` to avoid duplication across left/right limbs.
- Separate `visual` and `collision` geometry; use low-poly collision shapes to keep physics fast.
- Set realistic `inertial` values — poor inertia causes jitter or instability.

Example snippet (extend your existing URDF):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sim_robot">
    <xacro:property name="wheel_radius" value="0.1"/>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.4 0.2"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="0.5 0.4 0.2"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="3.0"/>
            <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.03"/>
        </inertial>
    </link>

    <!-- IMU link -->
    <link name="imu_link"/>
    <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
        <origin xyz="0 0 0.1"/>
    </joint>

    <!-- Gazebo IMU plugin (remapped topic) -->
    <gazebo reference="imu_link">
        <sensor type="imu" name="imu_sensor">
            <always_on>true</always_on>
            <update_rate>200</update_rate>
            <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
                <ros>
                    <namespace>/sim_robot</namespace>
                    <remapping>~/out:=imu</remapping>
                </ros>
                <frame_name>imu_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>

</robot>
```

---

## Step 2 — Build and spawn the robot

Generate URDF and spawn with `ros2 run` utilities or a spawn node. Example spawn command (assuming `robot.urdf` exists):

```bash
# publish robot state and spawn
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity sim_robot

# alternatively, load URDF directly
ros2 param set /robot_state_publisher robot_description -p "$(xacro robot.urdf.xacro)"
ros2 run robot_state_publisher robot_state_publisher
```

Use `ros2 topic list` to confirm topics like `/sim_robot/imu`, `/sim_robot/scan`, and `/sim_robot/image_raw` are present.

---

## Step 3 — Control the robot

Most mobile robots accept velocity commands on `/cmd_vel` (geometry_msgs/Twist). Create a publisher to `/cmd_vel` and observe movement:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Drive(Node):
        def __init__(self):
                super().__init__('drive')
                self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
                self.timer = self.create_timer(0.5, self.send_cmd)

        def send_cmd(self):
                t = Twist()
                t.linear.x = 0.2
                t.angular.z = 0.1
                self.pub.publish(t)

def main():
        rclpy.init()
        node = Drive()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
```

Visualize movement in RViz or the Gazebo client.

---

## Step 4 — Visualize sensors in RViz

Start RViz and add displays for `LaserScan`, `Image`, and `IMU`. Use `rviz2` or `rqt_reconfigure` for runtime debugging.

Record a bag of sensor data for offline processing:

```bash
ros2 bag record /sim_robot/imu /sim_robot/scan /sim_robot/image_raw -o sim_run
```

Play it back later with `ros2 bag play` to validate perception pipelines without running Gazebo.

---

## Stability & Performance Tips

- Use `SDF` physics tuning: reduce `contact_max_correcting_vel` and increase `iters` for stable contacts when simulating walking.
- For humanoid kinematics, use simplified collision meshes and fast solvers to avoid slowdowns.
- Offload rendering to GPU; use headless rendering for CI with `OGRE` plugins.

## Debugging Common Problems

- If sensors publish NaNs or zeros, inspect the plugin configuration — wrong frame names or invalid parameters are common.
- If the robot spins uncontrollably, check joint limits and initial joint states; ensure controllers are not fighting each other.
- To debug spawn errors, run `gz sdf --validate <file>` or check console logs for missing plugins.

## Exercises

1. Extend the URDF with a kinematic arm (2 DOF), spawn it in Gazebo, and write a controller node to move the end-effector to a target pose.
2. Replace the `cmd_vel` open-loop driver with a closed-loop PID controller that uses odometry feedback.
3. Record a bag while the robot navigates around obstacles; implement a simple SLAM pipeline offline using recorded scans.

## Automation & CI

- For deterministic CI, use headless Gazebo runs and `ros2 bag` playback to validate perception algorithms.
- Create a GitHub Actions matrix for OS/distro combos (Ubuntu 22.04 with Humble, etc.) that runs `colcon build` and headless simulation smoke tests.

---

Next: integrate perception pipelines with `NVIDIA Isaac` in Module 3 to run GPU-accelerated inference in simulation and on edge devices.

## Understanding URDF

**URDF** (Unified Robot Description Format) describes robots for simulation.

```xml
<!-- robot.urdf.xacro -->
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Materials -->
    <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
    </material>

    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.3 0.1"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.5 0.3 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                     iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
    </link>

    <!-- Wheel -->
    <link name="front_wheel">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.15"/>
            </geometry>
            <origin rpy="0 0 0"/>
            <material name="black"/>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.15"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                     iyy="0.001" iyz="0.0" izz="0.001"/>
        </inertial>
    </link>

    <!-- Joint -->
    <joint name="front_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="front_wheel"/>
        <origin xyz="0.2 0 0" rpy="-1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <!-- Lidar Sensor -->
    <link name="lidar_link">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.05"/>
            </geometry>
        </visual>
    </link>

    <joint name="lidar_joint" type="fixed">
        <parent link="base_link"/>
        <child link="lidar_link"/>
        <origin xyz="0 0 0.1"/>
    </joint>

</robot>
```

## Creating a Gazebo World

```xml
<!-- world.sdf -->
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="robot_world">
        <!-- Physics -->
        <physics name="default_physics" default="true" type="ode">
            <gravity>0 0 -9.81</gravity>
            <ode>
                <solver>
                    <type>quick</type>
                    <iters>100</iters>
                    <precon_iters>0</precon_iters>
                    <tolerance>0.001</tolerance>
                </solver>
                <constraints>
                    <cfm>0.0</cfm>
                    <erp>0.2</erp>
                    <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
                </constraints>
            </ode>
        </physics>

        <!-- Ground Plane -->
        <ground_plane name="ground">
            <pose>0 0 0 0 0 0</pose>
            <static>true</static>
            <visual>
                <plane>
                    <size>100 100</size>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/FlatTerrain</name>
                        </script>
                    </material>
                </plane>
            </visual>
        </ground_plane>

        <!-- Lighting -->
        <light name="sun" type="directional">
            <cast_shadows>true</cast_shadows>
            <pose>0 10 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
        </light>

        <!-- Obstacles -->
        <model name="box_obstacle">
            <pose>2 0 0.5 0 0 0</pose>
            <static>true</static>
            <link name="box">
                <visual>
                    <geometry>
                        <box size="1 1 1"/>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Red</name>
                        </script>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <box size="1 1 1"/>
                    </geometry>
                </collision>
            </link>
        </model>

    </world>
</sdf>
```

## Simulating Sensors

### Lidar Plugin

```xml
<!-- lidar plugin in URDF -->
<gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
        <pose>0 0 0 0 0 0</pose>
        <ray>
            <scan>
                <horizontal>
                    <samples>720</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.1</min>
                <max>30.0</max>
                <resolution>0.01</resolution>
            </range>
        </ray>
        <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
            <ros>
                <namespace>/robot</namespace>
                <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>lidar_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

### Camera Plugin

```xml
<!-- camera plugin in URDF -->
<gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
        <pose>0 0 0 0 0 0</pose>
        <camera>
            <horizontal_fov>1.396</horizontal_fov>
            <image>
                <width>1280</width>
                <height>720</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
        <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
            <ros>
                <namespace>/robot</namespace>
                <remapping>~/out:=image_raw</remapping>
            </ros>
            <frame_name>camera_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

### IMU Plugin

```xml
<!-- IMU plugin in URDF -->
<gazebo reference="imu_link">
    <sensor type="imu" name="imu_sensor">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
            <ros>
                <namespace>/robot</namespace>
                <remapping>~/out:=imu</remapping>
            </ros>
            <frame_name>imu_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

## Launching in Simulation

```python
# launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_pkg')

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
            ])
        ),
        # Spawn robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'spawn_robot.launch.py'])
            ])
        ),
    ])
```

```python
# launch/gazebo.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
            output='screen'
        )
    ])
```

## Before and After

**Before launching the simulation:**

```
Environment: Empty Gazebo window
Robot: Not loaded
Sensors: No data
```

**After successful simulation launch:**

```
Environment: Ground plane, lights, obstacles loaded
Robot: URDF model visible, physics enabled
Sensors: Publishing data on /robot/scan, /robot/image_raw, /robot/imu
Controls: Can send velocity commands to move robot
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Robot falls through ground | Check inertial parameters and joint connections |
| Lidar returns no data | Verify sensor plugin XML syntax |
| Slow simulation | Reduce solver iterations, use GPU for rendering |
| Joints unstable | Adjust PID gains, check joint limits |
| Model not spawning | Check URDF/XACRO syntax with `check_urdf` |

## Next Steps

- Explore [NVIDIA Isaac Integration](/docs/tutorials/isaac-perception)
- Review [Hardware Setup](/docs/hardware/requirements)
- Build your [Capstone Project](/docs/assessments)
