---
sidebar_position: 1
title: Hardware Requirements
---

# Hardware Requirements

This course is technically demanding, sitting at the intersection of three heavy computational loads: Physics Simulation (Isaac Sim/Gazebo), Visual Perception (SLAM/Computer Vision), and Generative AI (LLMs/VLA).

## Detailed Hardware Breakdown

### 1. The NVIDIA RTX Ecosystem
The "Physical AI" revolution is powered by NVIDIA's **Ampere** and **Ada Lovelace** architectures.
- **VRAM is King**: Running a Vision-Language Model (VLM) like LLaVA or a VLA alongside a physics simulation requires massive video memory. 12GB is the floor for 7B parameter models; 24GB (RTX 3090/4090) allows for concurrent training and high-fidelity rendering.
- **CUDA Cores & Tensor Cores**: Essential for local model inference. Without high Tensor Core counts, your robot's "thinking time" (latency) will exceed its physical reaction time, leading to crashes.

### 2. Edge Computing (Jetson Series)
The **Jetson Orin Nano** and **Orin AGX** are the de-facto brains of modern humanoids.
- **SoC (System on Chip)**: Combines an ARM CPU, a powerful GPU, and Deep Learning Accelerators (NLA).
- **Power Efficiency**: Robots run on batteries. The Jetson platform provides high-TOPS (Tera Operations Per Second) at a fraction of the power of a desktop PC.

### 3. Perception Hardware: Eyes and Ears
- **Intel RealSense D435i**: Uses infrared projectors to create depth maps even in low-texture environments (like a white wall).
- **Stereo Vision vs. Time-of-Flight (ToF)**: We discuss why Stereo (RealSense) is often preferred for humanoids over ToF (LiDAR) for near-field manipulation.

## Digital Twin Workstation (Required)

This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires **RTX** (Ray Tracing) capabilities.

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | NVIDIA RTX 4070 Ti (12GB VRAM) | RTX 4090 (24GB VRAM) |
| **CPU** | Intel Core i7 (13th Gen+) | Intel Core i9 / AMD Ryzen 9 |
| **RAM** | 32 GB DDR5 | 64 GB DDR5 |
| **Storage** | 512 GB SSD | 1 TB NVMe SSD |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

### Why These Specs?

- **GPU VRAM (12GB+)**: Required to load USD (Universal Scene Description) assets for the robot and environment, plus run VLA (Vision-Language-Action) models simultaneously
- **CPU**: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive
- **RAM (64GB)**: 32GB is the absolute minimum, but will crash during complex scene rendering

:::warning
Standard laptops (MacBooks or non-RTX Windows machines) will not work for Isaac Sim.
:::

## Edge Kit (Optional - For Deployment)

Since a full humanoid robot is expensive, students learn "Physical AI" by setting up the nervous system on a desk before deploying it to a robot.

| Component | Model | Approximate Price |
|-----------|-------|-------------------|
| **The Brain** | NVIDIA Jetson Orin Nano (8GB) | $249 |
| **The Eyes** | Intel RealSense D435i | $349 |
| **The Ears** | ReSpeaker USB Mic Array v2.0 | $69 |
| **Storage** | 128GB High-endurance microSD | $30 |
| **Total** | | **~$700** |

### Component Details

- **Jetson Orin Nano**: Industry standard for embodied AI. Deploy ROS 2 nodes here to understand resource constraints vs. workstations
- **RealSense D435i**: Provides RGB and Depth data. Essential for VSLAM and Perception modules. Includes IMU
- **ReSpeaker**: Far-field microphone for voice commands (Whisper integration)

## Robot Lab Options

For the "Physical" part of the course, you have three tiers depending on budget:
Use a quadruped (dog) or robotic arm as a proxy. The software principles transfer 90% effectively to humanoids.

| Robot | Price | Pros | Cons |
|-------|-------|------|------|
| Unitree Go2 Edu | $1,800-$3,000 | Durable, excellent ROS 2 support, affordable | Not a biped (humanoid) |

### Option B: The "Miniature Humanoid" Approach

Small, table-top humanoids for kinematics and basic locomotion.

| Robot | Price | Notes |
|-------|-------|-------|
| Unitree G1 | ~$16,000 | Miniature humanoid |
| Robotis OP3 | ~$12,000 | Stable, older platform |
| Hiwonder TonyPi Pro | ~$600 | Budget option (limited capability) |

### Option C: The "Premium" Lab (Sim-to-Real)

For deploying the capstone to a real humanoid.

| Robot | Price | Notes |
|-------|-------|-------|
| Unitree G1 Humanoid | ~$90,000+ | Full-size humanoid with dynamic walking |

## Connectivity Requirements

Humanoid robots generate massive amounts of data. A standard home Wi-Fi network is usually insufficient for high-bandwidth ROS 2 traffic (especially raw point clouds from LiDAR and RGB-D streams).

| Component | Minimum | Recommended | Why? |
|-----------|---------|-------------|------|
| **Router** | Wi-Fi 6 (802.11ax) | Wi-Fi 6E (6GHz) | Minimize jitter and interference in dense environments. |
| **Local Link** | Gigabit Ethernet | 2.5GbE / 10GbE | Essential for low-latency tethered development and debugging. |
| **Dedicated Network** | VLAN | Physical LAN | Isolate robot traffic from general internet usage to prevent packet drops. |

:::tip
Always use a dedicated 5GHz or 6GHz band for your robot. The 2.4GHz band is highly susceptible to interference from Bluetooth and microwave ovens, which can cause erratic robot behavior (or "ghost pulses" in control topics).
:::

## Summary: Architecture

| Component | Hardware | Function |
|-----------|----------|----------|
| **Sim Rig** | PC with RTX 4080 + Ubuntu 22.04 | Runs Isaac Sim, Gazebo, Unity, trains LLM/VLA models |
| **Edge Brain** | Jetson Orin Nano | Runs the "Inference" stack - deploy code here |
| **Sensors** | RealSense Camera + LiDAR | Connected to Jetson for real-world data |
| **Actuator** | Unitree Go2 or G1 (Shared) | Receives motor commands from Jetson |

## Cloud Alternative

If you do not have access to RTX-enabled workstations, you can use cloud-based instances:

- **AWS g5.2xlarge** (A10G GPU, 24GB VRAM)
- **AWS g6e.xlarge** (L40S GPU, better for Isaac Sim)

Cost estimate: ~$1.50/hour, ~$205/quarter for 10 hours/week

:::note
Simulating in the cloud works well, but controlling a real robot from a cloud instance is dangerous due to latency. Students train in the Cloud, then download models for physical deployment.
:::
