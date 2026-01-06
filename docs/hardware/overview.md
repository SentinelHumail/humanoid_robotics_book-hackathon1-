---
title: Hardware Overview
sidebar_label: Overview
---

# Hardware Requirements

To successfully complete this course, you will need specific hardware for both your workstation (Sim Rig) and your physical robot (Edge Kit). This section provides a comprehensive overview of the hardware ecosystem required for Physical AI and humanoid robotics.

## Quick Summary

| Component | Recommendation | Minimum |
| :--- | :--- | :--- |
| **GPU** | NVIDIA RTX 4090 | NVIDIA RTX 3060 (12GB) |
| **CPU** | Intel Core i9 / AMD Ryzen 9 | Intel Core i7 / AMD Ryzen 7 |
| **RAM** | 64GB DDR5 | 32GB DDR4 |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 20.04 LTS |

## Hardware Architecture Overview

The Physical AI stack requires three distinct hardware layers:

### 1. The Simulation Rig (Sim Rig)
The primary workstation for running Isaac Sim, Gazebo, and training VLA models. This is the most demanding component due to the combination of:
- High-fidelity physics simulation
- Real-time rendering for digital twins
- VLA model training and inference
- Computer vision processing

### 2. The Edge Computing Unit (Edge Kit)
The embedded computer that runs on the robot during deployment. Key characteristics:
- Power efficiency for battery operation
- Real-time processing capabilities
- Robustness for field deployment
- ROS 2 compatibility

### 3. The Physical Platform
The actual robot hardware where your AI brain will be deployed. This can range from:
- Quadruped robots (e.g., Unitree Go2)
- Robotic arms for manipulation tasks
- Humanoid platforms for advanced locomotion
- Custom-built platforms for research

## Detailed Hardware Requirements

### Simulation Rig Specifications

The simulation rig is the most critical component for the development phase. Here's why these specifications matter:

**GPU Requirements:**
- **VRAM**: 12GB minimum, 24GB+ recommended
- **Architecture**: NVIDIA RTX (Ampere or Ada Lovelace) required for Isaac Sim
- **Ray Tracing**: Essential for realistic lighting in digital twins
- **CUDA Cores**: Required for AI model training and inference

**CPU Requirements:**
- **Cores**: 12+ cores recommended for parallel physics simulation
- **Threads**: 24+ threads for handling multiple ROS 2 nodes
- **Architecture**: Intel 12th Gen+ or AMD Ryzen 5000+ series

**Memory Requirements:**
- **Capacity**: 64GB minimum for complex scene rendering
- **Speed**: DDR5 4800MHz+ for faster data transfer
- **Type**: ECC memory recommended for research environments

### Edge Kit Specifications

For the embedded deployment platform:

| Component | Recommended | Alternative |
|-----------|-------------|-------------|
| **SoC** | NVIDIA Jetson Orin Nano (8GB) | Jetson AGX Orin |
| **CPU** | ARM Cortex-A78AE (8-core) | ARM Cortex-A78AE (16-core) |
| **GPU** | NVIDIA Ampere GPU (2048 CUDA cores) | NVIDIA Ampere GPU (4096 CUDA cores) |
| **Memory** | 8GB LPDDR5 | 16GB LPDDR5 |
| **Storage** | 128GB eMMC | 256GB eMMC |

## Hardware Comparison Matrix

| Platform | Use Case | Performance | Power | Cost | Recommendation |
|----------|----------|-------------|-------|------|----------------|
| RTX 4090 | Training + Simulation | 10/10 | 450W | $1,599 | Best for research |
| RTX 4080 | Simulation | 8/10 | 320W | $999 | Good balance |
| RTX 4070 Ti | Light simulation | 6/10 | 285W | $799 | Entry-level |
| Jetson Orin Nano | Edge deployment | 7/10 | 15W | $249 | Essential |
| Jetson AGX Orin | Heavy edge AI | 9/10 | 30W | $599 | Advanced |

## Edge Kit
We recommend the **NVIDIA Jetson Orin Nano** for the physical robot exercises. The Jetson platform offers:

### Key Advantages
- **NVIDIA Isaac ROS**: Full compatibility with ROS 2 ecosystem
- **TensorRT**: Optimized AI inference on embedded hardware
- **Power Efficiency**: 15W TDP for battery operation
- **RealSense Integration**: Seamless depth camera connectivity
- **Industrial Grade**: Designed for robotics applications

### Included Components
- **GPU**: 1024 CUDA cores (NVIDIA Ampere architecture)
- **CPU**: ARM Cortex-A78AE octa-core processor
- **Memory**: 4GB/8GB LPDDR5
- **Connectivity**: Gigabit Ethernet, Wi-Fi 6, Bluetooth 5.2
- **I/O**: Multiple GPIO, I2C, SPI, UART interfaces

## Alternative Hardware Options

### Cloud-Based Simulation
For users without high-end workstations, cloud alternatives include:
- **AWS g5.2xlarge**: A10G GPU (24GB VRAM) - $1.51/hour
- **Azure Standard_NC4as_T4_v3**: Tesla T4 (16GB VRAM) - $0.97/hour
- **Google A2 series**: A100 GPUs for heavy training workloads

### Budget-Friendly Options
For educational institutions or individual learners:
- **RTX 3060 Ti**: 8GB VRAM, sufficient for basic simulation
- **AMD Radeon Pro W6600**: Professional workstation GPU
- **Intel Arc A770**: Emerging competitor with limited Isaac Sim support

## Hardware Procurement Guide

### Timing Considerations
- **New GPU Launches**: Avoid periods around new RTX series launches due to price fluctuations
- **Educational Discounts**: Check NVIDIA Academic Hardware Program for discounts
- **Bulk Purchasing**: Institutions can often negotiate 10-20% discounts

### Quality Assurance
- **Warranty**: Minimum 3-year warranty for workstations
- **Support**: Direct NVIDIA support for RTX cards
- **Compatibility**: Verify Ubuntu 22.04 LTS compatibility before purchase

## Hardware Lifecycle Management

### Maintenance Schedule
- **Monthly**: Dust cleaning, thermal paste inspection
- **Quarterly**: Driver updates, performance benchmarking
- **Annually**: Hardware health assessment, upgrade planning

### Upgrade Path
- **GPU**: Plan for 3-4 year upgrade cycles
- **CPU**: Less critical, 5-6 year lifecycle
- **Memory**: Consider 128GB for future-proofing

## Troubleshooting Common Hardware Issues

### GPU-Related Problems
- **Isaac Sim Won't Launch**: Verify RTX series, update drivers to latest version
- **Performance Issues**: Check VRAM usage, close unnecessary applications
- **CUDA Errors**: Verify CUDA toolkit installation and version compatibility

### Edge Kit Issues
- **Overheating**: Ensure proper cooling, consider heat sinks or fans
- **Network Connectivity**: Verify ROS 2 domain settings
- **Power Management**: Monitor power consumption during AI inference

## Next Steps

- [Hardware Requirements](/docs/hardware/requirements) - Detailed specifications
- [Edge Kit Setup](/docs/hardware/edge-kit) - Configuration guide
- [Robot Lab Setup](/docs/hardware/robot-lab) - Physical setup and operations
- [Comparison Guide](/docs/hardware/comparison) - Cost analysis
