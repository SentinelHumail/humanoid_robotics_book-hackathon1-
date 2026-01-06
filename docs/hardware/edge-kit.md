---
sidebar_position: 2
title: Edge Kit Setup
---

# Edge Kit Setup Guide

This guide covers setting up the NVIDIA Jetson Orin Nano edge kit for Physical AI development.

## Components Overview

| Component | Purpose |
|-----------|---------|
| NVIDIA Jetson Orin Nano | Edge AI computing platform |
| Intel RealSense D435i | RGB-D camera with IMU |
| ReSpeaker USB Mic Array | Voice input for Whisper integration |
| 128GB microSD Card | Operating system and storage |

## Jetson Orin Nano Setup

### 1. Flash the OS

1. Download **NVIDIA JetPack 5.1.2** or newer from NVIDIA developer site
2. Use **balenaEtcher** to flash the Jetson OS image to your microSD card
3. Insert the microSD card into the Jetson Orin Nano developer kit

### 2. Initial Boot

1. Connect monitor, keyboard, mouse, and power
2. Boot up and complete Ubuntu initial setup
3. Create user account with sudo privileges

### 3. Install Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y curl gnupg ca-certificates lsb-release

# Install CUDA Toolkit (for GPU acceleration)
sudo apt install -y cuda-toolkit-12-2

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda-12.2/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA installation
nvcc --version
```

### 4. Install ROS 2 (Humble)

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

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

# Install dependencies
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

## RealSense D435i Setup

### 1. Install Intel RealSense SDK

```bash
# Install dependencies
sudo apt install -y libssl-dev libusb-1.0-0-dev pkg-config libglfw3-dev

# Install librealsense
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831C80A5FD0C118BF2D2B2B58DFE8 2>/dev/null || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831C80A5FD0C118BF2D2B2B58DFE8

sudo add-apt-repository 'deb https://librealsense.intel.com/DebianPackage/ $(lsb_release -cs) main'
sudo apt update
sudo apt install -y librealsense2-utils librealsense2-dev
```

### 2. Verify Installation

```bash
# Test camera
realsense-viewer
```

### 3. Enable IMU in ROS 2

```bash
# Install ROS 2 RealSense package
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

Launch the camera:

```bash
ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true
```

## ReSpeaker Microphone Setup

### 1. Install Dependencies

```bash
# Install Python libraries
pip3 install webrtcvad sphinx pocketsphinx
```

### 2. Test Microphone

```bash
# List audio devices
arecord -l

# Test recording
arecord -D plughw:CARD=ArrayUAC10 -f S16_LE -r 16000 -d 5 test.wav
aplay test.wav
```

### 3. Configure for Whisper

The ReSpeaker can be used with OpenAI Whisper for voice-to-text:

```bash
# Install Whisper
pip3 install openai-whisper

# Test transcription
whisper test.wav --model small
```

## Network Configuration

### WiFi Setup

The Jetson Orin Nano supports 2.4G/5G WiFi:

```bash
# Connect to WiFi
nmcli device wifi list
nmcli device wifi connect "your-ssid" password "your-password"

# Verify connection
ip addr show wlan0
```

### SSH Access

Enable SSH for remote development:

```bash
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

## Performance Optimization

### Power Mode

Select appropriate power mode:

```bash
# Check current mode
sudo nvpmodel -q

# Set to 15W mode (recommended for edge)
sudo nvpmodel -m 1

# Set to 25W mode (maximum performance, requires more power)
sudo nvpmodel -m 0
```

### Thermal Management

Monitor temperature:

```bash
# Check temperature
sudo tegrastats

# Set fan policy
sudo jetson_clocks --show
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Camera not detected | Check USB connection, try different port |
| ROS 2 nodes failing | Verify `source /opt/ros/humble/setup.bash` |
| CUDA not found | Check `nvcc --version`, verify PATH |
| WiFi disconnecting | Check power supply (needs 5V/4A) |
| Overheating | Clean heatsink, check thermal paste |

## Next Steps

Once your edge kit is set up:

1. Test all components individually
2. Run the [ROS 2 tutorials](/docs/tutorials/ros2-basics)
3. Try the [perception pipeline](/docs/tutorials/isaac-perception)
4. Integrate with your [simulation environment](/docs/modules/module2-gazebo)

---

## Expanded: Deployment Patterns, Benchmarks, and Exercises

### Lightweight Inference Deployment (Best Practices)

- Containerize inference stacks with Docker and use NVIDIA Container Toolkit for GPU passthrough: this eases reproducible deployments across Jetson devices.
- Cross-compile heavy C++ dependencies on a workstation or use Jetson SDK components to reduce build time on the device.

Dockerfile snippet for Jetson (multi-stage):

```dockerfile
FROM nvcr.io/nvidia/l4t-base:r35.3.1 as build
RUN apt-get update && apt-get install -y python3-pip build-essential
WORKDIR /workspace
COPY requirements.txt .
RUN pip3 install -r requirements.txt

FROM nvcr.io/nvidia/l4t-base:r35.3.1
COPY --from=build /usr/local/lib/python3.10/dist-packages /usr/local/lib/python3.10/dist-packages
CMD ["python3", "edge_inference.py"]
```

### Performance Benchmarks (Jetson)

- Measure inference latency and memory usage with `nvidia-smi` (or `tegrastats` on Jetson) while running your TensorRT engine.
- Profile CPU-bound nodes using `htop` and `perf` to find hotspots to offload to the GPU.

Example scripts:

```bash
# Measure CPU and GPU utilization
nvidia-smi --query-gpu=utilization.gpu,utilization.memory --format=csv -l 1
tegrastats --interval 1000
```

### Exercises

1. Containerize your inference node and deploy to the Jetson. Measure end-to-end latency from camera capture to published detection topic.
2. Implement a nightly model fetcher: the Jetson pulls a new TensorRT engine from a secure artifact store and atomically swaps the running model.
3. Create a lightweight healthcheck endpoint on the Jetson that reports CPU temperature, available memory, and inference FPS via an HTTP JSON API.

---

If you'd like, I can scaffold a `Dockerfile`, a `deploy.sh`, and a simple `healthcheck.py` and place them under `examples/edge/` in this repo.
