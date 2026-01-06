---
sidebar_position: 1
title: On-Premise Setup
---

# On-Premise Setup

Configure a dedicated physical laboratory for Physical AI education.

## Lab Requirements

### Physical Space

| Requirement | Specification | Notes |
|-------------|---------------|-------|
| **Room Size** | 20m² minimum | For 5 student stations |
| **Ceiling Height** | 2.5m minimum | For robot operation |
| **Power** | 20A circuit per station | 110V or 220V |
| **Network** | 1Gbps wired + WiFi | Low latency for control |

### Lab Layout

```
┌──────────────────────────────────────────────────────────────┐
│                      Physical AI Lab                          │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│   │Station 1│  │Station 2│  │Station 3│  │Station 4│        │
│   │Student A│  │Student B│  │Student C│  │Student D│        │
│   └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘        │
│        │            │            │            │              │
│        └────────────┴─────┬──────┴────────────┘              │
│                           │                                   │
│                    ┌──────┴──────┐                            │
│                    │ Shared Area │                            │
│                    │ (Robots)    │                            │
│                    └─────────────┘                            │
│                                                               │
│   ┌─────────────────────────────────────────┐                 │
│   │ Instructor Station + Demo Area          │                 │
│   └─────────────────────────────────────────┘                 │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

## Workstation Configuration

### Per-Student Station

| Component | Specification | Cost |
|-----------|---------------|------|
| **Workstation** | RTX 4070 Ti, i7, 64GB RAM, 1TB NVMe | $2,500 |
| **Monitor** | 27" 4K display | $400 |
| **Peripherals** | Keyboard, mouse, headset | $150 |
| **Desk** | Standing desk, ergonomic chair | $800 |
| **Total** | | **~$3,850** |

### Shared Resources (Per 5 Students)

| Component | Quantity | Cost |
|-----------|----------|------|
| Unitree Go2 Robot | 2 | $5,000 |
| Network Switch (10Gbps) | 1 | $500 |
| AV Equipment | 1 | $1,000 |
| Spare Parts | - | $500 |
| **Total** | | **$7,000** |

## Installation Checklist

### Workstation Setup

- [ ] Install Ubuntu 22.04 LTS
- [ ] Install NVIDIA drivers (550+)
- [ ] Install CUDA 12.2
- [ ] Install Docker and NVIDIA Container Toolkit
- [ ] Install ROS 2 Humble
- [ ] Install Isaac Sim (via Omniverse)
- [ ] Configure network settings
- [ ] Verify GPU passthrough (if using VMs)

### Network Configuration

```bash
# Static IP configuration for workstations
# /etc/netplan/01-netcfg.yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp3s0:
      addresses:
        - 192.168.1.10/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

### Robot Lab Configuration

1. **Power Setup**
   - Dedicated 20A circuit per robot area
   - UPS backup for computers
   - Proper grounding

2. **Network Setup**
   - Private VLAN for robots
   - QoS for control traffic
   - Firewall rules

3. **Safety Setup**
   - Emergency stop buttons
   - Safety barriers if needed
   - First aid kit

## Software Installation

### Automated Setup Script

```bash
#!/bin/bash
# setup-workstation.sh

# Update system
sudo apt update && sudo apt upgrade -y

# Install NVIDIA drivers
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo apt install -y nvidia-driver-550

# Install CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-2

# Install ROS 2
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
sudo apt-key add /tmp/ros.key
echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop

# Install Isaac ROS
sudo apt install -y ros-humble-isaac-ros-*

echo "Setup complete! Reboot to apply changes."
```

## Maintenance

### Daily

- [ ] Check all workstations boot correctly
- [ ] Verify network connectivity
- [ ] Test robot communication

### Weekly

- [ ] Update software packages
- [ ] Check disk space
- [ ] Backup student work
- [ ] Inspect robot hardware

### Monthly

- [ ] Deep clean workstations
- [ ] Check robot calibration
- [ ] Review and replace spare parts
- [ ] Network security audit

## Troubleshooting

| Issue | Solution |
|-------|----------|
| GPU not detected | Reinstall NVIDIA drivers, check BIOS |
| ROS 2 nodes not communicating | Check firewall, verify DDS configuration |
| Robot not connecting | Check IP settings, restart network |
| Slow simulation | Close background apps, reduce graphics quality |

## Cost Summary

| Category | 5 Students | 10 Students | 20 Students |
|----------|------------|-------------|-------------|
| Workstations | $19,250 | $38,500 | $77,000 |
| Shared Resources | $7,000 | $10,000 | $15,000 |
| Installation | $2,000 | $3,000 | $5,000 |
| **Total Upfront** | **$28,250** | **$51,500** | **$97,000** |
| Monthly (power, internet) | $500 | $800 | $1,200 |
| Quarterly Maintenance | $1,000 | $1,500 | $2,500 |

---

## Expanded: On-Premise Automation & Runbook

### Automated Provisioning

Use an automated setup script (example above) combined with an image-based provisioning workflow (PXE or disk-cloning) to quickly provision lab machines.

Example Ansible playbook snippet (workstation provisioning):

```yaml
- hosts: workstations
  become: yes
  tasks:
    - name: Ensure NVIDIA drivers
      apt:
        name: nvidia-driver-550
        state: present

    - name: Install ROS 2 packages
      apt:
        name: ['ros-humble-desktop', 'python3-colcon-common-extensions']
        state: present
```

### Runbook: First-Day Lab Startup

1. Power on all workstations and verify network connectivity.
2. Run `setup-healthcheck.sh` to verify `nvidia-smi`, `ros2`, and `docker` are operational across stations.
3. Start shared services: RViz server, artifact repository, central logging endpoint.
4. Launch instructor demo (single script) that starts Gazebo and a pre-recorded scenario for students to observe.

### Maintenance Automation

- Use nightly `ansible` runs to apply security patches to non-critical nodes and weekly reboots for workstations.
- Maintain a `golden` snapshot VM or disk image for quick recovery.

Would you like me to scaffold the `Ansible` playbook and `setup-healthcheck.sh` script in this repo? 
