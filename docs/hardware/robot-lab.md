---
sidebar_position: 3
title: Robot Lab Setup
---

# Robot Lab Setup

Setting up a physical robot lab is essential for the "Physical" part of Physical AI. This section covers the requirements, layout, safety considerations, and operational procedures for creating an effective robot lab environment.

## Lab Options Based on Budget and Goals

Depending on your budget and goals, you have several options for setting up a physical robot lab.

### Option A: The "Proxy" Approach (Recommended)

Use a quadruped robot as a learning platform. This is the most cost-effective way to learn ROS 2 and robot control.

#### Unitree Go2 Edu

| Feature | Specification |
|---------|---------------|
| **Price** | $1,800 - $3,000 |
| **Type** | Quadruped (dog-like) |
| **ROS 2 Support** | Excellent - native support |
| **Durability** | High - designed for education |
| **Sensors** | LiDAR, camera, IMU included |

**Why This Approach Works:**

- Software principles (ROS 2, VSLAM, Isaac Sim) transfer 90% to humanoids
- Affordable enough to have multiple units for a class
- Durable design withstands student use
- Excellent documentation and community support

**What You Lose:**

- Not a biped (doesn't walk on two legs)
- Different center of gravity considerations
- Simplified manipulation (no humanoid arms)

### Option B: Miniature Humanoid

For students who want to work with actual humanoid form factor at a lower cost.

#### Comparison Table

| Robot | Price | Pros | Cons |
|-------|-------|------|------|
| **Unitree G1** | ~$16,000 | Actual humanoid form, dynamic walking | Expensive, complex |
| **Robotis OP3** | ~$12,000 | Stable platform, good ROS support | Older, limited capabilities |
| **Hiwonder TonyPi Pro** | ~$600 | Affordable | Limited compute, Raspberry Pi based |

#### Unitree G1

The G1 is a miniature humanoid that provides:

- 14 degrees of freedom
- On-board computing (Raspberry Pi compatible)
- WiFi control
- Basic grippers

**Limitations:**
- Cannot run NVIDIA Isaac ROS efficiently
- Use for kinematics (walking) only
- Use Jetson kit for AI perception

#### Robotis OP3

A proven educational humanoid:

- 20 degrees of freedom
- CM-530 controller
- Excellent ROS support
- Mature documentation

**Best for:** Learning humanoid kinematics and basic walking patterns

#### Hiwonder TonyPi Pro

Budget option for beginners:

- Raspberry Pi 4B based
- Vision capabilities
- Speech recognition
- Python programmable

**Warning:** Cannot run NVIDIA Isaac ROS. Use only for basic concepts.

### Option C: Premium Lab (Sim-to-Real)

For institutions focused on actual humanoid deployment.

#### Unitree H1

| Feature | Specification |
|---------|---------------|
| **Price** | $90,000+ |
| **Height** | ~1.8 meters |
| **Degrees of Freedom** | 13+ |
| **Computing** | External (Jetson/PC) |
| **Walking** | Dynamic, terrain-adaptive |

**When to choose this:**
- Research on human-robot interaction
- Sim-to-real transfer studies
- Demonstration of cutting-edge robotics

## Lab Infrastructure Requirements

### Physical Space Requirements

| Requirement | Specification | Rationale |
|-------------|---------------|-----------|
| **Minimum Area** | 20m² (215 ft²) | For safe robot operation and student workstations |
| **Ceiling Height** | 2.5m (8.2 ft) minimum | For humanoid robot operation and safety |
| **Floor Type** | Smooth, non-slip surface | Easy to clean, good for wheel/foot traction |
| **Weight Capacity** | 500 kg/m² | For heavy robots and equipment |
| **Lighting** | 500+ lux, consistent | For computer vision algorithms |

### Environmental Conditions

| Parameter | Range | Notes |
|-----------|-------|-------|
| **Temperature** | 18-25°C (64-77°F) | Optimal for electronics and human comfort |
| **Humidity** | 30-70% RH | Prevent condensation on electronics |
| **Air Quality** | Low dust, no strong odors | For sensors and human safety |
| **Noise Level** | &lt;60 dB | For audio processing and communication |

### Network Requirements

- **Bandwidth**: 1Gbps minimum for cloud connectivity
- **Latency**: &lt;5ms for local control
- **WiFi**: Dedicated network for robot control
- **Security**: VLAN isolation for robot traffic

### Power Requirements

| Component | Voltage | Current |
|-----------|---------|---------|
| Workstation | 110-240V | 10A |
| Jetson Orin | 5V | 4A |
| Robot | 24V | Varies |
| Charging Stations | 110-240V | 15A per station |

## Lab Layout Options

### Small Lab Layout (5-10 Students)

```
┌─────────────────────────────────────────────────────────────┐
│                    Small Robot Lab (20m²)                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ Workstation │  │ Workstation │  │ Workstation │        │
│  │    (S1)     │  │    (S2)     │  │    (S3)     │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Robot Operating Area                   │   │
│  │  ┌─────────┐        ┌─────────┐                   │   │
│  │  │  Robot  │        │  Robot  │                   │   │
│  │  │  (R1)   │        │  (R2)   │                   │   │
│  │  └─────────┘        └─────────┘                   │   │
│  │                                                     │   │
│  │         ┌─────────────────────────┐                 │   │
│  │         │    Demo & Charging      │                 │   │
│  │         │        Station          │                 │   │
│  │         └─────────────────────────┘                 │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ Workstation │  │ Workstation │  │ Workstation │        │
│  │    (S4)     │  │    (S5)     │  │    (S6)     │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Medium Lab Layout (10-20 Students)

```
┌─────────────────────────────────────────────────────────────┐
│                   Medium Robot Lab (50m²)                  │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────┐   │
│  │                Instructor Area                      │   │
│  │  ┌─────────────┐  ┌─────────────────────────────┐  │   │
│  │  │ Instructor  │  │    Large Demo Screen        │  │   │
│  │  │ Workstation │  │    (65" 4K)               │  │   │
│  │  └─────────────┘  └─────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Robot Operating Area                   │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐ │   │
│  │  │  Robot  │  │  Robot  │  │  Robot  │  │  Robot  │ │   │
│  │  │  (R1)   │  │  (R2)   │  │  (R3)   │  │  (R4)   │ │   │
│  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘ │   │
│  │                                                     │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐ │   │
│  │  │  Robot  │  │  Robot  │  │  Robot  │  │  Robot  │ │   │
│  │  │  (R5)   │  │  (R6)   │  │  (R7)   │  │  (R8)   │ │   │
│  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘ │   │
│  └─────────────────────────────────────────────────────┘   │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ Workstation │  │ Workstation │  │ Workstation │        │
│  │    (S1)     │  │    (S2)     │  │    (S3)     │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ Workstation │  │ Workstation │  │ Workstation │        │
│  │    (S4)     │  │    (S5)     │  │    (S6)     │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

## Safety Requirements

### Safety Equipment

| Equipment | Quantity | Location | Notes |
|-----------|----------|----------|-------|
| **First Aid Kit** | 1 per 10 students | Central location, visible | Certified kit with robot-specific injuries |
| **Emergency Stop Buttons** | 1 per robot area | Wall-mounted, red | Hardwired, immediate shutdown |
| **Fire Extinguisher** | 1 per 20m² | Accessible locations | Class C for electronics |
| **Safety Barriers** | As needed | Around robot areas | Prevent unauthorized access |
| **Safety Goggles** | 1 per student | Station access | For high-speed operations |

### Safety Protocols

#### Pre-Operation Checklist
- [ ] All safety equipment functional
- [ ] Emergency procedures reviewed
- [ ] Robot battery level >50%
- [ ] Operating area clear of obstacles
- [ ] Emergency stop accessible
- [ ] Students briefed on safety procedures

#### Emergency Procedures
1. **Robot Malfunction**: Press nearest emergency stop button
2. **Injury**: Stop all operations, provide first aid, call emergency services
3. **Fire**: Evacuate area, call fire department, use appropriate extinguisher
4. **Power Outage**: Switch to emergency lighting, safely power down robots

### Operational Limits

| Parameter | Limit | Rationale |
|-----------|-------|-----------|
| **Maximum Speed** | 2 m/s in lab | Safety for humans in shared space |
| **Operating Hours** | 8 AM - 8 PM | Supervised operation required |
| **Student-to-Robot Ratio** | Max 3:1 | Adequate supervision |
| **Noise Level** | &lt;70 dB | Hearing protection and communication |

## Equipment Setup

### Robot Charging Stations

```
┌─────────────────────────────────────────────────────────────┐
│                   Charging Station                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Robot     │  │   Robot     │  │   Robot     │        │
│  │   Dock 1    │  │   Dock 2    │  │   Dock 3    │        │
│  │             │  │             │  │             │        │
│  │ [Charging]  │  │ [Charging]  │  │ [Charging]  │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Power Distribution                     │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐           │   │
│  │  │  220V AC │ │  24V DC  │ │  12V DC  │           │   │
│  │  │  (Main)  │ │(Chargers)│ │(Robot)   │           │   │
│  │  └──────────┘ └──────────┘ └──────────┘           │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Network Infrastructure

#### Physical Network Layout

| Component | Quantity | Specification | Purpose |
|-----------|----------|---------------|---------|
| **Core Switch** | 1 | 48-port, 10GbE | Central networking |
| **Access Points** | 2-4 | Wi-Fi 6E, 6GHz | Robot connectivity |
| **Cables** | As needed | Cat6a or better | Reliable connections |
| **Rack** | 1 | 19", 12U | Equipment organization |

### Power Infrastructure

#### Power Distribution

| Circuit | Amperage | Purpose | Devices |
|---------|----------|---------|---------|
| **Main Lab** | 20A | Workstations | 6-8 student stations |
| **Robot Areas** | 20A | Robots & Charging | 4-6 robot stations |
| **Network/AV** | 15A | Infrastructure | Switches, APs, Displays |
| **Emergency** | 20A | Safety systems | Emergency lighting, alarms |

## Maintenance Procedures

### Daily Maintenance

- [ ] Check all workstations operational
- [ ] Verify robot charging status
- [ ] Test network connectivity
- [ ] Inspect safety equipment
- [ ] Clean robot operating area

### Weekly Maintenance

- [ ] Update robot firmware
- [ ] Check battery health (voltage, charge cycles)
- [ ] Clean sensors (cameras, LiDAR, IMU)
- [ ] Test emergency stop systems
- [ ] Update software packages

### Monthly Maintenance

- [ ] Deep clean all equipment
- [ ] Calibrate robot sensors
- [ ] Check mechanical components (wheels, joints)
- [ ] Backup student projects
- [ ] Review and update safety procedures

### Quarterly Maintenance

- [ ] Comprehensive system backup
- [ ] Robot calibration validation
- [ ] Network security audit
- [ ] Safety equipment inspection
- [ ] Inventory check and reorder

## Robot Fleet Management

### Robot Profiles

| Robot Type | Quantity | Use Case | Maintenance Cycle |
|------------|----------|----------|-------------------|
| **Unitree Go2** | 4-6 | Quadruped locomotion | Weekly |
| **Robotic Arm** | 2-3 | Manipulation tasks | Daily |
| **Differential Drive** | 2-4 | Navigation basics | Daily |
| **Custom Platform** | 1-2 | Research projects | As needed |

### Software Management

```bash
#!/bin/bash
# robot-fleet-manager.sh

# Fleet status monitoring
check_fleet_status() {
    echo "Checking robot fleet status..."

    for robot in robot-{01..08}; do
        if ssh -o ConnectTimeout=5 $robot "systemctl is-active --quiet robot-ai"; then
            echo "$robot: ACTIVE"
        else
            echo "$robot: INACTIVE"
        fi
    done
}

# Software update across fleet
update_fleet_software() {
    echo "Updating software across fleet..."

    for robot in robot-{01..08}; do
        echo "Updating $robot..."
        ssh $robot "sudo apt update && sudo apt upgrade -y"
        ssh $robot "docker system prune -f"
    done
}

# Configuration sync
sync_configurations() {
    echo "Syncing configurations..."

    for robot in robot-{01..08}; do
        rsync -av config/ $robot:/opt/robot/config/
    done
}

case "${1:-}" in
    status)
        check_fleet_status
        ;;
    update)
        update_fleet_software
        ;;
    sync)
        sync_configurations
        ;;
    *)
        echo "Usage: $0 {status|update|sync}"
        ;;
esac
```

## Troubleshooting Common Issues

### Network Issues

| Problem | Symptoms | Solution |
|---------|----------|----------|
| Robot disconnects | Intermittent communication loss | Check Wi-Fi signal strength, reduce interference |
| Slow communication | High latency, dropped packets | Check network congestion, prioritize robot traffic |
| IP conflicts | Communication failures | Use static IPs, check DHCP settings |

### Hardware Issues

| Problem | Symptoms | Solution |
|---------|----------|----------|
| Motor errors | Robot not moving properly | Check motor connections, recalibrate |
| Sensor failures | Perception issues | Clean sensors, check calibration |
| Battery drain | Short operating time | Check battery health, optimize code |

### Software Issues

| Problem | Symptoms | Solution |
|---------|----------|----------|
| ROS communication | Nodes not talking | Check ROS domain settings, network config |
| Performance issues | Slow responses | Monitor CPU/GPU usage, optimize code |
| Model loading failures | AI not working | Check model compatibility, memory usage |

## Budget Summary

| Lab Type | Upfront Cost | Per Student |
|----------|--------------|-------------|
| Proxy (Go2) | $15,000 (5 robots) | $3,000 |
| Miniature (G1) | $50,000 (3 robots) | $16,000 |
| Premium (H1) | $300,000 (3 robots) | $100,000 |
| Cloud-Only | $5,000 (cloud credits) | $1,000 |

### Setup Costs (10-Student Lab)

| Category | Item | Unit Cost | Quantity | Total |
|----------|------|-----------|----------|-------|
| **Infrastructure** | Tables & Chairs | $200 | 10 | $2,000 |
| | Network Equipment | $1,500 | 1 | $1,500 |
| | Safety Equipment | $800 | 1 | $800 |
| **Robots** | Unitree Go2 | $1,800 | 4 | $7,200 |
| | Robotic Arms | $800 | 2 | $1,600 |
| **Computing** | Edge Kits (Jetson) | $700 | 10 | $7,000 |
| | Workstations | $2,500 | 10 | $25,000 |
| **Total Setup** | | | | **$45,100** |

### Operational Costs (Annual)

| Category | Cost | Notes |
|----------|------|-------|
| **Electricity** | $1,200 | Lab operation, charging |
| **Maintenance** | $3,000 | Parts, service contracts |
| **Software Licenses** | $2,000 | Simulation, development tools |
| **Consumables** | $800 | Batteries, cables, cleaning |
| **Total Annual** | **$7,000** | |

## Recommendation

For most educational institutions:

1. **Start with Proxy Approach** (Unitree Go2)
2. **Add Cloud Access** for Isaac Sim
3. **Consider Miniature Humanoid** for advanced courses
4. **Reserve Premium** for research labs only

## Best Practices

### Lab Operation

1. **Structured Schedule**: Implement scheduled time slots to maximize utilization
2. **Student Training**: Mandatory safety and operation training before access
3. **Project Management**: Use issue tracking for student projects
4. **Resource Sharing**: Implement reservation system for high-demand equipment

### Safety First

1. **Supervision**: Maintain appropriate student-to-instructor ratios
2. **Protocols**: Enforce all safety procedures without exception
3. **Documentation**: Keep detailed logs of all incidents and near-misses
4. **Training**: Regular safety training updates for all users

### Maintenance Focus

1. **Preventive**: Regular maintenance over reactive repairs
2. **Documentation**: Keep detailed maintenance logs
3. **Spare Parts**: Maintain critical spare parts inventory
4. **Expertise**: Train staff on basic maintenance procedures

## System Architecture

```
                    ┌─────────────────┐
                    │  Control PC     │
                    │  (RTX Workstation)│
                    └────────┬────────┘
                             │ High-speed
                             │ network
                             ▼
┌──────────────────────────────────────────────────────┐
│                      Robot                            │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────┐  │
│  │   Sensors   │───▶│   Computer  │───▶│Actuators│  │
│  │ LiDAR/Camera│    │ Jetson Orin │    │ Motors  │  │
│  │    IMU      │    │    Nano     │    │         │  │
│  └─────────────┘    └─────────────┘    └─────────┘  │
└──────────────────────────────────────────────────────┘
```

---

## Expanded: Safety, Test Plans, and Lab Exercises

### Safety First

Robots are heavy and can be dangerous. Implement the following before allowing students to test code on hardware:

- **Hardware E-Stop**: A physical, accessible emergency stop that cuts power to actuators.
- **Software Safety Layer**: A watchdog node that monitors torque/velocity commands and clamps values exceeding safe thresholds.
- **Safety Barriers**: For dynamic robots, install clear plastic barriers or designated test zones.

### Test Plan Template

1. **Bench Test**: Run motors at low power, verify encoders and joint sensors report sensible values.
2. **Tethered Test**: Run controllers with safety harnesses engaged, limited joint ranges.
3. **Free Motion**: Once tethered tests pass, run free motion in a clear area with observers.

### Lab Exercises

1. **Controller Tuning**: Students tune PID controllers for a single joint and record step responses.
2. **Safety Watchdog**: Implement a ROS 2 node that kills motor commands if an IMU reports values outside safe ranges.
3. **Reproducible Failure Mode**: Intentionally misconfigure a joint limit in simulation to demonstrate safe hard limits vs controller saturation.

---

Would you like me to generate a printable safety checklist PDF or a `tests/` folder with starter test scripts? 
