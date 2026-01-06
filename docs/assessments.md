---
sidebar_position: 99
title: Course Assessments
---

# Course Assessments

This document outlines the assessment criteria and projects for the Physical AI & Humanoid Robotics course.

## Assessment Overview

| Assessment | Weight | Due Week | Type |
|------------|--------|----------|------|
| ROS 2 Package Development | 20% | Week 5 | Project |
| Gazebo Simulation | 20% | Week 7 | Project |
| Isaac Perception Pipeline | 20% | Week 10 | Project |
| Capstone Project | 40% | Week 13 | Project |

## 1. ROS 2 Package Development (20%)

### Requirements

Create a functional ROS 2 package that demonstrates:

- **Node architecture**: At least 3 interconnected nodes
- **Topics**: Publisher-subscriber communication
- **Services**: Synchronous service calls
- **Actions**: Asynchronous action clients
- **Parameters**: Dynamic parameter configuration
- **Launch files**: Reproducible startup configuration

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Code Quality | 30% | Clean, documented, Pythonic code |
| ROS 2 Concepts | 30% | Correct use of nodes, topics, services |
| Testing | 20% | Unit tests with coverage > 80% |
| Documentation | 20% | README with running instructions |

### Example Package Structure

```text
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── robot_controller.py
│   ├── sensor_node.py
│   └── navigation_node.py
├── test/
│   ├── test_robot_controller.py
│   └── test_sensor_node.py
├── launch/
│   ├── robot_system.launch.py
│   └── single_node.launch.py
├── config/
│   └── params.yaml
├── package.xml
├── setup.py
└── setup.cfg
```

## 2. Gazebo Simulation (20%)

### Requirements

Implement a robot simulation in Gazebo that demonstrates:

- **URDF/SDF model**: Custom robot description
- **Physics integration**: Mass, inertia, joint configuration
- **Sensor simulation**: LiDAR, camera, IMU data
- **World environment**: Gravity, ground plane, obstacles
- **Plugin integration**: Custom Gazebo plugins

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Model Accuracy | 30% | Realistic robot representation |
| Physics Behavior | 30% | Correct dynamics and collisions |
| Sensor Quality | 20% | Accurate sensor simulation |
| Environment | 20% | Realistic test scenarios |

### Deliverables

1. Complete URDF/XACRO robot model
2. Gazebo world file with obstacles
3. Sensor configuration (LiDAR, camera, IMU)
4. Video demonstration of simulation

## 3. Isaac Perception Pipeline (20%)

### Requirements

Build an AI perception pipeline using NVIDIA Isaac that demonstrates:

- **VSLAM**: Visual SLAM for localization
- **Object Detection**: Bounding box detection
- **Depth Estimation**: Monocular or stereo depth
- **Point Cloud Processing**: 3D scene understanding

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Accuracy | 30% | Detection and localization precision |
| Performance | 30% | Real-time processing (> 15 FPS) |
| Integration | 20% | ROS 2 bridge functionality |
| Documentation | 20% | Pipeline architecture diagram |

### Architecture

```text
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Camera    │────▶│  VSLAM      │────▶│  Map        │
│   Input     │     │  (Isaac)    │     │  Building   │
└─────────────┘     └─────────────┘     └─────────────┘
                          │
                          ▼
                   ┌─────────────┐
                   │  Object     │
                   │  Detection  │
                   └─────────────┘
                          │
                          ▼
                   ┌─────────────┐
                   │  3D Scene   │
                   │ 理解        │
                   └─────────────┘
```

## 4. Capstone Project (40%)

### Requirements

The capstone brings together all course concepts. Build an autonomous humanoid robot system that:

1. **Receives voice commands** via OpenAI Whisper
2. **Plans actions** using LLMs to translate natural language
3. **Navigates** using Nav2 path planning
4. **Identifies objects** using computer vision
5. **Manipulates objects** with humanoid hands

### Example Capstone Projects

| Project | Description |
|---------|-------------|
| Room Cleaning Bot | Navigate home, identify objects, move to trash |
| Fetch Bot | Find and retrieve requested objects |
| Organizer | Sort items into designated locations |
| Companion Bot | Follow and assist a human user |

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| System Integration | 25% | All components working together |
| Autonomy | 25% | Independent decision-making |
| Physical Interaction | 20% | Successful manipulation |
| Innovation | 15% | Creative problem-solving |
| Documentation | 15% | Technical report and presentation |

### Submission Requirements

1. **Technical Report** (5000 words)
   - System architecture
   - Design decisions
   - Implementation details
   - Challenges and solutions
   - Future improvements

2. **Demo Video** (10 minutes)
   - System overview
   - Each capability demonstrated
   - Failure modes and recovery

3. **Source Code**
   - Complete ROS 2 package
   - Documentation
   - Setup instructions

4. **Presentation** (15 minutes)
   - Problem statement
   - Solution approach
   - Live demo
   - Q&A

## Grading Scale

| Grade | Percentage | Description |
|-------|------------|-------------|
| A | 90-100% | Exceeds all requirements |
| B | 80-89% | Meets all requirements |
| C | 70-79% | Meets basic requirements |
| D | 60-69% | Partial completion |
| F | &lt;60% | Incomplete |

## Late Policy

- **1-7 days late**: 10% penalty per day
- **7+ days late**: Grade capped at 70%
- **Extensions**: Must request 48 hours before deadline

## Academic Integrity

All work must be original. You may:

- Use open-source libraries (with attribution)
- Reference tutorials and documentation
- Collaborate on conceptual understanding

You may not:

- Copy code without attribution
- Submit others' work as your own
- Use AI to generate code without understanding

Violations result in academic disciplinary action.

---

## Expanded Guidance, Examples & Grading Notes

This section provides practical guidance to help you design submissions that meet assessment criteria and scale to real-world robotics problems.

### Design & Architecture Expectations

- **Modularity**: Organize packages by responsibility (perception, planner, controller). Each node should do one logical thing and publish clear topics/interfaces.
- **Interfaces**: Prefer standard ROS message types (`sensor_msgs`, `geometry_msgs`, `nav_msgs`, `vision_msgs`) to simplify integration with other teams and tools.
- **Configuration**: Put tunable parameters in YAML files and expose them through ROS 2 `parameters` so graders can reproduce results with different settings.

### Example Project Walkthrough: "Fetch Bot"

1. `perception/` package: subscribes to camera and depth topics, publishes `DetectedObjectArray` messages. Includes a `detector.py` node and unit tests for non-vision logic.
2. `planner/` package: receives text intents, queries an LLM (or rule-based fallback), and outputs an action sequence as ROS 2 `Action` goals.
3. `control/` package: navigation action client and manipulation action client. Uses `FollowJointTrajectory` for arms and `NavigateToPose` for movement.
4. `launch/` folder: `demo.launch.py` starts the perception, planner, and control stacks together with simulation.

### Common Grading Failures & How to Avoid Them

- **Non-reproducible demos**: Ensure `launch` files set deterministic seeds, declare required parameters, and document hardware/software versions in the README.
- **No tests**: Even simple unit tests for pure functions catch regressions and are expected. Provide `pytest` tests for perception preprocessing and planner parsing.
- **Tight coupling**: Avoid ad-hoc global state; use topics/services/actions to decouple components and make substitution easy (e.g., swap in a simulated camera).

### Example Rubric Addendum (for graders)

- **Reproducibility (10%)**: Does `colcon build` succeed and do `ros2 launch` commands start the system? Are dependencies declared?
- **Resilience (10%)**: If a node crashes, does the system recover or fail gracefully? Are error handlers present?
- **Safety (10%)**: Are limits enforced before commanding actuators? Is an emergency stop implemented?

### Feedback Template for Students

When reviewers provide feedback, they should include:
- **Repro Steps**: Exact commands used to reproduce the result
- **Logs**: Key `ros2` logs and `dmesg` or driver errors if hardware-related
- **Suggested Fixes**: Concise steps (1-3) to address major issues

---

If you want, I can now create starter templates for each assessment (skeleton packages, launch files, and CI workflow). Would you like that? 
