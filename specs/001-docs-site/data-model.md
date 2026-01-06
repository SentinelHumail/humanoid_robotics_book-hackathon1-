# Data Model: Physical AI & Humanoid Robotics Documentation Site

This document describes the content entities for the documentation site. Since this is a static site, the "data model" represents the content structure rather than database entities.

## Content Entity Overview

The documentation uses file-based content management where each Markdown/MDX file represents a discrete content entity. Entities are organized hierarchically to support navigation and cross-referencing.

## Core Entities

### CourseModule

Represents a major topic area in the 13-week curriculum. Each module contains multiple lessons and has associated learning objectives.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (e.g., "module1-ros2") |
| title | string | Yes | Display title (e.g., "ROS 2 Robotic Nervous System") |
| description | string | Yes | Brief overview (2-3 sentences) |
| duration | string | Yes | Time allocation (e.g., "Weeks 3-5") |
| lessons | array | Yes | Array of Lesson references |
| order | number | Yes | Display order in navigation (1-4) |
| prerequisites | array | No | Required prior modules |

**Example**:
```yaml
id: module1-ros2
title: ROS 2 Robotic Nervous System
description: Learn middleware for robot control including ROS 2 Nodes, Topics, Services, and URDF for humanoids.
duration: Weeks 3-5
order: 1
lessons:
  - lesson-ros2-intro
  - lesson-ros2-nodes
  - lesson-ros2-topics
  - lesson-ros2-services
  - lesson-ros2-urdf
```

### Lesson

Represents a single learning unit within a module. Lessons contain the actual educational content.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (e.g., "lesson-ros2-nodes") |
| title | string | Yes | Lesson title |
| module | string | Yes | Parent module ID |
| objectives | array | Yes | Learning outcomes |
| content | markdown | Yes | Main lesson content (MDX) |
| prerequisites | array | No | Prior knowledge needed |
| estimatedTime | string | No | Duration (e.g., "2 hours") |
| assessments | array | No | Associated exercises |

**Example**:
```yaml
id: lesson-ros2-nodes
title: ROS 2 Nodes
module: module1-ros2
objectives:
  - Understand ROS 2 node architecture
  - Create and configure nodes with rclpy
  - Use ros2 CLI tools for node management
estimatedTime: 2 hours
```

### HardwareSpec

Represents hardware requirements and specifications. Used across multiple documentation sections.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| category | enum | Yes | "workstation", "edge-kit", "robot", "sensor" |
| name | string | Yes | Product name |
| manufacturer | string | Yes | Company name |
| specs | object | Yes | Technical specifications |
| price | object | Yes | Cost information |
| alternatives | array | No | Alternative products |

**Spec Structure**:
```yaml
specs:
  gpu: "NVIDIA RTX 4070 Ti (12GB VRAM)"
  cpu: "Intel Core i7 (13th Gen+)"
  ram: "64 GB DDR5"
  os: "Ubuntu 22.04 LTS"
```

**Price Structure**:
```yaml
price:
  msrp: 2500
  currency: "USD"
  notes: "Approximate system cost"
```

**Example**:
```yaml
category: workstation
name: Digital Twin Workstation
manufacturer: Custom Build
specs:
  gpu: "NVIDIA RTX 4070 Ti (12GB VRAM)"
  cpu: "Intel Core i7 (13th Gen+)"
  ram: "64 GB DDR5"
  os: "Ubuntu 22.04 LTS"
price:
  msrp: 2500
  currency: "USD"
alternatives:
  - name: Budget Option
    specs:
      gpu: "RTX 3060 (8GB)"
      ram: "32 GB"
    price:
      msrp: 1200
```

### DeploymentOption

Represents deployment approach options for course infrastructure.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| type | enum | Yes | "on-premise", "cloud", "hybrid" |
| name | string | Yes | Display name |
| description | string | Yes | Overview description |
| pros | array | Yes | Advantages |
| cons | array | Yes | Disadvantages |
| cost | object | Yes | Cost breakdown |
| useCases | array | Yes | Recommended scenarios |
| architecture | string | No | Diagram reference |

**Cost Structure**:
```yaml
cost:
  upfront: 0
  ongoing: number
  period: "month" or "quarter" or "year"
  breakdown:
    - item: "Hardware"
      amount: number
    - item: "Cloud"
      amount: number
```

**Example**:
```yaml
type: on-premise
name: Physical Lab (On-Premise)
description: Dedicated workstations and robot hardware in a classroom setting.
pros:
  - Low latency for real-time control
  - No internet dependency
  - Direct robot access
cons:
  - High upfront hardware cost
  - Maintenance overhead
  - Space requirements
cost:
  upfront: 15000
  ongoing: 500
  period: "quarter"
  breakdown:
    - item: "Workstations (5x)"
      amount: 10000
    - item: "Robots (2x)"
      amount: 5000
useCases:
  - Institutions with dedicated lab space
  - Courses requiring physical robot access
```

## Entity Relationships

```
CourseModule (1) --> (many) Lesson
CourseModule (1) --> (many) HardwareSpec (linked references)
DeploymentOption (1) --> (many) HardwareSpec (component list)
```

## Validation Rules

1. **Module Order**: Must be 1-4 with no gaps
2. **Lesson References**: Must point to existing lesson IDs
3. **Hardware Categories**: Must be one of: workstation, edge-kit, robot, sensor
4. **Deployment Types**: Must be one of: on-premise, cloud, hybrid
5. **Price Values**: Must be positive numbers

## File Structure Mapping

| Entity Type | File Location | File Pattern |
|-------------|---------------|--------------|
| CourseModule | docs/modules/ | module[1-4]-*.md |
| Lesson | docs/modules/lessons/ | lesson-*.md |
| HardwareSpec | docs/hardware/ | *.md (frontmatter defines spec) |
| DeploymentOption | docs/deployment/ | *.md (frontmatter defines option) |
