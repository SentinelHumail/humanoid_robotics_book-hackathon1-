---
title: Deployment Options
sidebar_label: Overview
---

# Deployment Options

Learn how to deploy your robotic stack on-premise or in the cloud. This section covers the architecture, setup, and operational considerations for deploying your Physical AI applications across different environments.

## Overview

Deployment in Physical AI involves three key phases:
1. **Development & Training**: Running simulations and training models
2. **Testing & Validation**: Validating on digital twins and edge hardware
3. **Production Deployment**: Running on physical robots in real environments

Each phase has different requirements for latency, reliability, security, and cost that influence your deployment strategy.

## Deployment Architecture Patterns

### 1. Hybrid Cloud-Edge Architecture
The most common approach combines cloud-based training with edge deployment:

```
┌─────────────────────────────────────────────────────────────┐
│                    Hybrid Architecture                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    Cloud Training & Simulation            │
│  │   Student   │                                           │
│  │   Laptop    │    ┌─────────────────────────────────┐   │
│  │   (Dev)     │───▶│  Cloud Instance (GPU)         │   │
│  └─────────────┘    │  - Isaac Sim                    │   │
│         │            │  - Model Training               │   │
│         │            │  - Synthetic Data Generation    │   │
│         ▼            └─────────────────────────────────┘   │
│  ┌─────────────┐              │                           │
│  │   Edge Kit  │              │                           │
│  │   (Jetson)  │◀─────────────┘                           │
│  │             │    ┌─────────────────────────────────┐   │
│  │   Robot     │    │   Model Deployment & Inference│   │
│  │   Control   │───▶│   - TensorRT Optimization     │   │
│  └─────────────┘    │   - Real-time Processing      │   │
│                     │   - Safety Systems            │   │
│                     └─────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2. Pure Edge Architecture
For applications requiring ultra-low latency and privacy:

```
┌─────────────────────────────────────────────────────────────┐
│                    Pure Edge Architecture                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌─────────────────────────────────┐   │
│  │   Student   │    │   Edge Training Setup         │   │
│  │   Laptop    │    │   - Jetson Orin AGX          │   │
│  │   (Dev)     │───▶│   - Local Model Training     │   │
│  └─────────────┘    │   - Isaac ROS                  │   │
│                     └─────────────────────────────────┘   │
│                              │                            │
│                              ▼                            │
│  ┌─────────────┐    ┌─────────────────────────────────┐   │
│  │   Robot     │    │   Robot Deployment            │   │
│  │   Control   │───▶│   - Optimized Models         │   │
│  │   (Jetson)  │    │   - Real-time Inference      │   │
│  └─────────────┘    │   - Local Sensors            │   │
│                     └─────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3. Cloud-First Architecture
For collaborative development and large-scale training:

```
┌─────────────────────────────────────────────────────────────┐
│                   Cloud-First Architecture                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌─────────────────────────────────┐   │
│  │   Student   │    │   Cloud Infrastructure        │   │
│  │   Laptop    │    │   - GPU Instances (g5.xlarge) │   │
│  │   (Thin)    │───▶│   - Isaac Sim in Containers   │   │
│  └─────────────┘    │   - Model Registry            │   │
│         │            │   - CI/CD Pipeline            │   │
│         │            └─────────────────────────────────┘   │
│         │                            │                     │
│         │                            ▼                     │
│         │                   ┌─────────────────────────┐   │
│         └──────────────────▶│   Edge Deployment       │   │
│                            │   - Model Download      │   │
│                            │   - Jetson Inference    │   │
│                            │   - Robot Control       │   │
│                            └─────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Deployment Options

### On-Premise (Local Lab)
Build your own physical lab with local servers and networking. Best for low latency and privacy.

**Advantages:**
- Ultra-low latency for real-time control
- Complete data privacy and security
- Full hardware control and customization
- No ongoing cloud costs after initial investment

**Disadvantages:**
- High upfront capital expenditure
- Requires IT infrastructure management
- Limited scalability beyond hardware capacity
- Requires physical space and power infrastructure

**Best For:**
- Production robot deployments
- Security-sensitive applications
- Applications requiring guaranteed low latency
- Organizations with existing IT infrastructure

### Cloud-Native
Utilize AWS RoboMaker or Isaac Sim on Azure. Best for scalability and training large models.

**Advantages:**
- High scalability and elasticity
- No upfront hardware investment
- Access to latest GPU hardware
- Built-in CI/CD and monitoring tools

**Disadvantages:**
- Network latency can affect real-time control
- Ongoing operational costs
- Data privacy considerations
- Potential network reliability issues

**Best For:**
- Model training and simulation
- Development and testing environments
- Applications with flexible latency requirements
- Organizations without dedicated IT infrastructure

### Hybrid Approach
Combine the best of both worlds with cloud-based training and edge deployment.

**Advantages:**
- Scalable training with edge deployment
- Optimized for both cost and performance
- Flexibility to adjust based on needs
- Reduced risk of vendor lock-in

**Disadvantages:**
- Increased complexity in architecture
- Requires coordination between environments
- Potential data synchronization challenges
- More complex monitoring and debugging

**Best For:**
- Most Physical AI applications
- Organizations with mixed requirements
- Applications requiring both training and deployment
- Teams with distributed development needs

## Deployment Strategies

### 1. Blue-Green Deployment
Maintain two identical environments and switch between them for zero-downtime updates.

### 2. Canary Deployment
Gradually roll out changes to a subset of robots before full deployment.

### 3. Rolling Deployment
Update robots one at a time to maintain overall system availability.

## Cost Considerations

### On-Premise Costs
| Component | Initial Cost | Annual Cost |
|-----------|--------------|-------------|
| Workstation (RTX 4080) | $2,500 | $0 |
| Jetson Orin Nano | $250 | $0 |
| Networking Equipment | $1,000 | $200 |
| Power & Cooling | $0 | $800 |
| Maintenance | $500 | $1,000 |

### Cloud Costs
| Service | Monthly Cost (per robot) |
|---------|--------------------------|
| GPU Instance (g5.xlarge) | $724 |
| Storage (500GB) | $15 |
| Data Transfer | $10 |
| Management Tools | $25 |
| **Total** | **~$774** |

## Performance Metrics

### Latency Requirements
- **Control Loop**: &lt;10ms for stable robot control
- **Perception Pipeline**: &lt;50ms for real-time response
- **AI Inference**: &lt;100ms for acceptable responsiveness
- **Cloud Communication**: &lt;500ms for basic monitoring

### Throughput Requirements
- **Sensor Data**: 10-100 MB/s for multi-modal perception
- **Model Updates**: 1-10 GB for large VLA models
- **Logging**: 1-10 MB/s for comprehensive debugging

## Security Considerations

### Data Protection
- Encrypt data in transit and at rest
- Implement secure key management
- Regular security audits and updates

### Access Control
- Role-based access control (RBAC)
- Multi-factor authentication (MFA)
- Network segmentation and firewalls

### Compliance
- GDPR for European deployments
- SOC 2 for data security
- Industry-specific regulations

## Monitoring & Observability

### Key Metrics
- Robot uptime and availability
- Model inference performance
- Sensor data quality
- Network connectivity status

### Logging Strategy
- Structured logging for analysis
- Centralized log aggregation
- Real-time alerting for critical issues

## Next Steps

- [On-Premise Setup](/docs/deployment/on-premise) - Detailed local lab configuration
- [Cloud-Native Setup](/docs/deployment/cloud) - Cloud infrastructure guide
- [Edge Deployment](/docs/deployment/edge) - Robot deployment guide
- [CI/CD Pipeline](/docs/deployment/cicd) - Automated deployment workflows
- [Robot Lab Setup](/docs/hardware/robot-lab) - Physical robot lab configuration
