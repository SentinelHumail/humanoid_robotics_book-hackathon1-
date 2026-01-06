---
sidebar_position: 2
title: Cloud-Native Setup
---

# Cloud-Native Setup

Deploy your Physical AI lab using cloud resources.

## Overview

The cloud-native approach runs simulation and training in the cloud while using edge kits for physical deployment.

```
┌─────────────────────────────────────────────────────────────┐
│                     Cloud-Native Architecture                 │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                   Cloud Region                       │    │
│  │   ┌─────────────┐    ┌─────────────────────────┐   │    │
│  │   │  Instance   │    │   Isaac Sim / Gazebo    │   │    │
│  │   │  (AWS/Azure)│───▶│   Running in container  │   │    │
│  │   └─────────────┘    └─────────────────────────┘   │    │
│  │          │                                       │    │
│  │          │ HTTPS/API                             │    │
│  │          ▼                                       │    │
│  └─────────────────────────────────────────────────────┘    │
│                    │                                     │
│                    │ Internet                           │
│                    ▼                                     │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                    Edge Kit                          │    │
│  │   ┌─────────────┐    ┌─────────────────────────┐   │    │
│  │   │ Jetson Orin │◀───│   Model Deployment      │   │    │
│  │   │   Nano      │    │   (Downloaded nightly)  │   │    │
│  │   └─────────────┘    └─────────────────────────┘   │    │
│  │          │                                       │    │
│  │          ▼                                       │    │
│  │   ┌─────────────┐                                │    │
│  │   │   Robot     │                                │    │
│  │   │   (Go2)     │                                │    │
│  │   └─────────────┘                                │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Cloud Provider Setup

### AWS Configuration

```yaml
# cloudformation-template.yaml
AWSTemplateFormatVersion: '2010-09-09'
Description: 'Physical AI Cloud Lab'

Resources:
  # GPU Instance for Simulation
  SimulationInstance:
    Type: AWS::EC2::Instance
    Properties:
      InstanceType: g5.2xlarge  # A10G GPU, 24GB VRAM
      ImageId: ami-0c02fb7332af6f31b  # Ubuntu 22.04
      KeyName: physical-ai-key
      SecurityGroupIds:
        - !Ref SimulationSecurityGroup
      IamInstanceProfile: !Ref SimulationInstanceProfile
      BlockDeviceMappings:
        - DeviceName: /dev/sda1
          Ebs:
            VolumeSize: 200
            VolumeType: gp3

  # Auto Scaling Group
  SimulationASG:
    Type: AWS::AutoScaling::AutoScalingGroup
    Properties:
      AutoScalingGroupName: physical-ai-sim
      LaunchConfigurationName: !Ref SimulationLaunchConfig
      MinSize: 0
      MaxSize: 10
      DesiredCapacity: 1
      VPCZoneIdentifier: !Ref SubnetId

Parameters:
  InstanceType:
    Type: String
    Default: g5.2xlarge
    AllowedValues:
      - g4dn.xlarge
      - g5.xlarge
      - g5.2xlarge
      - g6.xlarge

Outputs:
  SimulationURL:
    Description: 'URL for simulation access'
    Value: !Sub 'https://${SimulationInstance.PublicDnsName}:8080'
```

### Azure Configuration

```bash
# Create GPU VM
az vm create \
  --resource-group physical-ai-rg \
  --name simulation-vm \
  --image Ubuntu2204 \
  --size Standard_NC4as_T4_v3 \
  --admin-username azureuser \
  --ssh-key-values ~/.ssh/id_rsa.pub \
  --assign-identity \
  --role contributor \
  --scope /subscriptions/$(az account show --query id -o tsv)/resourceGroups/physical-ai-rg
```

## Cost Estimation

### AWS Pricing (US East)

| Instance | GPU | VRAM | $/hour | $/month (720 hrs) |
|----------|-----|------|--------|-------------------|
| g4dn.xlarge | T4 | 16GB | $0.526 | $379 |
| g5.xlarge | A10G | 24GB | $1.006 | $724 |
| **g5.2xlarge** | **A10G** | **48GB** | **$1.51** | **$1,087** |
| g6.xlarge | L40S | 48GB | $1.77 | $1,274 |

### Cost Breakdown (5 Students)

| Item | Cost |
|------|------|
| Cloud Instances (g5.2xlarge, 10 hrs/week × 12 weeks) | $906 |
| Storage (EBS volumes) | $150 |
| Data Transfer | $50 |
| **Total Cloud Cost/Quarter** | **$1,106** |

### Total Cost Comparison

| Approach | Upfront | Quarterly | Year 1 |
|----------|---------|-----------|--------|
| On-Premise (5 students) | $28,250 | $2,000 | $34,250 |
| Cloud-Native (5 students) | $3,500 (edge kits) | $1,300 | $8,600 |

## Cloud-Native Workflow

### Phase 1: Training (Cloud)

Students use cloud instances for:
- Running Isaac Sim
- Training VLA models
- Generating synthetic data
- Building large simulations

### Phase 2: Deployment (Edge)

After training, models are:
- Optimized for inference (TensorRT)
- Deployed to edge kits
- Tested with real robots
- Iterated based on results

### Daily Workflow

```bash
#!/bin/bash
# student-workflow.sh

# Morning: Connect to cloud instance
ssh student@cloud-physical-ai.example.com

# Resume training
cd /home/student/robot-training
git pull
python train.py --config config/vla_model.yaml

# Afternoon: Deploy to edge kit
scp trained_model.pt student@edge-kit:/home/student/models/
ssh student@edge-kit "python deploy.py --model /home/student/models/trained_model.pt"
```

## Performance Considerations

### Latency Impact

| Operation | Local | Cloud | Difference |
|-----------|-------|-------|------------|
| Isaac Sim render | 16ms | 25ms | +9ms |
| VSLAM tracking | 10ms | 18ms | +8ms |
| Inference (TensorRT) | 5ms | 12ms | +7ms |
| **Total Cycle** | **31ms** | **55ms** | **+24ms** |

### Mitigation Strategies

1. **Batch Processing**: Run heavy computations in cloud
2. **Model Optimization**: Use TensorRT for edge deployment
3. **Caching**: Cache common assets locally
4. **Hybrid Approach**: Keep real-time components local

## Security Considerations

### Network Security

```yaml
# Security Group Rules (AWS)
- Inbound:
  - Port 22 (SSH): 10.0.0.0/16 (lab network)
  - Port 8888 (Jupyter): 10.0.0.0/16
- Outbound:
  - Port 443 (HTTPS): 0.0.0.0/0
  - Port 8888: 10.0.0.0/16 (edge kit communication)
```

### Data Security

- Encrypt data at rest and in transit
- Use VPC with private subnets
- Implement IAM roles with least privilege
- Regular security audits

## Recommended Configuration

### For Individual Learners

| Component | Configuration |
|-----------|---------------|
| Cloud Instance | g5.xlarge (spot) |
| Storage | 200GB gp3 SSD |
| Edge Kit | Jetson Orin Nano Super |
| Robot | Optional (Go2) |
| Monthly Cost | ~$400 (with spot pricing) |

### For Small Classes (5-10)

| Component | Configuration |
|-----------|---------------|
| Cloud Instance | g5.2xlarge (on-demand + spot) |
| Storage | 500GB per student |
| Edge Kit | Jetson Orin Nano Super (per student) |
| Shared Robot | 2x Unitree Go2 |
| Monthly Cost | ~$800-1200 |

## Next Steps

- [On-Premise Setup](/docs/deployment/on-premise) - Physical lab alternative
- [Hardware Comparison](/docs/hardware/comparison) - Cost analysis
- [Edge Kit Setup](/docs/hardware/edge-kit) - Configure your Jetson

---

## Expanded: Automation, Cost Controls, and Reproducible Cloud Labs

### Automation: Terraform + Packer

Use `Packer` to build a reproducible machine image and `Terraform` to provision cloud infrastructure. This reduces configuration drift and simplifies lab teardown.

Example Terraform resource snippet for an AWS GPU instance:

```hcl
resource "aws_instance" "sim" {
  ami           = var.ami
  instance_type = var.instance_type
  key_name      = var.key_pair
  tags = {
    Name = "physical-ai-sim"
  }
}
```

### Cost Controls

- Use spot instances for non-critical workloads (training) and on-demand instances for lab demos.
- Implement auto-scaling groups that scale to zero when not in use and warm up a pool of instances before scheduled lab hours.

### Reproducible Student Workspaces

1. Build a golden VM image with `Packer` that includes Isaac Sim, ROS 2, and required dependencies.
2. Provide students with containerized dev environments (`docker-compose`) that map to the golden image to ensure consistent behavior.

### CI for Cloud Labs

Create a CI pipeline that:
1. Builds the student package with `colcon`.
2. Runs headless Isaac smoke tests in a container.
3. Runs lightweight integration tests that replay `ros2 bag` samples.

Would you like me to add example `Packer` and `Terraform` configs under `infrastructure/` in the repository? 
