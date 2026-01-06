---
title: CI/CD Pipeline
sidebar_label: CI/CD Pipeline
---

# CI/CD Pipeline for Physical AI

Continuous Integration and Continuous Deployment (CI/CD) is essential for maintaining reliable and scalable Physical AI systems. This section covers creating automated pipelines for testing, building, and deploying your robotics applications.

## Overview

CI/CD for Physical AI involves unique challenges compared to traditional software:
- Hardware-in-the-loop testing
- Simulation-based validation
- Resource-intensive builds
- Safety-critical deployment requirements

## CI/CD Architecture

### Pipeline Stages

```
┌─────────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Code Push   │───▶│   Build    │───▶│  Test      │───▶│  Deploy    │
│   (Git)       │    │   & Lint   │    │  & Validate│    │  & Monitor │
└─────────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
        │                       │                   │                   │
        ▼                       ▼                   ▼                   ▼
┌─────────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Pull Request │    │   Docker   │    │ Unit Tests  │    │   Robot     │
│   Review      │    │   Images   │    │ Integration │    │   Update    │
└─────────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

## Git Workflow

### Branch Strategy

Use a Git flow optimized for robotics development:

```bash
# Main branches
main           # Production-ready code
develop        # Integration branch
release/*      # Release preparation
feature/*      # Feature development
hotfix/*       # Critical fixes
```

### Commit Conventions

Follow conventional commits with robotics-specific types:

```bash
# Examples of good commit messages
feat(robot): add new perception pipeline
fix(control): resolve PID controller instability
test(sim): add collision detection tests
docs(hardware): update Jetson setup guide
refactor(ai): optimize neural network inference
```

## Build Pipeline

### Docker Multi-Stage Build

Create optimized Docker images for different environments:

```dockerfile
# Dockerfile for simulation environment
FROM nvidia/cuda:12.2-devel-ubuntu22.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Install Isaac ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-* \
    && rm -rf /var/lib/apt/lists/*

# Set up Python environment
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Copy application code
WORKDIR /app
COPY . .

# Install application
RUN pip3 install -e .

CMD ["python3", "-m", "robot_app"]
```

### Build Optimization

Use build caching and parallel builds:

```yaml
# .gitlab-ci.yml example
stages:
  - build
  - test
  - deploy

variables:
  DOCKER_DRIVER: overlay2
  DOCKER_TLS_CERTDIR: "/certs"

build-simulation:
  stage: build
  image: docker:20.10.16
  services:
    - docker:20.10.16-dind
  before_script:
    - docker login -u $CI_REGISTRY_USER -p $CI_REGISTRY_PASSWORD $CI_REGISTRY
  script:
    - |
      docker build
        --cache-from $CI_REGISTRY_IMAGE:sim-cache
        --target simulation
        -t $CI_REGISTRY_IMAGE:simulation-$CI_COMMIT_SHA
        .
    - docker push $CI_REGISTRY_IMAGE:simulation-$CI_COMMIT_SHA
  after_script:
    - docker rmi $CI_REGISTRY_IMAGE:simulation-$CI_COMMIT_SHA || true
  cache:
    paths:
      - .docker_cache/
    key: "$CI_COMMIT_REF_SLUG"
```

## Testing Pipeline

### Unit Testing

```python
import unittest
import numpy as np
from unittest.mock import Mock, patch

class TestPerceptionPipeline(unittest.TestCase):
    def setUp(self):
        self.perception = PerceptionPipeline()

    def test_depth_estimation_accuracy(self):
        """Test depth estimation accuracy within tolerance"""
        test_image = np.random.rand(480, 640, 3).astype(np.uint8)
        depth_map = self.perception.estimate_depth(test_image)

        # Validate output shape and range
        self.assertEqual(depth_map.shape, (480, 640))
        self.assertTrue(np.all(depth_map >= 0.1))
        self.assertTrue(np.all(depth_map <= 100.0))

    def test_object_detection(self):
        """Test object detection functionality"""
        test_image = np.random.rand(480, 640, 3).astype(np.uint8)
        detections = self.perception.detect_objects(test_image)

        self.assertIsInstance(detections, list)
        # Add more specific assertions based on your detection format

class TestControlSystem(unittest.TestCase):
    def setUp(self):
        self.controller = PIDController(kp=1.0, ki=0.1, kd=0.05)

    def test_stability(self):
        """Test controller stability with various inputs"""
        for target in [0.0, 1.0, -1.0]:
            for current in [0.0, 0.5, -0.5]:
                output = self.controller.compute(target, current, dt=0.01)
                # Controller should not produce extreme outputs
                self.assertLess(abs(output), 100.0)

if __name__ == '__main__':
    unittest.main()
```

### Integration Testing

```python
import pytest
import subprocess
import time
from pathlib import Path

class TestSimulationIntegration:
    @pytest.fixture(scope="class")
    def simulation_env(self):
        """Setup simulation environment for integration tests"""
        # Start Isaac Sim or Gazebo in headless mode
        sim_process = subprocess.Popen([
            "isaac-sim",
            "--/headless",
            "--/renderer=Vulkan"
        ])

        # Wait for simulation to start
        time.sleep(10)

        yield sim_process

        # Cleanup
        sim_process.terminate()
        sim_process.wait()

    def test_robot_navigation(self, simulation_env):
        """Test complete navigation pipeline in simulation"""
        # Run navigation test
        result = subprocess.run([
            "ros2", "launch",
            "navigation_test.launch.py"
        ], capture_output=True, text=True)

        assert result.returncode == 0
        assert "Navigation successful" in result.stdout

    def test_perception_pipeline(self, simulation_env):
        """Test perception pipeline with simulated sensors"""
        # Test perception with simulated camera data
        result = subprocess.run([
            "python3", "test_perception.py",
            "--simulated"
        ], capture_output=True, text=True)

        assert result.returncode == 0
        assert "Perception accuracy: 0.95" in result.stdout
```

### Hardware-in-the-Loop Testing

```python
import unittest
from unittest.mock import Mock, patch
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class TestHardwareIntegration(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_robot_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.received_image = None
        self.received_laser = None

    def image_callback(self, msg):
        self.received_image = msg

    def laser_callback(self, msg):
        self.received_laser = msg

    def test_sensor_data_flow(self):
        """Test that sensor data flows correctly through the system"""
        # Wait for sensor data
        timeout = rospy.Duration(5.0)  # 5 seconds
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time) < timeout:
            if self.received_image is not None and self.received_laser is not None:
                break
            rospy.sleep(0.1)

        self.assertIsNotNone(self.received_image, "No image received")
        self.assertIsNotNone(self.received_laser, "No laser scan received")

    def test_control_output(self):
        """Test that control commands are published correctly"""
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.5

        self.cmd_vel_pub.publish(cmd)

        # Verify command was published (this would require mocking the publisher)
        # In real testing, you'd verify with a test robot or simulation
        pass
```

## Deployment Pipeline

### Containerized Deployment

```yaml
# docker-compose.yml for robot deployment
version: '3.8'

services:
  perception:
    image: ${REGISTRY}/robot-perception:${TAG}
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    environment:
      - CUDA_VISIBLE_DEVICES=0
      - MODEL_PATH=/models/perception_model.trt
    volumes:
      - /tmp:/tmp
      - robot-models:/models
    devices:
      - /dev/video0:/dev/video0
    networks:
      - robot-net
    restart: unless-stopped

  control:
    image: ${REGISTRY}/robot-control:${TAG}
    environment:
      - CONTROL_CONFIG=/config/control.yaml
    volumes:
      - robot-config:/config
      - /dev:/dev
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # Motor controller
    networks:
      - robot-net
    restart: unless-stopped

  monitoring:
    image: ${REGISTRY}/robot-monitoring:${TAG}
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
    ports:
      - "3000:3000"  # Grafana
    networks:
      - robot-net
    restart: unless-stopped

networks:
  robot-net:
    driver: bridge

volumes:
  robot-models:
  robot-config:
```

### Deployment Scripts

```bash
#!/bin/bash
# deploy-robot.sh

set -e  # Exit on any error

# Configuration
ROBOT_IP=${ROBOT_IP:-"192.168.1.100"}
ROBOT_USER=${ROBOT_USER:-"robot"}
SSH_KEY=${SSH_KEY:-"~/.ssh/robot_key"}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check prerequisites
check_prerequisites() {
    log "Checking prerequisites..."

    if ! command -v docker &> /dev/null; then
        error "Docker is not installed"
        exit 1
    fi

    if ! command -v docker-compose &> /dev/null; then
        error "Docker Compose is not installed"
        exit 1
    fi
}

# Function to build images
build_images() {
    log "Building Docker images..."

    # Build simulation image
    docker build -t robot-simulation:${BUILD_TAG} \
        --target simulation \
        -f Dockerfile .

    # Build edge deployment image
    docker build -t robot-edge:${BUILD_TAG} \
        --target edge \
        -f Dockerfile .

    log "Images built successfully"
}

# Function to run tests
run_tests() {
    log "Running tests..."

    # Run unit tests
    python -m pytest tests/unit/ -v

    # Run integration tests (in simulation)
    docker-compose -f docker-compose.test.yml up --abort-on-container-exit

    log "All tests passed"
}

# Function to deploy to robot
deploy_to_robot() {
    log "Deploying to robot at ${ROBOT_IP}..."

    # Copy images to robot
    log "Copying Docker images to robot..."
    docker save robot-edge:${BUILD_TAG} | \
        ssh -i ${SSH_KEY} ${ROBOT_USER}@${ROBOT_IP} \
        "docker load"

    # Copy deployment files
    scp -i ${SSH_KEY} docker-compose.yml ${ROBOT_USER}@${ROBOT_IP}:~/robot/
    scp -i ${SSH_KEY} .env ${ROBOT_USER}@${ROBOT_IP}:~/robot/

    # Deploy on robot
    ssh -i ${SSH_KEY} ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
        cd ~/robot
        docker-compose down
        docker-compose up -d
        docker system prune -f
EOF

    log "Deployment completed"
}

# Function to verify deployment
verify_deployment() {
    log "Verifying deployment..."

    # Check if services are running
    ssh -i ${SSH_KEY} ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
        docker-compose ps
        if [ $? -ne 0 ]; then
            echo "Failed to check service status"
            exit 1
        fi

        # Check service logs for errors
        docker-compose logs perception | tail -20
        docker-compose logs control | tail -20
EOF

    log "Deployment verification completed"
}

# Main execution
main() {
    log "Starting robot deployment pipeline"

    check_prerequisites
    build_images
    run_tests
    deploy_to_robot
    verify_deployment

    log "Deployment pipeline completed successfully!"
}

# Parse command line arguments
case "${1:-}" in
    build)
        build_images
        ;;
    test)
        run_tests
        ;;
    deploy)
        deploy_to_robot
        verify_deployment
        ;;
    *)
        main
        ;;
esac
```

## Monitoring and Observability

### Pipeline Monitoring

```yaml
# monitoring/pipeline-monitoring.yml
apiVersion: v1
kind: ConfigMap
metadata:
  name: pipeline-monitoring
data:
  prometheus.yml: |
    global:
      scrape_interval: 15s
    scrape_configs:
      - job_name: 'ci-pipeline'
        static_configs:
          - targets: ['jenkins:9100', 'gitlab:9100']
      - job_name: 'robot'
        static_configs:
          - targets: ['robot-control:9100', 'robot-perception:9100']

---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: pipeline-monitoring
spec:
  replicas: 1
  selector:
    matchLabels:
      app: pipeline-monitoring
  template:
    metadata:
      labels:
        app: pipeline-monitoring
    spec:
      containers:
        - name: prometheus
          image: prom/prometheus:latest
          ports:
            - containerPort: 9090
          volumeMounts:
            - name: config
              mountPath: /etc/prometheus/
        - name: grafana
          image: grafana/grafana:latest
          ports:
            - containerPort: 3000
          env:
            - name: GF_SECURITY_ADMIN_PASSWORD
              value: "admin"
      volumes:
        - name: config
          configMap:
            name: pipeline-monitoring
```

## Security in CI/CD

### Secret Management

```yaml
# .github/workflows/deploy.yml
name: Deploy to Robot

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v2

      - name: Login to Container Registry
        uses: docker/login-action@v2
        with:
          registry: ${{ secrets.CONTAINER_REGISTRY }}
          username: ${{ secrets.REGISTRY_USERNAME }}
          password: ${{ secrets.REGISTRY_PASSWORD }}

      - name: Build and push
        uses: docker/build-push-action@v4
        with:
          context: .
          push: true
          tags: ${{ secrets.CONTAINER_REGISTRY }}/robot:${{ github.sha }}

      - name: Deploy to Robot
        uses: appleboy/ssh-action@v0.1.5
        with:
          host: ${{ secrets.ROBOT_HOST }}
          username: robot
          key: ${{ secrets.ROBOT_SSH_KEY }}
          script: |
            docker pull ${{ secrets.CONTAINER_REGISTRY }}/robot:${{ github.sha }}
            docker tag ${{ secrets.CONTAINER_REGISTRY }}/robot:${{ github.sha }} robot:latest
            docker-compose down
            docker-compose up -d
```

## Rollback Strategy

```bash
#!/bin/bash
# rollback.sh

# Configuration
DEPLOYMENT_DIR="/opt/robot"
BACKUP_DIR="/opt/robot-backups"
TIMESTAMP=$(date -u +"%Y%m%d_%H%M%S")

rollback() {
    if [ -z "$1" ]; then
        echo "Usage: $0 <backup_timestamp>"
        echo "Available backups:"
        ls $BACKUP_DIR/
        exit 1
    fi

    BACKUP_TIMESTAMP=$1
    BACKUP_PATH="$BACKUP_DIR/backup_$BACKUP_TIMESTAMP"

    if [ ! -d "$BACKUP_PATH" ]; then
        echo "Backup $BACKUP_PATH does not exist"
        exit 1
    fi

    echo "Rolling back to backup: $BACKUP_TIMESTAMP"

    # Stop current deployment
    docker-compose -f $DEPLOYMENT_DIR/docker-compose.yml down

    # Restore from backup
    cp -r $BACKUP_PATH/* $DEPLOYMENT_DIR/

    # Restart services
    docker-compose -f $DEPLOYMENT_DIR/docker-compose.yml up -d

    echo "Rollback completed"
}

create_backup() {
    echo "Creating backup: $TIMESTAMP"

    # Create backup directory
    mkdir -p $BACKUP_DIR/backup_$TIMESTAMP

    # Backup current deployment
    cp -r $DEPLOYMENT_DIR/* $BACKUP_DIR/backup_$TIMESTAMP/

    # Backup Docker images
    docker images --format "table {{.Repository}}:{{.Tag}}" | grep robot | while read image; do
        docker save $image -o $BACKUP_DIR/backup_$TIMESTAMP/$(echo $image | tr ':' '_').tar
    done

    echo "Backup created: $TIMESTAMP"
}

case "${1:-}" in
    create)
        create_backup
        ;;
    rollback)
        rollback $2
        ;;
    *)
        echo "Usage: $0 {create|rollback [timestamp]}"
        ;;
esac
```

## Performance Optimization

### Caching Strategies

```yaml
# .gitlab-ci.yml with caching
stages:
  - build
  - test
  - deploy

variables:
  PIP_CACHE_DIR: "$CI_PROJECT_DIR/.cache/pip"

cache:
  paths:
    - .cache/pip
    - venv/
    - node_modules/
    - build/
  key: "$CI_COMMIT_REF_SLUG"

before_script:
  - python -v
  - pip install virtualenv
  - virtualenv venv
  - source venv/bin/activate

build:
  stage: build
  script:
    - pip install -r requirements.txt
    - python setup.py build
  artifacts:
    paths:
      - build/
    expire_in: 1 week
```

## Best Practices

### 1. Automated Testing at Every Stage

- Unit tests for individual components
- Integration tests for subsystems
- Simulation tests for complete behaviors
- Hardware tests for deployment validation

### 2. Gradual Rollouts

- Canary deployments to a subset of robots
- Feature flags for safe rollbacks
- Blue-green deployments for zero-downtime updates

### 3. Comprehensive Monitoring

- Application performance metrics
- Hardware resource utilization
- Model accuracy tracking
- Safety system status

### 4. Security Integration

- Static code analysis
- Dependency vulnerability scanning
- Container security scanning
- Secrets management

## Next Steps

- [On-Premise Setup](/docs/deployment/on-premise) - Local lab configuration
- [Cloud-Native Setup](/docs/deployment/cloud) - Cloud infrastructure
- [Edge Deployment](/docs/deployment/edge) - Robot deployment guide
- [Hardware Requirements](/docs/hardware/requirements) - System specifications