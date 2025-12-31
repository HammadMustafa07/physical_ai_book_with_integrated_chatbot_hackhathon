---
title: Chapter 2 - Sensor Simulation for Perception
sidebar_position: 2
description: Simulate robot sensors for perception in the digital twin environment
---

# Chapter 2: Sensor Simulation for Perception

## Why Sensor Simulation Matters Before Real Hardware

Sensor simulation is critical for developing AI perception capabilities. Students need realistic sensor data to train computer vision and sensor fusion algorithms before deployment to real hardware. This approach provides:
- Safe testing environment for perception algorithms
- Consistent conditions for algorithm validation
- Cost-effective experimentation
- Foundation for sim-to-real transfer

## Simulated LiDAR: Range, Noise, and Scan Topics

LiDAR simulation in Gazebo provides realistic 2D or 3D range data with appropriate noise characteristics.

### Key Parameters:
- **Range**: Configurable minimum and maximum distances
- **Resolution**: Angular resolution and accuracy
- **Noise**: Realistic noise patterns similar to real sensors
- **Scan Rate**: Configurable update frequency

The simulated LiDAR publishes to standard ROS 2 topics:
- `sensor_msgs/LaserScan` for 2D LiDAR
- `sensor_msgs/PointCloud2` for 3D LiDAR

## Simulated Depth Cameras and RGB-D Pipelines

Depth camera simulation provides both color (RGB) and depth (D) data streams, creating RGB-D data similar to real sensors like Intel RealSense.

### RGB-D Pipeline Components:
- **Color Camera**: Simulates RGB image data
- **Depth Camera**: Simulates depth information
- **Registration**: Aligns RGB and depth data

## IMU Simulation: Orientation, Acceleration, Drift

Inertial Measurement Unit (IMU) simulation provides:
- **Orientation**: Quaternion-based orientation data
- **Angular Velocity**: 3-axis gyroscope readings
- **Linear Acceleration**: 3-axis accelerometer readings
- **Drift Characteristics**: Realistic drift patterns over time

## Validating Sensor Data via ROS 2 Topics

All simulated sensors publish data to standard ROS 2 topics that match real hardware interfaces:

```bash
# Monitor sensor data
ros2 topic echo /your_robot/laser_scan
ros2 topic echo /your_robot/camera/depth/image_raw
ros2 topic echo /your_robot/imu/data
```

