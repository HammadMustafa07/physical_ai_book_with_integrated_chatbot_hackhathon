---
title: Chapter 2 - ROS 2 Core Concepts
description: Understanding the fundamental building blocks of ROS 2
---

# ROS 2 Core Concepts

## Learning Objectives
- Understand Nodes, Topics, Services, and Actions
- Explain the Pub/Sub communication model
- Describe data flow between sensors and controllers

## Nodes

Nodes are the fundamental building blocks of any ROS 2 system. A node is an executable process that performs a specific function within the robot system. Each node is responsible for a particular task, such as controlling a sensor, processing data, or commanding actuators.

Key characteristics of nodes include:
- **Modularity**: Each node performs a single, well-defined function
- **Communication**: Nodes communicate with other nodes through topics, services, and actions
- **Independence**: Nodes run as separate processes and can be started/stopped independently

In Python, you create a node by inheriting from the `rclpy.Node` class:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
```

## Topics

Topics are named channels through which nodes exchange messages in a publish-subscribe pattern for asynchronous communication.

Common message types include:
- `std_msgs`: Basic data types (integers, floats, strings)
- `sensor_msgs`: Sensor data (images, laser scans, IMU data)
- `geometry_msgs`: Spatial information (poses, velocities, transforms)

## Services

Services provide synchronous, request-response communication between nodes.

## Actions

Actions are a more sophisticated communication pattern that combines features of both topics and services for long-running tasks.

## Summary

In this chapter, we've explored the fundamental building blocks of ROS 2:
- **Nodes** serve as the basic execution units that encapsulate specific robot functionality
- **Topics** enable asynchronous, anonymous communication through publish-subscribe patterns
- **Services** provide synchronous request-response communication for direct interactions
- **Actions** support long-running operations with feedback and cancellation capabilities

