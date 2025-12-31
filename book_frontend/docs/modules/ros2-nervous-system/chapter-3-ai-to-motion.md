---
title: Chapter 3 - From AI to Motion
description: Connecting Python AI agents to robot controllers
---

# From AI to Motion

## Learning Objectives
- Connect Python AI agents via rclpy
- Understand controllers and actuators overview
- Learn URDF basics for humanoid structure

## Connecting Python AI Agents via rclpy

The `rclpy` library is the Python client library for ROS 2, enabling Python-based AI agents to communicate with the ROS 2 ecosystem and control robot hardware. It provides APIs for creating nodes, publishing/subscribing to topics, making service calls, and managing actions.

Key components of rclpy include:
- **Node Creation**: The fundamental building block for ROS 2 Python programs
- **Publishers and Subscribers**: For asynchronous communication via topics
- **Clients and Servers**: For synchronous communication via services
- **Action Clients and Servers**: For long-running operations with feedback

To get started with rclpy:
```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = MyAIAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Controllers and Actuators Overview

Controllers and actuators form the bridge between high-level AI commands and physical robot motion.

Actuators are the physical components that generate motion or force in a robot:
- **Rotary Servos**: Precise angular position control
- **DC Motors**: Continuous rotation with variable speed
- **Stepper Motors**: Precise incremental movement
- **Linear Actuators**: Straight-line motion

Controllers translate high-level commands into precise actuator control signals.

## URDF Basics for Humanoid Structure

URDF (Unified Robot Description Format) is an XML-based format that describes robot models in ROS for humanoid robots.

## Summary

In this chapter, we've covered the essential concepts for connecting AI agents to robot motion:
- **rclpy Integration**: Python client library that enables AI agents to communicate with ROS 2
- **Controllers and Actuators**: The bridge between AI commands and physical robot motion
- **URDF for Humanoids**: Robot description format that defines physical structure

