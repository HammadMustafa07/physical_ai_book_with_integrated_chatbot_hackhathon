---
title: Chapter 1 - What is ROS 2?
description: Introduction to ROS 2 as a robotic nervous system
---

# What is ROS 2?

## Learning Objectives
- Explain what ROS 2 is and its role in robotics
- Describe the distributed software model
- Identify the role of ROS 2 in Physical AI and embodied intelligence

## ROS 2 as a Robotic Nervous System

ROS 2 (Robot Operating System 2) functions as the nervous system of a robot, connecting different components and enabling them to communicate effectively. Just as the nervous system connects different parts of the human body and allows them to coordinate, ROS 2 provides a communication infrastructure that allows different robot components—such as sensors, actuators, and processing units—to work together seamlessly.

The key characteristics of ROS 2 as a nervous system include:

- **Distributed Architecture**: Components can run on different machines or processes
- **Message Passing**: Components communicate by exchanging messages through topics, services, and actions
- **Real-time Communication**: Enables responsive behavior in dynamic environments
- **Modularity**: Allows for easy addition or removal of components

## Distributed Robot Software Model

The distributed robot software model in ROS 2 allows different components of a robot system to run on separate processes or even separate machines. This architecture provides several advantages:

- **Scalability**: Components can be distributed across multiple computing resources
- **Fault Tolerance**: Failure in one component doesn't necessarily affect others
- **Flexibility**: Different components can be developed and maintained independently
- **Resource Optimization**: Computationally intensive tasks can be offloaded to appropriate hardware

In this model, each component runs as a separate "node" that communicates with other nodes through messages. This approach contrasts with monolithic architectures where all functionality runs in a single process, making ROS 2 more robust and maintainable for complex robotic systems.

## Role in Physical AI and Embodied Intelligence

ROS 2 plays a crucial role in Physical AI and embodied intelligence by providing the communication layer that connects AI algorithms to physical robot bodies. This connection enables:

- **Real-world Interaction**: AI models can perceive and act in the physical world through robot sensors and actuators
- **Embodied Cognition**: AI systems can learn from physical interactions and environmental feedback
- **Sensorimotor Integration**: Seamless integration between perception, decision-making, and action
- **Adaptive Behavior**: Robots can adapt their behavior based on real-world experiences

Physical AI represents the next evolution of AI, moving beyond digital-only models to systems that understand and operate in the physical world. ROS 2 serves as the middleware that enables this transition by bridging the gap between high-level AI logic and low-level robot control.

## Summary

In this chapter, we've explored the fundamental concepts of ROS 2 as a robotic nervous system. We've learned that:

1. ROS 2 functions as a communication infrastructure connecting different robot components
2. The distributed software model allows for scalability and fault tolerance
3. ROS 2 enables the transition from digital AI to Physical AI and embodied intelligence

These foundational concepts provide the basis for understanding more advanced ROS 2 features covered in subsequent chapters.

## Learning Objectives Review

- ✅ Explain what ROS 2 is and its role in robotics
- ✅ Describe the distributed software model
- ✅ Identify the role of ROS 2 in Physical AI and embodied intelligence