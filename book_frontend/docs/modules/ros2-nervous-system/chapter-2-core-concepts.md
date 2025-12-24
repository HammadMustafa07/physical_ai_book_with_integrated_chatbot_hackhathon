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

Nodes are the fundamental building blocks of any ROS 2 system. A node is an executable process that performs a specific function within the robot system. Each node is responsible for a particular task, such as controlling a sensor, processing data, or commanding actuators. Nodes provide a way to organize robot software into modular, manageable components that can be developed, tested, and maintained independently.

Key characteristics of nodes include:

- **Modularity**: Each node performs a single, well-defined function
- **Communication**: Nodes communicate with other nodes through topics, services, and actions
- **Independence**: Nodes run as separate processes and can be started/stopped independently
- **Names**: Each node has a unique name within the ROS 2 system to identify it
- **Lifecycle**: Nodes can be configured with different lifecycle states (unconfigured, inactive, active, finalized)

In Python, you create a node by inheriting from the `rclpy.Node` class:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
```

Nodes can be distributed across multiple machines, allowing for scalable and fault-tolerant robot systems. This distributed architecture is one of the key advantages of ROS 2 over monolithic software architectures.

## Topics

Topics are named channels through which nodes exchange messages in a publish-subscribe pattern. Topics enable asynchronous, decoupled communication between nodes. A node can publish messages to a topic, and any number of other nodes can subscribe to that topic to receive the messages. This allows for flexible, many-to-many communication patterns.

Key aspects of topics include:

- **Publish-Subscribe Pattern**: Publishers send messages to topics without knowing who will receive them
- **Anonymous Communication**: Publishers and subscribers are decoupled - they don't need to know about each other
- **Message Types**: Each topic has a specific message type that defines the structure of the data
- **Data Flow**: Topics are typically used for continuous data streams like sensor readings or actuator commands
- **Quality of Service (QoS)**: Configurable settings for reliability, durability, and performance

Common message types include:
- `std_msgs`: Basic data types (integers, floats, strings)
- `sensor_msgs`: Sensor data (images, laser scans, IMU data)
- `geometry_msgs`: Spatial information (poses, velocities, transforms)
- `nav_msgs`: Navigation-related messages (paths, occupancy grids)

Example of publishing to a topic:

```python
from std_msgs.msg import String

publisher = self.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello World'
self.publisher.publish(msg)
```

## Services

Services provide synchronous, request-response communication between nodes. Unlike topics which are asynchronous and anonymous, services establish a direct connection between a client (requester) and a server (responder). Services are ideal for operations that require a specific response or acknowledgment, such as configuration changes, triggering specific actions, or requesting current status.

Key characteristics of services:

- **Request-Response Pattern**: Client sends a request and waits for a response from the server
- **Synchronous**: Communication is blocking until the response is received
- **Direct Connection**: Client and server are directly coupled during the service call
- **Service Types**: Each service has a defined request and response message structure
- **Reliability**: Service calls typically guarantee delivery and response

Service definitions are stored in `.srv` files that specify both request and response message structures. For example, a simple service might take two integers as input and return their sum.

Example of creating a service server:

```python
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Returning {response.sum}')
    return response

service = self.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
```

## Actions

Actions are a more sophisticated communication pattern that combines features of both topics and services. They're designed for long-running tasks that require feedback, status updates, and the ability to cancel operations. Actions are ideal for tasks like navigation to a goal, trajectory execution, or any operation that takes a significant amount of time to complete.

The action pattern includes three components:

- **Goal**: The request to start a long-running task
- **Feedback**: Periodic updates on the progress of the task
- **Result**: The final outcome of the task when it completes

Key features of actions:

- **Long-running Operations**: Designed for tasks that take time to complete
- **Feedback**: Continuous updates on progress during execution
- **Cancelability**: Ability to cancel a running action
- **Status Information**: Real-time status updates (active, succeeded, canceled, aborted)
- **Preemption**: Ability to replace a running goal with a new one

Actions use the same Quality of Service (QoS) settings as topics but provide additional control mechanisms for managing long-running operations. They're particularly useful in robotics applications where tasks like navigation, manipulation, or complex sensor operations need to be monitored and potentially interrupted.

## Pub/Sub Communication Model

The publish-subscribe (pub/sub) communication model is the backbone of ROS 2's distributed architecture. In this model, publishers send messages to named topics without knowledge of who will receive them, and subscribers receive messages from topics without knowing who published them. This loose coupling allows for flexible, scalable robot systems where components can be added, removed, or replaced without affecting other parts of the system.

The pub/sub model works as follows:

1. **Discovery**: Nodes discover each other through a distributed discovery mechanism
2. **Matching**: Publishers and subscribers with matching topic names and message types are connected
3. **Transport**: Messages are transported using middleware (typically DDS - Data Distribution Service)
4. **Delivery**: Messages are delivered according to Quality of Service policies

Quality of Service (QoS) settings allow fine-tuning of communication behavior:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (history of messages)
- **History**: Keep last N messages vs. keep all messages
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is still active

The pub/sub model enables several important architectural patterns in robotics:

- **Sensor Fusion**: Multiple sensor nodes publish to topics that are consumed by fusion algorithms
- **Control Loops**: Sensor data flows to controllers which publish actuator commands
- **Monitoring**: Diagnostic information published to monitoring nodes
- **Logging**: All topics can be recorded for analysis and replay

## Data Flow Between Sensors and Controllers

The flow of data between sensors and controllers forms the foundation of robot perception and action. This data flow typically follows a pipeline where sensor data is acquired, processed, and then used to make control decisions. Understanding this flow is crucial for designing responsive and reliable robot systems.

The typical data flow pattern includes:

1. **Sensor Acquisition**: Raw data from physical sensors (cameras, IMU, LiDAR, etc.)
2. **Preprocessing**: Calibration, filtering, and initial processing of raw sensor data
3. **Fusion**: Combining data from multiple sensors to create a coherent view
4. **Perception**: Higher-level processing to extract meaningful information
5. **Planning**: Decision making based on perceived information
6. **Control**: Generating commands for actuators based on plans
7. **Actuation**: Physical execution of commands by robot hardware

For example, in a mobile robot navigation system:
- LiDAR sensors publish laser scan data to `/scan` topic
- IMU sensors publish orientation data to `/imu` topic
- Camera sensors publish image data to `/camera/image_raw` topic
- Perception nodes subscribe to these topics and publish processed data
- Planning nodes use processed data to generate navigation plans
- Controller nodes subscribe to plans and publish motor commands to `/cmd_vel` topic

This distributed approach allows for:
- **Modularity**: Each processing step can be developed independently
- **Flexibility**: Different algorithms can be swapped without changing the overall architecture
- **Scalability**: Additional sensors or processing nodes can be added easily
- **Robustness**: Failure in one component doesn't necessarily stop the entire system

## Summary

In this chapter, we've explored the fundamental building blocks of ROS 2:

1. **Nodes** serve as the basic execution units that encapsulate specific robot functionality
2. **Topics** enable asynchronous, anonymous communication through publish-subscribe patterns
3. **Services** provide synchronous request-response communication for direct interactions
4. **Actions** support long-running operations with feedback and cancellation capabilities
5. The **pub/sub model** creates a flexible, distributed communication architecture
6. **Data flow** between sensors and controllers enables the perception-action loop essential for robotics

These core concepts form the foundation for building complex, distributed robot systems. Understanding how nodes, topics, services, and actions work together is essential for developing robust robotic applications that can scale from simple prototypes to complex real-world systems.

---
sidebar_label: Core Concepts
sidebar_position: 2
pagination_next: modules/ros2-nervous-system/chapter-3-ai-to-motion
pagination_prev: modules/ros2-nervous-system/chapter-1-what-is-ros2
keywords: [ROS 2, nodes, topics, services, actions, pub/sub, communication model, data flow, sensors, controllers]
description: Understanding the fundamental building blocks of ROS 2: nodes, topics, services, and actions, and how they work together in the pub/sub communication model.
---

## Learning Objectives Review

- ✅ Understand Nodes, Topics, Services, and Actions
- ✅ Explain the Pub/Sub communication model
- ✅ Describe data flow between sensors and controllers