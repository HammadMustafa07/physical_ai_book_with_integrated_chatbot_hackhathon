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

The `rclpy` library is the Python client library for ROS 2, enabling Python-based AI agents to communicate with the ROS 2 ecosystem and control robot hardware. It provides the essential APIs for creating nodes, publishing/subscribing to topics, making service calls, and managing actions - all of which are critical for connecting AI logic to physical robot behavior.

Key components of rclpy include:

- **Node Creation**: The fundamental building block for ROS 2 Python programs
- **Publishers and Subscribers**: For asynchronous communication via topics
- **Clients and Servers**: For synchronous communication via services
- **Action Clients and Servers**: For long-running operations with feedback
- **Parameter Management**: For runtime configuration of nodes
- **Timers and Callbacks**: For managing execution timing and event handling

To get started with rclpy, you must first initialize the ROS 2 Python client library:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = MyAIAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class MyAIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        # Node initialization code here
```

When connecting AI agents to ROS 2, consider these best practices:

- **Initialization Order**: Initialize rclpy before creating any nodes
- **Threading Model**: Understand the single-threaded executor model by default
- **Resource Management**: Always properly destroy nodes and shut down rclpy
- **Error Handling**: Implement proper exception handling for ROS communication
- **Logging**: Use the built-in logging system for debugging and monitoring

A typical AI agent node might subscribe to sensor data, process it through AI algorithms, and publish commands to robot controllers:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        # Publish commands to robot
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Timer for AI processing loop
        self.timer = self.create_timer(0.1, self.ai_processing_loop)

    def laser_callback(self, msg):
        # Store sensor data for AI processing
        self.laser_data = msg

    def ai_processing_loop(self):
        if hasattr(self, 'laser_data'):
            # Apply AI algorithm to sensor data
            command = self.apply_ai_logic(self.laser_data)
            # Publish command to robot
            self.publisher.publish(command)

    def apply_ai_logic(self, sensor_data):
        # Your AI algorithm implementation here
        cmd = Twist()
        cmd.linear.x = 0.5  # Example: move forward
        cmd.angular.z = 0.0 # Example: no rotation
        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This pattern allows AI agents to operate as ROS 2 nodes, receiving sensor inputs, applying intelligent processing, and sending commands to robot controllers. The asynchronous nature of ROS 2 topics makes it ideal for real-time AI applications that need to respond to sensor data continuously.

## Controllers and Actuators Overview

Controllers and actuators form the bridge between high-level AI commands and physical robot motion. Understanding this layer is crucial for AI developers who want their algorithms to have real-world impact on robot behavior.

### Actuators

Actuators are the physical components that generate motion or force in a robot. Common types include:

- **Rotary Servos**: Precise angular position control, commonly used in robot joints
- **DC Motors**: Continuous rotation with variable speed and direction
- **Stepper Motors**: Precise incremental movement control
- **Linear Actuators**: Straight-line motion for pushing/pulling applications
- **Pneumatic/Hydraulic Systems**: High-force applications requiring rapid response

Each actuator type has specific characteristics that affect how AI agents should interface with them:

- **Resolution**: The smallest increment of movement possible
- **Speed Range**: Minimum and maximum operational speeds
- **Torque/Force Limits**: Maximum output capability
- **Response Time**: How quickly the actuator responds to commands
- **Power Requirements**: Electrical, pneumatic, or hydraulic needs

### Controllers

Controllers are the software and hardware systems that translate high-level commands into precise actuator control signals. They operate at different levels in the control hierarchy:

**High-Level Controllers**: These receive AI-generated commands and plan trajectories or motion sequences. They might handle path planning, obstacle avoidance, or task-level commands.

**Low-Level Controllers**: These execute precise motor commands, often using PID (Proportional-Integral-Derivative) control or more advanced control algorithms to ensure actuators follow desired trajectories accurately.

**Joint Controllers**: These manage individual robot joints, ensuring each follows its commanded position, velocity, or effort profile.

**Effort Controllers**: These control the force or torque applied by actuators, useful for manipulation tasks requiring precise force control.

### ROS 2 Controller Architecture

ROS 2 uses the `ros2_control` framework to manage robot controllers. This framework provides:

- **Hardware Abstraction Layer**: Standardized interfaces to different hardware platforms
- **Controller Manager**: Runtime management of active controllers
- **Real-time Safety**: Built-in safety mechanisms and emergency stops
- **Configuration Flexibility**: Dynamic loading and switching of controllers

The controller architecture typically follows this pattern:
1. AI Agent (Node) - Generates high-level commands
2. Controller Manager - Manages which controllers are active
3. Individual Controllers - Translate commands to hardware-specific signals
4. Hardware Interface - Communicates directly with physical actuators

### Control Strategies

Different control strategies are appropriate for different AI applications:

- **Position Control**: For precise positioning tasks
- **Velocity Control**: For smooth motion at specified speeds
- **Effort/Torque Control**: For force-sensitive manipulation
- **Impedance Control**: For compliant interaction with the environment
- **Admittance Control**: For variable stiffness behaviors

## URDF Basics for Humanoid Structure

URDF (Unified Robot Description Format) is an XML-based format that describes robot models in ROS. For humanoid robots, URDF defines the physical structure, kinematic chain, and visual/collision properties. Understanding URDF is essential for AI agents that need to work with robot models and understand spatial relationships.

### URDF Structure

A URDF file contains several key elements:

- **Links**: Represent rigid bodies of the robot (e.g., torso, arms, legs)
- **Joints**: Define how links connect and move relative to each other
- **Materials**: Define visual appearance properties
- **Transmissions**: Define how actuators connect to joints
- **Gazebo Plugins**: Simulation-specific extensions

### Basic Humanoid URDF Components

A humanoid robot typically includes:

- **Torso**: The main body with head, arms, and legs attached
- **Head**: Usually with cameras or other sensors
- **Arms**: With shoulders, elbows, wrists, and hands/ends
- **Legs**: With hips, knees, and ankles for locomotion
- **Base**: Often the pelvis or connection point to the world

### Link Definition

Each link in a humanoid URDF defines physical properties:

```xml
<link name="torso">
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.2 0.4"/>
    </geometry>
    <material name="grey">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.2 0.4"/>
    </geometry>
  </collision>
</link>
```

### Joint Definition

Joints define how links connect and move:

```xml
<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
</joint>
```

### Common Joint Types for Humanoids

- **Revolute**: Rotational joint with limited range (e.g., elbow, knee)
- **Continuous**: Rotational joint without limits (e.g., wheel)
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement (e.g., camera mount)
- **Floating**: 6DOF movement (rarely used)

### URDF for AI Applications

AI agents can use URDF information for:

- **Forward Kinematics**: Calculating end-effector positions from joint angles
- **Inverse Kinematics**: Calculating joint angles for desired end-effector positions
- **Collision Detection**: Understanding when parts of the robot might collide
- **Visualization**: Rendering the robot in simulation or debugging tools
- **Motion Planning**: Planning paths that respect joint limits and collisions

### Robot State Publisher

The `robot_state_publisher` node in ROS 2 uses URDF to publish the current state of all joints, allowing AI agents to know the current configuration of the robot. This is crucial for AI algorithms that need to understand the robot's current pose.

## Practical Examples

Let's explore practical examples of connecting AI agents to robot motion using Python and ROS 2. These examples demonstrate the integration of AI logic with physical robot control.

### Example 1: Simple Navigation AI

This example shows a basic AI agent that uses sensor data to navigate safely:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class NavigationAINode(Node):
    def __init__(self):
        super().__init__('navigation_ai')

        # Create subscription to laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.2, self.navigate)

        self.laser_data = None
        self.safe_distance = 1.0  # meters

    def scan_callback(self, msg):
        self.laser_data = msg

    def navigate(self):
        if self.laser_data is None:
            return

        # AI logic: simple obstacle avoidance
        min_distance = min(self.laser_data.ranges)

        cmd = Twist()

        if min_distance > self.safe_distance:
            # Move forward safely
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        else:
            # Turn to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    ai_node = NavigationAINode()
    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Using AI Models with ROS 2

This example demonstrates how to integrate a machine learning model with ROS 2 for more sophisticated behavior:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionAINode(Node):
    def __init__(self):
        super().__init__('vision_ai')

        # Create subscription to camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Timer for AI processing
        self.timer = self.create_timer(0.1, self.process_vision)

        self.cv_bridge = CvBridge()
        self.latest_image = None

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_vision(self):
        if self.latest_image is None:
            return

        # Simple AI: detect red objects and move toward them
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours of red objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cmd = Twist()

        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 1000:  # Only react to significant objects
                # Get center of contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    image_center = self.latest_image.shape[1] / 2

                    # Turn toward the object
                    if cx < image_center - 50:
                        cmd.angular.z = 0.5  # Turn left
                    elif cx > image_center + 50:
                        cmd.angular.z = -0.5  # Turn right
                    else:
                        cmd.linear.x = 0.3  # Move forward
                else:
                    cmd.angular.z = 0.1  # Small random turn if no moment
        else:
            cmd.angular.z = 0.1  # Small random turn if no red object found

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionAINode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Humanoid Joint Control

This example shows how to control humanoid robot joints using ROS 2:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math

class HumanoidControlNode(Node):
    def __init__(self):
        super().__init__('humanoid_control')

        # Publisher for joint trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # Subscriber for current joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Timer for periodic movement
        self.timer = self.create_timer(2.0, self.send_trajectory_command)

        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

    def joint_state_callback(self, msg):
        # Store current joint positions
        self.current_positions = dict(zip(msg.name, msg.position))

    def send_trajectory_command(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create a trajectory point
        point = JointTrajectoryPoint()

        # Define target positions (simple walking gait)
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        # Oscillate joints to create walking motion
        point.positions = [
            0.1 * math.sin(t),      # left_hip
            0.2 * math.sin(t + 0.5), # left_knee
            0.1 * math.sin(t + 1.0), # left_ankle
            0.1 * math.sin(t + 0.5), # right_hip
            0.2 * math.sin(t),       # right_knee
            0.1 * math.sin(t + 0.5)  # right_ankle
        ]

        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)

        # Set timing (execute in 1 second)
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory_msg.points = [point]

        self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    humanoid_node = HumanoidControlNode()
    rclpy.spin(humanoid_node)
    humanoid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These examples demonstrate how AI agents can be integrated with ROS 2 to control robot behavior. The key is to use ROS 2's communication patterns (topics, services, actions) to connect AI algorithms with robot hardware.

## Summary

In this chapter, we've covered the essential concepts for connecting AI agents to robot motion:

1. **rclpy Integration**: Python client library that enables AI agents to communicate with ROS 2
2. **Controllers and Actuators**: The bridge between AI commands and physical robot motion
3. **URDF for Humanoids**: Robot description format that defines physical structure and properties
4. **Practical Examples**: Real-world implementations showing AI-robot integration

The connection between AI and physical motion is fundamental to Physical AI and embodied intelligence. By understanding how to properly interface AI algorithms with robot controllers and hardware, you can create intelligent systems that operate effectively in the physical world.

These concepts form the foundation for creating AI agents that can perceive their environment, make intelligent decisions, and execute those decisions through robot motion. The ROS 2 ecosystem provides the necessary tools and frameworks to implement these complex systems in a modular, distributed, and maintainable way.

---
sidebar_label: From AI to Motion
sidebar_position: 3
pagination_next: null
pagination_prev: modules/ros2-nervous-system/chapter-2-core-concepts
keywords: [ROS 2, rclpy, AI agents, controllers, actuators, URDF, humanoid, robot motion, Python]
description: Connecting Python AI agents to robot controllers using rclpy, understanding controllers and actuators, and learning URDF basics for humanoid structure.
---

## Learning Objectives Review

- ✅ Connect Python AI agents via rclpy
- ✅ Understand controllers and actuators overview
- ✅ Learn URDF basics for humanoid structure