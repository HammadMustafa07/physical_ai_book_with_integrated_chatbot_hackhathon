---
title: Chapter 3 - Navigation & Path Planning with Nav2
sidebar_position: 3
description: Navigation and path planning for bipedal humanoid robots using Nav2
---

# Chapter 3: Navigation & Path Planning with Nav2

## Nav2 Architecture Overview

Navigation2 (Nav2) is the next-generation navigation framework for ROS 2, designed to provide flexible, robust, and efficient navigation capabilities for mobile robots. For humanoid robots, Nav2 provides the essential "brain" functionality that enables autonomous movement through complex environments.

### Core Architecture Components

Nav2 follows a behavior tree-based architecture that allows for flexible and configurable navigation behaviors:

- **Navigation Server**: Central coordinator that manages the navigation system
- **Behavior Trees**: Define navigation behaviors and decision-making logic
- **Plugins**: Modular components for specific navigation tasks
- **Action Interface**: Standardized interface for navigation commands

### Key Navigation Capabilities

Nav2 provides several core capabilities essential for humanoid robot navigation:

- **Map Management**: Loading, updating, and managing environmental maps
- **Localization**: Determining the robot's position within the map
- **Path Planning**: Computing optimal paths from start to goal locations
- **Path Following**: Executing planned paths while avoiding obstacles
- **Recovery Behaviors**: Handling navigation failures and getting unstuck

### Humanoid-Specific Considerations

When configuring Nav2 for humanoid robots, several factors must be considered:
- **Bipedal Kinematics**: Different motion constraints compared to wheeled robots
- **Stability Requirements**: Path planning must consider balance and stability
- **Height Perspective**: Sensor placement affects mapping and obstacle detection
- **Dynamic Motion**: Bipedal gait creates unique navigation challenges

## Mapping, Localization, and Planning Pipelines

Nav2 implements three interconnected pipelines that work together to enable autonomous navigation:

### Mapping Pipeline

The mapping pipeline creates and maintains representations of the environment:

**SLAM (Simultaneous Localization and Mapping)**:
- Creates maps while simultaneously localizing the robot
- Uses sensor data from cameras, LiDAR, and IMU
- Handles loop closure and map optimization

**Map Updates**:
- Dynamically updates maps as the environment changes
- Incorporates new sensor data to refine existing maps
- Maintains multiple map layers for different purposes

### Localization Pipeline

The localization pipeline determines the robot's position within the map:

**AMCL (Adaptive Monte Carlo Localization)**:
- Uses particle filters to estimate robot pose
- Incorporates sensor data and motion models
- Handles localization uncertainty and recovery

**Sensor Fusion**:
- Combines data from multiple sensors for robust localization
- Integrates IMU data for motion prediction
- Uses visual features for improved accuracy

### Planning Pipeline

The planning pipeline computes optimal paths for navigation:

**Global Planner**:
- Computes long-term paths across the full map
- Considers overall route efficiency and safety
- Updates plans as the environment changes

**Local Planner**:
- Executes short-term navigation while avoiding obstacles
- Handles dynamic obstacle avoidance
- Maintains real-time path following

## Path Planning Constraints for Bipedal Humanoids

Bipedal humanoid robots have unique path planning requirements that differ significantly from wheeled or tracked robots:

### Kinematic Constraints

Humanoid robots must follow paths that respect their bipedal nature:

**Step Constraints**:
- Minimum and maximum step length limitations
- Step height restrictions for safe locomotion
- Turning radius based on foot placement patterns
- Balance constraints during walking transitions

**Stability Requirements**:
- Paths must maintain center of mass within support polygon
- Consideration of zero moment point (ZMP) for stable walking
- Dynamic balance during path execution
- Safe stopping distances for emergency situations

### Environmental Constraints

Humanoid-specific environmental considerations:

**Vertical Navigation**:
- Stair climbing capabilities and limitations
- Ramp navigation with appropriate angles
- Threshold and curb navigation
- Step height and depth constraints

**Space Requirements**:
- Body width and depth for passage clearance
- Arm swing space during walking
- Doorway and corridor navigation
- Turning space requirements

### Gait Pattern Considerations

Different walking patterns affect path planning:

**Walking Gaits**:
- Normal walking speed and stride patterns
- Slow, careful navigation modes
- Fast walking for efficiency
- Specialized gaits for different terrains

**Balance Transitions**:
- Standing to walking transitions
- Walking to standing transitions
- Direction changes while maintaining balance
- Recovery from minor disturbances

## Obstacle Avoidance and Dynamic Environments

Humanoid robots must navigate in environments with both static and dynamic obstacles:

### Static Obstacle Handling

**Map-Based Avoidance**:
- Pre-mapped obstacles in global planning
- Real-time detection of new obstacles
- Safe distance maintenance from known hazards
- Alternative path computation around obstacles

**Humanoid-Specific Obstacles**:
- Tables and furniture at human height
- Low obstacles that may not affect wheeled robots
- Overhead obstacles that affect tall robots
- Narrow passages requiring specific navigation

### Dynamic Obstacle Avoidance

**Moving Obstacle Detection**:
- Human and robot detection in environment
- Prediction of movement trajectories
- Safe distance maintenance from moving objects
- Right-of-way decision making

**Reactive Navigation**:
- Real-time path replanning for dynamic obstacles
- Velocity adjustment based on obstacle proximity
- Emergency stopping capabilities
- Social navigation behaviors

### Human-Robot Interaction

**Social Navigation**:
- Respect for personal space of humans
- Appropriate passing behaviors
- Eye contact and communication during navigation
- Yielding to humans in shared spaces

## Interaction between Perception (Isaac ROS) and Navigation (Nav2)

The integration between perception systems (Isaac ROS) and navigation systems (Nav2) forms the complete AI-robot brain:

### Sensor Data Flow

**Perception to Navigation**:
- Camera data for visual SLAM and localization
- LiDAR data for obstacle detection and mapping
- IMU data for motion prediction and stability
- Sensor fusion for robust environmental understanding

**Data Processing Pipeline**:
- Raw sensor data → Isaac ROS processing → ROS 2 messages → Nav2 consumption
- Real-time processing with low latency requirements
- Data validation and quality checks
- Fallback mechanisms for sensor failures

### Coordination Mechanisms

**Shared Maps**:
- Isaac ROS creates detailed perception maps
- Nav2 uses these maps for navigation planning
- Continuous map updates during operation
- Multi-sensor map fusion

**Localization Integration**:
- Isaac ROS provides precise localization
- Nav2 uses this for accurate navigation
- Sensor fusion for robust positioning
- Recovery from localization failures

### Performance Optimization

**Computational Load Balancing**:
- Distribute processing across available hardware
- Prioritize critical navigation tasks
- Optimize for real-time performance
- Handle computational resource constraints

## Preparing the Robot Brain for Voice-Driven and LLM-Driven Commands

The AI-robot brain implemented in this module serves as the foundation for higher-level cognitive capabilities:

### Integration Architecture

**Command Processing Pipeline**:
- Voice/LLM commands → Semantic understanding → Navigation goals → Path execution
- Natural language to navigation goal translation
- Context-aware command interpretation
- Multi-modal command processing

**Goal Specification**:
- High-level location descriptions (e.g., "go to the kitchen")
- Semantic navigation targets (e.g., "find the red chair")
- Context-dependent navigation (e.g., "go somewhere quiet")
- Multi-step navigation tasks

### Cognitive Integration

**Memory and Learning**:
- Remembering frequently visited locations
- Learning preferred navigation routes
- Adapting to user preferences
- Building semantic maps of the environment

**Safety and Ethics**:
- Obeying navigation constraints and rules
- Respecting privacy and safety boundaries
- Handling ambiguous or unsafe commands
- Maintaining human oversight capabilities

### Future Module Connection

This module establishes the foundation for Module 4 (Vision-Language-Action) by:
- Providing the navigation capabilities needed for physical tasks
- Creating the perception system for object identification
- Establishing the integration between high-level commands and physical actions
- Preparing the robot for complex multi-modal interactions

## Hands-On Exercise: Nav2 Configuration

### Objective
Configure Nav2 for bipedal humanoid navigation with perception integration.

### Prerequisites
- Understanding of Isaac ROS from previous chapter
- Knowledge of ROS 2 navigation concepts
- Basic understanding of humanoid robot kinematics

### Steps
1. **Nav2 Installation**: Set up Nav2 with humanoid-specific configurations
2. **Parameter Tuning**: Configure parameters for bipedal locomotion constraints
3. **Perception Integration**: Connect Isaac ROS perception to Nav2 navigation
4. **Path Planning**: Configure planners for humanoid-specific constraints
5. **Testing**: Validate navigation performance in simulation and real-world scenarios

### Expected Outcomes
By completing this exercise, you should understand how to:
- Configure Nav2 for humanoid-specific navigation requirements
- Integrate perception and navigation systems
- Optimize navigation parameters for bipedal robots
- Validate the complete AI-robot brain functionality