# Data Model: Homepage Theme & Section Redesign

## Theme Configuration

**Entity**: Theme Configuration
- **Description**: The color scheme and styling parameters that define the visual appearance of the site
- **Fields**:
  - primaryColor: string (hex code for navy blue theme)
  - secondaryColor: string (complementary color)
  - darkModePrimary: string (primary color for dark mode)
  - darkModeSecondary: string (secondary color for dark mode)
  - textOnPrimary: string (text color on primary backgrounds)
  - textOnSecondary: string (text color on secondary backgrounds)
  - contrastRatio: number (WCAG compliance ratio)

## Homepage Content Sections

**Entity**: Homepage Content Sections
- **Description**: Structured content blocks that present information about the book to visitors
- **Fields**:
  - sectionId: string (unique identifier for the section)
  - title: string (section heading)
  - content: string (main content text)
  - order: number (display order)
  - styling: object (CSS classes and custom styles)

### Section 1: What This Book Is About
- **sectionId**: "about-book"
- **title**: "📘 What This Book Is About"
- **content**: "Physical AI & Humanoid Robotics focuses on Embodied Intelligence—AI systems tightly coupled with sensors, actuators, and the physical environment. You will learn how to: Design robotic nervous systems using ROS 2, Create digital twins using physics-accurate simulators, Train perception and navigation models, Connect Large Language Models to robot actions, Build autonomous humanoid agents. This book bridges the gap between: 🧠 AI reasoning, 🦾 Robotic motion, 🌍 Real-world physics."
- **order**: 1
- **styling**: { containerClass: "container", sectionClass: "padding-vert--lg" }

### Section 2: Course Structure
- **sectionId**: "course-structure"
- **title**: "🧩 Course Structure"
- **content**: "Module 1: The Robotic Nervous System (ROS 2) - ROS 2 nodes, topics, and services; Python agents with rclpy; URDF modeling for humanoid bodies. Module 2: The Digital Twin (Gazebo & Unity) - Physics, gravity, and collisions; Sensor simulation (LiDAR, cameras, IMUs); Human-robot interaction environments. Module 3: The AI-Robot Brain (NVIDIA Isaac) - Photorealistic simulation; Synthetic data generation; Visual SLAM and navigation. Module 4: Vision-Language-Action (VLA) - Voice-to-Action pipelines; Natural language task planning; Autonomous humanoid agents."
- **order**: 2
- **styling**: { containerClass: "container", sectionClass: "padding-vert--lg" }