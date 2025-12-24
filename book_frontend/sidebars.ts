import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'modules/ros2-nervous-system/chapter-1-what-is-ros2',
        'modules/ros2-nervous-system/chapter-2-core-concepts',
        'modules/ros2-nervous-system/chapter-3-ai-to-motion',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'modules/digital-twin/chapter-1-physics-simulation',
        'modules/digital-twin/chapter-2-sensor-simulation',
        'modules/digital-twin/chapter-3-unity-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/ai-robot-brain/chapter-1-isaac-sim',
        'modules/ai-robot-brain/chapter-2-isaac-ros',
        'modules/ai-robot-brain/chapter-3-nav2-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/module-4-vla/chapter-1-vision-language-action',
        'modules/module-4-vla/chapter-2-cognitive-planning',
        'modules/module-4-vla/chapter-3-capstone-project',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
