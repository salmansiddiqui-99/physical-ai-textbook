/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    'quick-reference',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1-ros2/ch1-ros2-basics',
        'module1-ros2/ch2-ros2-humanoids',
        'module1-ros2/ch3-urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Simulation)',
      items: [
        'module2-simulation/ch1-gazebo-essentials',
        'module2-simulation/ch2-sensor-simulation',
        'module2-simulation/ch3-unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3-isaac/ch1-isaac-sim',
        'module3-isaac/ch2-isaac-ros-perception',
        'module3-isaac/ch3-navigation-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4-vla/ch1-voice-to-action',
        'module4-vla/ch2-cognitive-planning',
        'module4-vla/ch3-capstone-project',
      ],
    },
  ],
};

module.exports = sidebars;
