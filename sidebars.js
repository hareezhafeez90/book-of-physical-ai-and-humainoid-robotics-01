// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/urdf-structure',
        'module-1-ros2/rclpy-bridge',
        'module-1-ros2/practical-examples',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Physical AI and Humanoid Control',
      link: {
        type: 'doc',
        id: 'module-2-humanoid-control/index',
      },
      items: [
        'module-2-humanoid-control/physical-ai-foundations',
        'module-2-humanoid-control/kinematics',
        'module-2-humanoid-control/dynamics-control',
        'module-2-humanoid-control/balance-stability',
        'module-2-humanoid-control/motor-control',
        'module-2-humanoid-control/practical-examples',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Perception and Sensing Systems',
      link: {
        type: 'doc',
        id: 'module-3-perception-sensing/index',
      },
      items: [
        'module-3-perception-sensing/sensory-modalities',
        'module-3-perception-sensing/computer-vision',
        'module-3-perception-sensing/tactile-sensing',
        'module-3-perception-sensing/inertial-sensing',
        'module-3-perception-sensing/sensor-fusion',
        'module-3-perception-sensing/environmental-perception',
        'module-3-perception-sensing/practical-examples',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Locomotion and Mobility',
      link: {
        type: 'doc',
        id: 'module-4-locomotion-mobility/index',
      },
      items: [
        'module-4-locomotion-mobility/biomechanics',
        'module-4-locomotion-mobility/dynamic-walking',
        'module-4-locomotion-mobility/gait-generation',
        'module-4-locomotion-mobility/terrain-adaptation',
        'module-4-locomotion-mobility/complex-terrain',
        'module-4-locomotion-mobility/energy-efficiency',
      ],
    },
    'setup-guide',
    'appendix-resources',
  ],
};

module.exports = sidebars;