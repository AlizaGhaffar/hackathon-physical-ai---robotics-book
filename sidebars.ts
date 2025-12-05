import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Chapter 1: ROS 2 Fundamentals',
      collapsed: true,
      items: [
        'chapter-1/intro',
        'chapter-1/what-is-ros2',
        'chapter-1/why-ros2',
        {
          type: 'category',
          label: 'Core Concepts',
          items: [
            'chapter-1/core-concepts/nodes',
            'chapter-1/core-concepts/topics',
            'chapter-1/core-concepts/services',
          ],
        },
        'chapter-1/installation',
        'chapter-1/urdf-basics',
        {
          type: 'category',
          label: 'Code Examples',
          items: [
            'chapter-1/examples/example-01-hello-node',
            'chapter-1/examples/example-02-publisher',
            'chapter-1/examples/example-03-subscriber',
            'chapter-1/examples/example-04-service-server',
            'chapter-1/examples/example-05-service-client',
          ],
        },
        {
          type: 'category',
          label: 'Hands-On Exercise',
          items: [
            'chapter-1/exercises/exercise-01',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: Gazebo Simulation',
      collapsed: false,
      items: [
        'chapter-2/intro',
        'chapter-2/what-is-gazebo',
        {
          type: 'category',
          label: 'URDF & SDF',
          items: [
            'chapter-2/urdf-sdf/urdf-basics',
            'chapter-2/urdf-sdf/sdf-basics',
            'chapter-2/urdf-sdf/urdf-vs-sdf',
          ],
        },
        {
          type: 'category',
          label: 'Physics Simulation',
          items: [
            'chapter-2/physics/physics-engines',
            'chapter-2/physics/gravity-friction',
            'chapter-2/physics/collisions',
          ],
        },
        {
          type: 'category',
          label: 'Sensor Simulation',
          items: [
            'chapter-2/sensors/cameras',
            'chapter-2/sensors/lidar',
            'chapter-2/sensors/imu',
            'chapter-2/sensors/sensor-plugins',
          ],
        },
        {
          type: 'category',
          label: 'Code Examples',
          items: [
            'chapter-2/examples/simple-robot',
            'chapter-2/examples/robot-with-sensors',
            'chapter-2/examples/physics-demo',
          ],
        },
        {
          type: 'category',
          label: 'Hands-On Exercise',
          items: [
            'chapter-2/exercises/build-your-robot',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Vision-Language-Action Models',
      collapsed: false,
      items: [
        'chapter-3/intro',
        'chapter-3/what-is-vla',
        {
          type: 'category',
          label: 'Voice AI',
          items: [
            'chapter-3/voice-ai/whisper-integration',
          ],
        },
        {
          type: 'category',
          label: 'LLM Planning',
          items: [
            'chapter-3/llm-planning/gpt4-integration',
          ],
        },
        {
          type: 'category',
          label: 'Multi-modal Integration',
          items: [
            'chapter-3/multimodal/gpt4-vision',
          ],
        },
        {
          type: 'category',
          label: 'Capstone Project',
          items: [
            'chapter-3/capstone/overview',
          ],
        },
        {
          type: 'category',
          label: 'Examples',
          items: [
            'chapter-3/examples/voice-robot-demo',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
