import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['home', 'text-selection-popup'],
      link: { type: 'doc', id: 'home' },
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/module-1-intro',
        'module-1/module-1-chapter-1',
        'module-1/module-1-chapter-2',
        'module-1/module-1-chapter-3',
        'module-1/module-1-chapter-4',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/module-2-intro',
        'module-2/module-2-chapter-1',
        'module-2/module-2-chapter-2',
        'module-2/module-2-chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/module-3-intro',
        'module-3/module-3-chapter-1',
        'module-3/module-3-chapter-2',
        'module-3/module-3-chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/module-4-intro',
        'module-4/module-4-chapter-1',
        'module-4/module-4-chapter-2',
        'module-4/module-4-chapter-3',
      ],
    },
  ],
};

export default sidebars;
