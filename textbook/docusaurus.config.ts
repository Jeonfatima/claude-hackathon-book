import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// Docusaurus config (Node.js runtime)
const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Complete Textbook',
  favicon: 'img/favicon.ico',

  // future: {
  //   v4: true,
  // },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Plugins
  plugins: [
    // Custom plugin to inject configuration into the client
    async function injectConfigPlugin() {
      return {
        name: 'inject-config',
        injectHtmlTags() {
          const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
          const apiKey = process.env.REACT_APP_API_KEY || '';

          return {
            preBodyTags: [
              `<script>
                window.__APP_CONFIG__ = {
                  REACT_APP_BACKEND_URL: ${JSON.stringify(backendUrl)},
                  REACT_APP_API_KEY: ${JSON.stringify(apiKey)}
                };
              </script>`,
            ],
          };
        },
      };
    },
  ],

  // Presets
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/your-org/physical-ai-textbook',
          routeBasePath: '/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  // Theme configuration
  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/your-org/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {label: 'Introduction', to: '/'},
          ],
        },
        {
          title: 'Modules',
          items: [
            {label: 'Module 1: ROS 2', to: 'module-1/module-1-intro'},
            {label: 'Module 2: Digital Twin', to: 'module-2/module-2-intro'},
            {label: 'Module 3: AI-Robot Brain', to: 'module-3/module-3-intro'},
            {label: 'Module 4: Vision-Language-Action', to: 'module-4/module-4-intro'},
          ],
        },
        {
          title: 'More',
          items: [
            {label: 'GitHub', href: 'https://github.com/your-org/physical-ai-textbook'},
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Fatima Salman roll# 464666.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
