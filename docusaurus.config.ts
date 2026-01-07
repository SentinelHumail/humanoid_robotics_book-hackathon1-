import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the gap between the digital brain and the physical body',
  favicon: 'img/favicon.ico',

  url: 'https://SentinelHumail.github.io',
  baseUrl: '/humanoid_robotics_book-hackathon1-/',

  organizationName: 'your-org',
  projectName: 'hackathon1-book',

  onBrokenLinks: 'warn',

  customFields: {
    CHATBOT_API_URL: process.env.DOCUSAURUS_PUBLIC_API_URL || 'https://humanoid-robotics-book-hackathon1.onrender.com',
  },

  headTags: [
    {
      tagName: 'meta',
      attributes: {
        'http-equiv': 'Content-Security-Policy',
        content: "default-src 'self'; connect-src 'self' https://humanoid-robotics-book-hackathon1.onrender.com; script-src 'self' 'unsafe-inline' 'unsafe-eval'; style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; font-src https://fonts.gstatic.com; img-src 'self' data:;"
      }
    }
  ],


  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/your-org/hackathon1-book/tree/main/',
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: '🤖 Physical AI Book',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'modulesSidebar',
          position: 'left',
          label: 'Course',
        },
        {
          type: 'dropdown',
          label: 'Learn',
          position: 'left',
          items: [
            {
              label: 'Modules',
              to: '/docs/modules/intro',
            },
            {
              label: 'Hardware',
              to: '/docs/hardware/overview',
            },
            {
              label: 'Deployment',
              to: '/docs/deployment/overview',
            },
            {
              label: 'Tutorials',
              to: '/docs/tutorials/ros2-basics',
            },
          ],
        },
        {
          href: 'https://github.com/your-org/hackathon1-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
