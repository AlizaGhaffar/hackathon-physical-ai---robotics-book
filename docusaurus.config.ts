import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Chapter 1: ROS 2 - The Robotic Nervous System',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-site.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config (not used for Vercel)
  organizationName: 'AlizaGhaffar',
  projectName: 'hackathon-physical-ai---robotics-book',

  onBrokenLinks: 'throw',

  // Markdown configuration
  markdown: {
    format: 'mdx',
    mermaid: false,
    preprocessor: undefined,
    parseFrontMatter: undefined,
    mdx1Compat: undefined,
    remarkRehypeOptions: undefined,
    anchors: undefined,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'], // Only English for now (Urdu support requires i18n folder structure)
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // Serve docs at site root
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: undefined, // Disable edit links for now
        },
        blog: false, // Disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/social-card.jpg',
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {
          href: 'https://github.com/AlizaGhaffar/hackathon-physical-ai---robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Chapter 1: ROS 2 Fundamentals',
              to: '/chapter-1/intro',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/AlizaGhaffar/hackathon-physical-ai---robotics-book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
