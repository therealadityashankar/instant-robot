// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import svelte from '@astrojs/svelte';

// https://astro.build/config
export default defineConfig({
  site: 'https://therealadityashankar.github.io',
  base: '/instant-robot',
  integrations: [
    starlight({
      title: 'Instant Robot Docs',
      description: 'Zero-install robotics, simulation, and visual navigation documentation.',
      logo: {
        dark: './src/assets/logo.svg',
        light: './src/assets/logo.svg',
        replacesTitle: false,
      },
      social: {
        github: 'https://github.com/therealadityashankar/instant-robot',
      },
      sidebar: [
        {
          label: 'Getting Started',
          autogenerate: { directory: 'getting-started' },
        },
        {
          label: 'Interactive Tag Tools',
          autogenerate: { directory: 'tags-and-navigation' },
        },
        {
          label: 'Hardware & Assembly',
          autogenerate: { directory: 'hardware' },
        },
        {
          label: 'User & Developer Guides',
          autogenerate: { directory: 'guides' },
        },
      ],
      customCss: ['./src/styles/custom.css'],
    }),
    svelte(),
  ],
});
