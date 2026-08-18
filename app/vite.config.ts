import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
  plugins: [svelte()],
  server: {
    proxy: {
      '/docs': {
        target: 'http://localhost:4321',
        changeOrigin: true,
      },
      '/_astro': {
        target: 'http://localhost:4321',
        changeOrigin: true,
      },
    },
  },
});
