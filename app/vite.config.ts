import { defineConfig, type Plugin } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import fs from 'node:fs';
import path from 'node:path';

function serveDocsPlugin(): Plugin {
  return {
    name: 'serve-docs',
    configureServer(server) {
      const docsDistDir = path.resolve(__dirname, 'dist/docs');
      server.middlewares.use('/docs', (req, res, next) => {
        if (!req.url) return next();
        let reqPath = req.url.split('?')[0];
        if (reqPath === '' || reqPath === '/') {
          reqPath = '/index.html';
        }

        let targetFile = path.join(docsDistDir, reqPath);
        // If it's a directory, check for index.html
        if (fs.existsSync(targetFile) && fs.statSync(targetFile).isDirectory()) {
          targetFile = path.join(targetFile, 'index.html');
        } else if (!fs.existsSync(targetFile) && fs.existsSync(targetFile + '.html')) {
          targetFile = targetFile + '.html';
        } else if (!fs.existsSync(targetFile) && fs.existsSync(path.join(targetFile, 'index.html'))) {
          targetFile = path.join(targetFile, 'index.html');
        }

        if (fs.existsSync(targetFile) && fs.statSync(targetFile).isFile()) {
          const ext = path.extname(targetFile).toLowerCase();
          const mimeTypes: Record<string, string> = {
            '.html': 'text/html; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
            '.mjs': 'application/javascript; charset=utf-8',
            '.json': 'application/json',
            '.png': 'image/png',
            '.jpg': 'image/jpeg',
            '.jpeg': 'image/jpeg',
            '.svg': 'image/svg+xml',
            '.webp': 'image/webp',
            '.stl': 'model/stl',
            '.woff': 'font/woff',
            '.woff2': 'font/woff2',
            '.ttf': 'font/ttf',
          };
          res.setHeader('Content-Type', mimeTypes[ext] || 'application/octet-stream');
          return fs.createReadStream(targetFile).pipe(res);
        }
        next();
      });
    },
  };
}

export default defineConfig({
  plugins: [svelte(), serveDocsPlugin()],
});
