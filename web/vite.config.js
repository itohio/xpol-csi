import { defineConfig } from 'vite';
import preact from '@preact/preset-vite';
import { compression } from 'vite-plugin-compression2';

// Output goes to dist/, flat (no assets/ dir) with short hashed names so SPIFFS
// object-name limits are comfortable. Every text asset is gzipped and the
// original is dropped; the firmware serves the .gz with Content-Encoding: gzip.
export default defineConfig({
  base: './',
  plugins: [
    preact(),
    compression({
      algorithm: 'gzip',
      deleteOriginalAssets: true,
      threshold: 0,
      include: /\.(js|css|html|svg|json|ico)$/,
    }),
  ],
  build: {
    target: 'es2020',
    assetsDir: '',
    cssMinify: true,
    rollupOptions: {
      output: {
        entryFileNames: '[name]-[hash].js',
        chunkFileNames: '[name]-[hash].js',
        assetFileNames: '[name]-[hash][extname]',
      },
    },
  },
});
