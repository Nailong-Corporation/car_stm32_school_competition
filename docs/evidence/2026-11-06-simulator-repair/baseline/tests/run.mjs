import { createServer } from 'vite';

// Use the project's TypeScript transform; keep the cache inside this staged copy.
const server = await createServer({
  server: { middlewareMode: true },
  logLevel: 'error',
  cacheDir: '.vite-test-cache',
  optimizeDeps: { noDiscovery: true, include: [] },
  resolve: { preserveSymlinks: true },
});
try {
  await server.ssrLoadModule('/tests/startup.test.ts');
  await server.ssrLoadModule('/tests/navigation.test.ts');
} finally {
  await server.close();
}
