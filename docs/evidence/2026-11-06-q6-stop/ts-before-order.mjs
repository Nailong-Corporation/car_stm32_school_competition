import { createServer } from '/Users/zhendong/Documents/stm32car_simulator/node_modules/vite/dist/node/index.js';
import { readFileSync } from 'node:fs';
const root = '/Users/zhendong/Documents/stm32car_simulator';
const before = readFileSync(new URL('./line-follow-ts-before-if-order.ts', import.meta.url), 'utf8');
const server = await createServer({
  root,
  cacheDir: new URL('./ts-before-cache', import.meta.url).pathname,
  server: { middlewareMode: true },
  optimizeDeps: { noDiscovery: true, include: [] },
  logLevel: 'error',
  plugins: [{
    name: 'reorder-recovery-before-timeout',
    enforce: 'pre',
    transform(code, id) { if (id.endsWith('/src/sim/line_follow.ts')) return before; },
  }],
});
try {
  await server.ssrLoadModule('/tests/q4_q6.test.ts');
} finally {
  await server.close();
}
