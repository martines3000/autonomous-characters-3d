#!/usr/bin/env node
// Repeat every public scene/tier with the real GPU renderer. Keep other work idle.
import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
import { createRequire } from 'node:module';
import os from 'node:os';
import path from 'node:path';

const require = createRequire(new URL('../web/package.json', import.meta.url));
const { chromium } = require('playwright');
const url = process.argv[2] || 'http://127.0.0.1:8080';
const output = path.resolve(process.argv[3] || path.join(os.tmpdir(), 'flock-web-measurements'));
await mkdir(output, { recursive: true });
const args = process.env.AC3D_BROWSER_ARGS ? JSON.parse(process.env.AC3D_BROWSER_ARGS)
  : process.platform === 'linux'
    ? ['--enable-unsafe-webgpu', '--enable-features=Vulkan', '--use-angle=vulkan', '--ozone-platform=wayland'] : [];
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_EXECUTABLE || undefined, headless: false, args });
const report = { browser: browser.version(), executable: process.env.CHROMIUM_EXECUTABLE || chromium.executablePath(), args, os: `${os.type()} ${os.release()} ${os.arch()}`, loadAverage: os.loadavg(), warmupSeconds: 3, sampleSeconds: 6, seed: 'Interactive GPU scatter; no seeded replay is claimed', effects: 'Public scene defaults; reduced camera motion; 4x MSAA; volumetrics and motion blur off', results: [], errors: [] };
try {
  const context = await browser.newContext({ viewport: { width: 1920, height: 1080 }, deviceScaleFactor: 1, reducedMotion: 'reduce' });
  const page = await context.newPage();
  page.on('pageerror', error => report.errors.push(String(error)));
  page.on('console', message => { if (message.type() === 'error') report.errors.push(message.text()); });
  page.on('response', response => { if (response.status() >= 400) report.errors.push(`${response.status()} ${response.url()}`); });
  await page.goto(url, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => window.flockReport?.state === 'ready', undefined, { timeout: 120_000 });
  report.artifact = await (await context.request.get(new URL('release.json', url).href)).json();
  const control = await browser.newBrowserCDPSession();
  report.gpu = await control.send('SystemInfo.getInfo');
  for (const [scene, name] of [['murmuration', 'Sunset murmuration'], ['reef', 'Reef drift'], ['shark', 'Shark hunt']]) {
    for (const population of ['low', 'medium', 'high']) {
      for (let run = 1; run <= 2; run++) {
        await page.evaluate(([scene, population]) => window.flock.flock_scene(scene, population), [scene, population]);
        await page.waitForFunction(name => window.flockReport?.state === 'ready' && window.flockReport?.scene?.name === name, name, { timeout: 120_000 });
        await page.waitForTimeout(3000);
        await page.evaluate(() => { window.flockReport.frames = []; });
        await page.waitForTimeout(6000);
        const sample = await page.evaluate(() => ({ frames: window.flockReport.frames, state: window.flockReport.scene, resolution: [document.querySelector('canvas').width, document.querySelector('canvas').height] }));
        assert(sample.frames.length >= 100, `${name}/${population} produced too few rendered intervals`);
        assert.equal(sample.state.name, name);
        assert(sample.state.animated && !sample.state.paused);
        assert.deepEqual(sample.resolution, [1920, 1080]);
        const sorted = [...sample.frames].sort((a, b) => a - b);
        const meanMs = sample.frames.reduce((sum, value) => sum + value, 0) / sample.frames.length;
        const result = { scene, population, run, count: sample.state.count, hunters: sample.state.hunters, resolution: sample.resolution, frames: sample.frames.length, meanMs, p50Ms: sorted[Math.floor(sorted.length * .5)], p95Ms: sorted[Math.floor(sorted.length * .95)], maxMs: sorted.at(-1), fps: 1000 / meanMs, loadAverage: os.loadavg() };
        report.results.push(result);
        console.log(JSON.stringify(result));
      }
    }
  }
  assert.deepEqual(report.errors, []);
  report.result = 'PASS';
} catch (error) {
  report.result = 'FAIL';
  report.failure = error.stack || String(error);
  process.exitCode = 1;
} finally {
  await writeFile(path.join(output, 'measurements.json'), JSON.stringify(report, null, 2));
  await browser.close();
}
console.log(`${report.result}: measurements in ${output}`);
if (report.failure) console.error(report.failure);
