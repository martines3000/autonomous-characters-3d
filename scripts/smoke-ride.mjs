#!/usr/bin/env node
// Check moving GPU camera subjects in the real headed browser, without HUD-coordinate assumptions.
import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
import { createRequire } from 'node:module';
import os from 'node:os';
import path from 'node:path';

const require = createRequire(new URL('../web/package.json', import.meta.url));
const { chromium } = require('playwright');
const url = new URL(process.argv[2] || 'http://127.0.0.1:8080');
const output = path.resolve(process.argv[3] || path.join(os.tmpdir(), 'flock-ride-review'));
await mkdir(output, { recursive: true });
const args = process.env.AC3D_BROWSER_ARGS ? JSON.parse(process.env.AC3D_BROWSER_ARGS)
  : process.platform === 'linux'
    ? ['--enable-unsafe-webgpu', '--enable-features=Vulkan', '--use-angle=vulkan', '--ozone-platform=wayland'] : [];
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_EXECUTABLE || undefined, headless: false, args });
const report = { browser: browser.version(), args, cases: [], errors: [] };
try {
  const context = await browser.newContext({ viewport: { width: 1280, height: 720 }, reducedMotion: 'reduce' });
  await context.addInitScript(() => {
    window.rideSamples = [];
    const mappedRange = GPUBuffer.prototype.getMappedRange;
    GPUBuffer.prototype.getMappedRange = function (...args) {
      const range = mappedRange.apply(this, args);
      if (this.size === 80 && window.flockReport?.scene?.camera === 'ride') {
        // Observe the existing readback once; mirror Rust's rejection of stale selection replies.
        const bytes = range.slice(0);
        const words = new Uint32Array(bytes);
        if (words[18] === 1 && words[16] === window.flockReport.scene.cameraAnimalId) {
          const values = new Float32Array(bytes);
          window.rideSamples.push({ id: words[16], stamp: values[17], position: [...values.slice(0, 3)], velocity: [...values.slice(4, 7)] });
        }
      }
      return range;
    };
  });
  const page = await context.newPage();
  page.on('pageerror', error => report.errors.push(String(error)));
  for (const scene of ['murmuration', 'reef', 'shark']) {
    const target = new URL(url);
    target.searchParams.set('scene', scene);
    target.searchParams.set('population', 'medium');
    await page.goto(target.href, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => window.flockReport?.state === 'ready' && window.flockReport.scene?.animated, undefined, { timeout: 120_000 });
    await page.keyboard.press('b');
    await page.waitForFunction(() => window.flockReport.scene.camera === 'ride' && window.flockReport.scene.cameraAnimalId !== null);
    await page.waitForTimeout(750);
    await page.evaluate(() => { window.rideSamples = []; });
    await page.waitForTimeout(3000);
    const sample = await page.evaluate(() => ({ poses: window.rideSamples, scene: window.flockReport.scene, adapter: window.flockReport.adapter }));
    report.cases.push(sample);
    assert(sample.poses.length > 30, 'Moving Ride did not return enough animal poses');
    assert.equal(new Set(sample.poses.map(pose => pose.id)).size, 1, 'Moving Ride changed animal without input');
    assert.equal(sample.scene.cameraAnimalId, sample.poses[0].id);
    for (let index = 1; index < sample.poses.length; index++) {
      const previous = sample.poses[index - 1];
      const current = sample.poses[index];
      const dt = current.stamp - previous.stamp;
      if (dt <= 0) continue; // Async completions may be out of order; Rust rejects older poses.
      const distance = Math.hypot(...current.position.map((value, axis) => value - previous.position[axis]));
      const speed = Math.max(Math.hypot(...previous.velocity), Math.hypot(...current.velocity));
      assert(distance <= speed * dt * 3 + .1, `Ride animal ${current.id} jumped ${distance} units in ${dt} seconds`);
    }
    await page.screenshot({ path: path.join(output, `${scene}-ride.png`) });
    console.log(`PASS: ${scene} Ride retained animal ${sample.scene.cameraAnimalId} across ${sample.poses.length} continuous poses`);
  }
  assert.deepEqual(report.errors, []);
} finally {
  await writeFile(path.join(output, 'ride-report.json'), JSON.stringify(report, null, 2));
  await browser.close();
}
