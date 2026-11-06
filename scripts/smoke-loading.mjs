#!/usr/bin/env node
// Headed proof of real streamed progress, reveal, reduced motion and late-error recovery.
import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
import { createRequire } from 'node:module';
import os from 'node:os';
import path from 'node:path';

const require = createRequire(new URL('../web/package.json', import.meta.url));
const { chromium } = require('playwright');
const url = process.argv[2] || 'http://127.0.0.1:8080';
const output = path.resolve(process.argv[3] || path.join(os.tmpdir(), 'flock-loading-review'));
await mkdir(output, { recursive: true });
const args = process.env.AC3D_BROWSER_ARGS ? JSON.parse(process.env.AC3D_BROWSER_ARGS)
  : ['--enable-unsafe-webgpu', '--enable-features=Vulkan', '--use-angle=vulkan', '--ozone-platform=wayland'];
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_EXECUTABLE || undefined, headless: false, args });
const report = { browser: browser.version(), args, cases: [] };
try {
  for (const mode of ['normal', 'reduced', 'late-error']) {
    const context = await browser.newContext({ viewport: { width: 1920, height: 1080 }, reducedMotion: mode === 'reduced' ? 'reduce' : 'no-preference' });
    await context.addInitScript(() => {
      window.reviewAnimations = [];
      const animate = Element.prototype.animate;
      Element.prototype.animate = function (keyframes, options) {
        if (this.id === 'flock') window.reviewAnimations.push({ keyframes, options });
        return animate.call(this, keyframes, options);
      };
    });
    const page = await context.newPage();
    const errors = [];
    page.on('pageerror', error => errors.push(String(error)));
    if (mode === 'normal') {
      const cdp = await context.newCDPSession(page);
      await cdp.send('Network.enable');
      await cdp.send('Network.emulateNetworkConditions', { offline: false, latency: 30, downloadThroughput: 2 * 1024 * 1024, uploadThroughput: 1024 * 1024 });
    }
    await page.goto(url, { waitUntil: 'domcontentloaded' });
    if (mode === 'normal') {
      await page.waitForFunction(() => window.flockReport?.progress?.stage === 'download' && window.flockReport.progress.received > 0 && window.flockReport.progress.total > window.flockReport.progress.received, undefined, { timeout: 120_000 });
      const progress = await page.evaluate(() => ({ ...window.flockReport.progress, displayedValue: document.querySelector('progress').value, displayedMax: document.querySelector('progress').max, inert: document.querySelector('main').inert }));
      assert(progress.inert && progress.displayedValue > 0 && progress.displayedValue < progress.displayedMax);
      report.cases.push({ mode: 'streamed-progress', network: '2 MiB/s, 30 ms CDP network throttle', progress });
      await page.screenshot({ path: path.join(output, 'loading-progress.png') });
    }
    if (mode !== 'reduced') {
      await page.waitForFunction(() => document.querySelector('canvas').getAnimations().length > 0, undefined, { timeout: 120_000 });
      if (mode === 'late-error') {
        await page.evaluate(() => window.flockStatus('error', 'Review fixture: failure during reveal'));
        await page.waitForTimeout(1200);
        assert(await page.evaluate(() => window.flockReport.state === 'error' && !document.querySelector('#loading').hidden && !document.querySelector('#retry').hidden && document.querySelector('main').inert));
        await page.screenshot({ path: path.join(output, 'loading-late-error.png') });
        report.cases.push({ mode, simulatedError: true, remainedVisible: true });
        await context.close();
        continue;
      }
      await page.waitForTimeout(250);
      assert(await page.evaluate(() => document.querySelector('main').inert));
      await page.screenshot({ path: path.join(output, 'loading-reveal.png') });
    }
    await page.waitForFunction(() => window.flockReport?.state === 'ready', undefined, { timeout: 120_000 });
    const ready = await page.evaluate(() => ({ report: window.flockReport, animations: window.reviewAnimations, hidden: document.querySelector('#loading').hidden, inert: document.querySelector('main').inert, busy: document.querySelector('main').getAttribute('aria-busy'), opacity: getComputedStyle(document.querySelector('canvas')).opacity }));
    assert(ready.hidden && !ready.inert && ready.busy === 'false' && ready.opacity === '1');
    assert(ready.report.readyMs >= ready.report.rustReadyMs && ready.report.reveal.applicationFrames > 0);
    assert.equal(ready.report.reveal.reducedMotion, mode === 'reduced');
    assert(ready.animations.length > 0);
    const spatial = ready.animations.some(animation => animation.keyframes.some(frame => frame.transform && frame.transform !== 'none'));
    assert.equal(spatial, mode === 'normal');
    assert.deepEqual(errors, []);
    await page.screenshot({ path: path.join(output, `loading-ready-${mode}.png`) });
    report.cases.push({ mode, ready });
    await context.close();
  }
  report.pass = true;
} catch (error) {
  report.pass = false;
  report.error = String(error.stack || error);
  throw error;
} finally {
  await writeFile(path.join(output, 'loading-report.json'), JSON.stringify(report, null, 2));
  await browser.close();
}
console.log(`PASS: loading and reveal evidence in ${output}`);
