#!/usr/bin/env node
// Real headed browser acceptance. Run without a competing build, app or benchmark.
import assert from 'node:assert/strict';
import { mkdir, mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { spawn } from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import { createRequire } from 'node:module';

const require = createRequire(new URL('../web/package.json', import.meta.url));
const { chromium } = require('playwright');
const url = new URL(process.argv[2] || 'http://127.0.0.1:8080');
const output = path.resolve(process.argv[3] || path.join(os.tmpdir(), 'flock-browser-review'));
await mkdir(output, { recursive: true });
const args = process.env.AC3D_BROWSER_ARGS ? JSON.parse(process.env.AC3D_BROWSER_ARGS)
  : process.platform === 'linux'
    ? ['--enable-unsafe-webgpu', '--enable-features=Vulkan', '--use-angle=vulkan', '--ozone-platform=wayland'] : [];
const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM_EXECUTABLE || undefined,
  headless: process.env.AC3D_HEADLESS === '1',
  args,
});
const report = { url: url.href, browser: browser.version(), executable: process.env.CHROMIUM_EXECUTABLE || chromium.executablePath(), args, os: `${os.type()} ${os.release()} ${os.arch()}`, loadAverage: os.loadavg(), cases: [], performance: [] };

function attach(page, label) {
  const entry = { label, console: [], pageErrors: [], responses: [], failedRequests: [] };
  report.cases.push(entry);
  page.on('console', message => entry.console.push({ type: message.type(), text: message.text(), location: message.location() }));
  page.on('pageerror', error => entry.pageErrors.push(String(error)));
  page.on('response', response => { if (response.status() >= 400) entry.responses.push({ status: response.status(), url: response.url() }); });
  page.on('requestfailed', request => entry.failedRequests.push({ url: request.url(), error: request.failure()?.errorText }));
  return entry;
}
async function waitState(page, state = 'ready') {
  await page.waitForFunction(expected => window.flockReport?.state === expected, state, { timeout: 120_000 });
}
async function waitScene(page, name) {
  await page.waitForFunction(expected => window.flockReport?.state === 'ready' && window.flockReport?.scene?.name === expected && window.flockReport.scene.animated, name, { timeout: 120_000 });
  await page.waitForTimeout(200);
}
function summarize(values) {
  assert(values.length > 100, 'Too few rendered intervals for a performance sample');
  const sorted = [...values].sort((a, b) => a - b);
  const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
  return { frames: values.length, meanMs: mean, p50Ms: sorted[Math.floor(sorted.length * .5)], p95Ms: sorted[Math.floor(sorted.length * .95)], maxMs: sorted.at(-1), fps: 1000 / mean };
}
async function checkUnsupported(label, script) {
  const context = await browser.newContext({ viewport: { width: 1280, height: 720 } });
  await context.addInitScript(script);
  const page = await context.newPage();
  const entry = attach(page, label);
  await page.goto(url.href, { waitUntil: 'domcontentloaded' });
  await waitState(page, 'unsupported');
  assert(await page.locator('#message').isVisible());
  assert(await page.locator('#retry').isVisible());
  entry.report = await page.evaluate(() => window.flockReport);
  await page.screenshot({ path: path.join(output, `${label}.png`) });
  await context.close();
}

async function realVisibilityAndRecovery() {
  // A normal persistent context has real tab visibility and a disk cache. Playwright's ordinary
  // contexts force focus through their primary CDP session, which a second session cannot undo.
  const profile = await mkdtemp(path.join(os.tmpdir(), 'flock-browser-'));
  const processArgs = [...args, '--no-sandbox', '--no-first-run', '--no-default-browser-check', '--remote-debugging-port=0', `--user-data-dir=${profile}`, 'about:blank'];
  const processHandle = spawn(report.executable, processArgs, { stdio: 'ignore' });
  const exited = new Promise(resolve => processHandle.once('exit', resolve));
  let instance;
  try {
    let port;
    for (let attempt = 0; attempt < 100; attempt++) {
      try { port = (await readFile(path.join(profile, 'DevToolsActivePort'), 'utf8')).split('\n')[0]; break; }
      catch { await new Promise(resolve => setTimeout(resolve, 100)); }
    }
    assert(port, 'Disposable Chromium did not open its local DevTools endpoint');
    instance = await chromium.connectOverCDP(`http://127.0.0.1:${port}`, { noDefaults: true });
    const context = instance.contexts()[0];
    const page = context.pages()[0];
    const entry = attach(page, 'real-tab-visibility-cache-device-loss');
    entry.args = processArgs.filter(arg => !arg.startsWith('--user-data-dir='));
    await page.emulateMedia({ reducedMotion: 'reduce' });
    await page.goto(url.href, { waitUntil: 'domcontentloaded' });
    await waitState(page);
    const snapshot = () => page.evaluate(() => ({ readyMs: window.flockReport.readyMs, resources: performance.getEntriesByType('resource').map(({ name, transferSize, encodedBodySize, decodedBodySize }) => ({ name, transferSize, encodedBodySize, decodedBodySize })) }));
    entry.coldLoad = await snapshot();
    await page.reload({ waitUntil: 'domcontentloaded' });
    await waitState(page);
    entry.warmLoad = await snapshot();
    const background = await context.newPage();
    await background.goto('about:blank');
    await background.bringToFront();
    await page.waitForFunction(() => document.hidden, undefined, { timeout: 10_000 });
    await page.waitForTimeout(600);
    const before = await page.evaluate(() => window.flockReport.scene.simulationSeconds);
    await page.waitForTimeout(1000);
    const after = await page.evaluate(() => window.flockReport.scene.simulationSeconds);
    assert.equal(before, after, 'A real background tab must pause simulation');
    await page.bringToFront();
    await page.waitForFunction(() => !document.hidden && !window.flockReport.scene.paused);
    entry.visibility = { hiddenClockBefore: before, hiddenClockAfter: after, resumed: true };
    await background.close();
    assert.deepEqual(entry.pageErrors, []);
    assert.deepEqual(entry.responses, []);
    assert.deepEqual(entry.failedRequests, []);
    assert.deepEqual(entry.console.filter(message => message.type === 'error'), []);
    const control = await instance.newBrowserCDPSession();
    await control.send('Browser.crashGpuProcess');
    await waitState(page, 'error');
    assert.match(await page.locator('#message').innerText(), /graphics device/i);
    await page.screenshot({ path: path.join(output, 'device-loss.png') });
    entry.deviceLoss = await page.evaluate(() => window.flockReport.events.at(-1));
    await page.waitForTimeout(1000);
    await page.reload({ waitUntil: 'domcontentloaded' });
    await waitState(page);
    entry.recovered = await page.evaluate(() => window.flockReport.scene);
    await page.screenshot({ path: path.join(output, 'device-loss-recovered.png') });
  } finally {
    await instance?.close();
    processHandle.kill('SIGTERM');
    await exited;
    await rm(profile, { recursive: true });
  }
}

try {
  await checkUnsupported('unsupported-no-webgpu', () => Object.defineProperty(navigator, 'gpu', { value: undefined }));
  await checkUnsupported('unsupported-no-adapter', () => Object.defineProperty(navigator, 'gpu', { value: { requestAdapter: async () => null } }));
  await checkUnsupported('unsupported-limits', () => Object.defineProperty(navigator, 'gpu', { value: { requestAdapter: async () => ({ limits: { maxStorageBuffersPerShaderStage: 4, maxComputeInvocationsPerWorkgroup: 256, maxComputeWorkgroupSizeX: 256, maxComputeWorkgroupStorageSize: 16384 }, info: {} }) } }));

  const context = await browser.newContext({ viewport: { width: 1920, height: 1080 }, deviceScaleFactor: 1, reducedMotion: 'reduce' });
  const page = await context.newPage();
  const main = attach(page, 'visitor-flow');
  const cdp = await browser.newBrowserCDPSession();
  report.gpu = await cdp.send('SystemInfo.getInfo');
  await page.goto(url.href, { waitUntil: 'domcontentloaded' });
  await waitState(page);
  await page.waitForFunction(() => window.flockReport?.scene?.animated, undefined, { timeout: 120_000 });
  assert(await page.locator('#loading').isHidden());
  assert(await page.evaluate(() => window.flockReport.scene.reducedMotion));
  main.coldLoad = await page.evaluate(() => ({ readyMs: window.flockReport.readyMs, resources: performance.getEntriesByType('resource').map(({ name, transferSize, encodedBodySize, decodedBodySize }) => ({ name, transferSize, encodedBodySize, decodedBodySize })) }));
  report.release = await (await context.request.get(new URL('release.json', url).href)).json();
  const first = await page.screenshot({ path: path.join(output, 'visitor-start-1080p.png') });
  assert(first.length > 32_000, 'Presented screenshot looks blank; inspect the artifact and browser GPU/compositor support');

  for (let run = 1; run <= (process.env.AC3D_SKIP_PERFORMANCE === '1' ? 0 : 2); run++) {
    if (run === 2) {
      await page.reload({ waitUntil: 'domcontentloaded' });
      await waitState(page);
      main.warmLoad = await page.evaluate(() => ({ readyMs: window.flockReport.readyMs, resources: performance.getEntriesByType('resource').map(({ name, transferSize, encodedBodySize, decodedBodySize }) => ({ name, transferSize, encodedBodySize, decodedBodySize })) }));
    }
    await page.waitForTimeout(5000);
    await page.evaluate(() => { window.flockReport.frames = []; });
    await page.waitForTimeout(12000);
    const sample = await page.evaluate(() => ({ frames: window.flockReport.frames, scene: window.flockReport.scene, width: document.querySelector('canvas').width, height: document.querySelector('canvas').height }));
    report.performance.push({ run, resolution: [sample.width, sample.height], scene: sample.scene, warmupSeconds: 5, sampleSeconds: 12, loadAverage: os.loadavg(), ...summarize(sample.frames) });
  }

  await page.keyboard.press('p');
  await page.waitForFunction(() => window.flockReport.scene.paused);
  const worldClip = { x: 600, y: 150, width: 800, height: 650 };
  await page.waitForTimeout(300);
  const beforeHudDrag = await page.screenshot({ path: path.join(output, 'hud-drag-before.png'), clip: worldClip });
  for (const button of ['right', 'middle']) {
    await page.mouse.move(160, 165);
    await page.mouse.down({ button });
    await page.mouse.move(250, 180, { steps: 8 });
    await page.mouse.move(1000, 500, { steps: 8 });
    await page.mouse.up({ button });
    await page.waitForTimeout(300);
    assert(beforeHudDrag.equals(await page.screenshot({ path: path.join(output, `hud-drag-${button}.png`), clip: worldClip })), `${button} HUD drag escaped into the world camera`);
  }
  await page.mouse.move(160, 165);
  await page.mouse.wheel(0, 150);
  await page.waitForTimeout(300);
  assert(beforeHudDrag.equals(await page.screenshot({ path: path.join(output, 'hud-scroll.png'), clip: worldClip })), 'HUD scroll moved the world camera');
  // Egui renders into the canvas; these bounded clicks exercise the real HUD at scale factor 1.
  await page.mouse.click(48, 228);
  await page.waitForFunction(() => window.flockReport.scene.help);
  await page.screenshot({ path: path.join(output, 'visitor-help.png') });
  await page.mouse.click(48, 228);
  await page.waitForFunction(() => !window.flockReport.scene.help);
  await page.mouse.click(149, 228);
  await page.waitForFunction(() => window.flockReport.scene.lab);
  await page.screenshot({ path: path.join(output, 'lab.png') });
  await page.mouse.click(135, 92);
  await page.waitForFunction(() => !window.flockReport.scene.lab);
  await waitScene(page, 'Sunset murmuration');
  await page.keyboard.press('h');
  await page.waitForFunction(() => !window.flockReport.scene.uiVisible);
  await page.screenshot({ path: path.join(output, 'ui-hidden.png') });
  await page.keyboard.press('h');
  await page.waitForFunction(() => window.flockReport.scene.uiVisible);
  await page.keyboard.press('p');
  await page.waitForFunction(() => !window.flockReport.scene.paused);

  for (const [scene, name] of [['reef', 'Reef drift'], ['murmuration', 'Sunset murmuration'], ['shark', 'Shark hunt']]) {
    await page.evaluate(([scene]) => window.flock.flock_scene(scene, 'medium'), [scene]);
    await waitScene(page, name);
    const counts = [];
    for (const tier of ['low', 'medium', 'high']) {
      await page.evaluate(([scene, tier]) => window.flock.flock_scene(scene, tier), [scene, tier]);
      await waitScene(page, name);
      counts.push(await page.evaluate(() => window.flockReport.scene.count));
    }
    assert(counts[0] < counts[1] && counts[1] < counts[2], `${name} population tiers must increase`);
    await page.evaluate(([scene]) => window.flock.flock_scene(scene, 'medium'), [scene]);
    await waitScene(page, name);
    await page.waitForTimeout(2000);
    await page.screenshot({ path: path.join(output, `scene-${scene}.png`) });
  }
  await page.evaluate(() => { for (const scene of ['reef', 'shark', 'murmuration', 'reef', 'shark']) window.flock.flock_scene(scene, 'medium'); });
  await waitScene(page, 'Shark hunt');
  assert((await page.evaluate(() => window.flockReport.scene.hunters)) > 0);

  await page.keyboard.press('p');
  await page.waitForFunction(() => window.flockReport.scene.paused);
  await page.waitForTimeout(600);
  const clip = { x: 600, y: 150, width: 800, height: 650 };
  const before = await page.screenshot({ clip });
  await page.mouse.move(1000, 500);
  await page.mouse.down({ button: 'right' });
  await page.mouse.move(1200, 580, { steps: 12 });
  await page.mouse.move(170, 165, { steps: 12 });
  await page.waitForTimeout(100);
  const worldDragOverHud = await page.screenshot({ clip });
  await page.mouse.move(250, 180, { steps: 8 });
  await page.waitForTimeout(100);
  assert(!worldDragOverHud.equals(await page.screenshot({ clip })), 'World drag stopped when crossing the HUD');
  await page.mouse.up({ button: 'right' });
  await page.waitForTimeout(500);
  const after = await page.screenshot({ path: path.join(output, 'camera-drag-paused.png'), clip });
  assert(!before.equals(after), 'World drag must move the paused camera');
  await page.mouse.click(135, 165);
  await page.waitForTimeout(500);
  const reset = await page.screenshot({ path: path.join(output, 'camera-reset-paused.png'), clip });
  assert(!after.equals(reset), 'Reset view did not move the camera');
  assert(await page.evaluate(() => window.flockReport.scene.paused));
  await page.mouse.click(151, 140);
  await page.waitForFunction(() => window.flockReport.scene.camera === 'ride');
  await page.waitForTimeout(1000);
  await page.screenshot({ path: path.join(output, 'camera-ride.png') });
  await page.mouse.move(160, 165);
  await page.mouse.down({ button: 'right' });
  await page.mouse.move(250, 180, { steps: 8 });
  await page.mouse.up({ button: 'right' });
  await page.waitForTimeout(200);
  assert.equal(await page.evaluate(() => window.flockReport.scene.camera), 'ride', 'HUD drag cancelled Ride');
  await page.mouse.click(135, 165);
  await page.waitForFunction(() => window.flockReport.scene.camera === 'free');
  await page.mouse.click(38, 190);
  await page.waitForFunction(() => !window.flockReport.scene.reducedMotion && window.flockReport.scene.camera === 'scenic');
  await page.mouse.move(160, 165);
  await page.mouse.down({ button: 'middle' });
  await page.mouse.move(250, 180, { steps: 8 });
  await page.mouse.up({ button: 'middle' });
  await page.waitForTimeout(200);
  assert.equal(await page.evaluate(() => window.flockReport.scene.camera), 'scenic', 'HUD drag cancelled Scenic');
  await page.mouse.click(38, 190);
  await page.waitForFunction(() => window.flockReport.scene.reducedMotion && window.flockReport.scene.camera === 'free');
  await page.keyboard.press('p');
  await page.waitForFunction(() => !window.flockReport.scene.paused);

  await page.setViewportSize({ width: 900, height: 900 });
  await page.waitForFunction(() => document.querySelector('canvas').width === 900);
  await page.screenshot({ path: path.join(output, 'visitor-square.png') });
  await page.setViewportSize({ width: 600, height: 800 });
  await page.waitForFunction(() => document.querySelector('canvas').width === 600);
  await page.screenshot({ path: path.join(output, 'visitor-narrow.png') });
  main.report = await page.evaluate(() => window.flockReport);
  assert.equal(main.report.state, 'ready');
  assert.deepEqual(main.pageErrors, []);
  assert.deepEqual(main.responses, []);
  assert.deepEqual(main.failedRequests, []);
  assert.deepEqual(main.console.filter(message => message.type === 'error'), []);

  await context.close();

  // Chromium's device-pixel ResizeObserver does not follow a context-only DPR override.
  // Force the browser display factor too, so winit receives a consistent 2x pixel extent.
  const retinaArgs = [...args, '--force-device-scale-factor=2'];
  const retinaBrowser = await chromium.launch({ executablePath: report.executable, headless: false, args: retinaArgs });
  try {
    const retina = await retinaBrowser.newContext({ viewport: { width: 960, height: 540 }, deviceScaleFactor: 2, reducedMotion: 'reduce' });
    const retinaPage = await retina.newPage();
    const dpi = attach(retinaPage, 'high-dpi');
    dpi.args = retinaArgs;
    await retinaPage.goto(url.href, { waitUntil: 'domcontentloaded' });
    await waitState(retinaPage);
    await retinaPage.waitForFunction(() => document.querySelector('canvas').width === 1920);
    dpi.report = await retinaPage.evaluate(() => window.flockReport);
    await retinaPage.screenshot({ path: path.join(output, 'visitor-high-dpi.png') });
  } finally { await retinaBrowser.close(); }

  for (const [label, pattern] of [
    ['missing-environment', '**/*.flockenv'],
    ['missing-wasm', '**/flock_bg.wasm'],
    ['missing-shader', '**/vehicle_instancing.wesl'],
    ['missing-font', '**/FiraSans-Bold.ttf'],
    ['missing-catalog', '**/asset-list.json'],
  ]) {
    const failed = await browser.newContext({ viewport: { width: 1280, height: 720 } });
    await failed.route(pattern, route => route.fulfill({ status: 404, body: 'intentional smoke fixture' }));
    const failedPage = await failed.newPage();
    const entry = attach(failedPage, label);
    await failedPage.goto(url.href, { waitUntil: 'domcontentloaded' });
    await waitState(failedPage, 'error');
    assert(await failedPage.locator('#retry').isVisible());
    entry.report = await failedPage.evaluate(() => window.flockReport);
    await failedPage.screenshot({ path: path.join(output, `${label}.png`) });
    await failed.close();
  }
  await realVisibilityAndRecovery();
  report.result = 'PASS';
} catch (error) {
  report.result = 'FAIL';
  report.failure = error.stack || String(error);
  process.exitCode = 1;
} finally {
  await writeFile(path.join(output, 'smoke-report.json'), JSON.stringify(report, null, 2));
  await browser.close();
}
console.log(`${report.result}: browser evidence in ${output}`);
if (report.failure) console.error(report.failure);
