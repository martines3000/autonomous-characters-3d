const loading = document.querySelector('#loading');
const message = document.querySelector('#message');
const detail = document.querySelector('#detail');
const retry = document.querySelector('#retry');
const canvas = document.querySelector('#flock');
const started = performance.now();

// Kept outside the visitor HUD. The smoke runner records these actual application frame intervals.
window.flockReport = { state: 'loading', started, events: [], frames: [], errors: [] };
window.flockStatus = (state, text) => {
  const report = window.flockReport;
  const previous = report.events.at(-1);
  if (previous?.state === state && previous.text === text) return;
  report.state = state;
  report.events.push({ state, text, at: performance.now() - started });
  loading.hidden = state === 'ready';
  message.textContent = text;
  retry.hidden = state !== 'error' && state !== 'unsupported';
  detail.textContent = state === 'ready' ? '' : state === 'error'
    ? 'Reload to start a fresh scene.'
    : 'Designed for desktop browsers with WebGPU.';
  if (state === 'ready') {
    if (report.readyMs === undefined) {
      report.readyMs = performance.now() - started;
      canvas.focus({ preventScroll: true });
    }
  }
};
window.flockFrame = (milliseconds, count, paused, animated, hunters, uiVisible, lab, reducedMotion, help, name, simulationSeconds, camera, hunterCamera = false, firstPerson = false, cameraAnimalId = 0xffffffff) => {
  const frames = window.flockReport.frames;
  if (milliseconds > 0) frames.push(milliseconds);
  if (frames.length > 7200) frames.shift();
  window.flockReport.scene = { count, paused, animated, hunters, uiVisible, lab, reducedMotion, help, name, simulationSeconds, camera, cameraSubject: hunterCamera ? 'hunter' : 'prey', firstPerson, cameraAnimalId: cameraAnimalId === 0xffffffff ? null : cameraAnimalId };
};
window.flockReducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;
const assets = new Set();
window.flockHasAsset = path => assets.has(path);
window.flockHasDirectory = path => [...assets].some(asset => asset.startsWith(`${path}/`));
retry.addEventListener('click', () => location.reload());
canvas.addEventListener('contextmenu', event => event.preventDefault());
canvas.addEventListener('pointerdown', () => canvas.focus({ preventScroll: true }));

function fail(error) {
  const text = String(error?.message || error);
  window.flockReport.errors.push(text);
  console.error('Flock startup/runtime error:', error);
  window.flockStatus('error', 'The scene could not start. Check your connection and graphics support, then try again.');
}
window.addEventListener('error', event => fail(event.error || event.message));
window.addEventListener('unhandledrejection', event => fail(event.reason));

async function start() {
  if (!window.isSecureContext || !navigator.gpu) {
    window.flockStatus('unsupported', 'This browser cannot open the living world. Use a desktop browser with WebGPU over HTTPS.');
    return;
  }
  const adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
  if (!adapter) {
    window.flockStatus('unsupported', 'No compatible graphics device is available. Enable hardware acceleration, then try again.');
    return;
  }
  const required = {
    maxStorageBuffersPerShaderStage: 8,
    maxComputeInvocationsPerWorkgroup: 256,
    maxComputeWorkgroupSizeX: 256,
    maxComputeWorkgroupStorageSize: 16384,
  };
  const missing = Object.entries(required).filter(([name, value]) => adapter.limits[name] < value);
  window.flockReport.adapter = { info: { vendor: adapter.info?.vendor, architecture: adapter.info?.architecture, description: adapter.info?.description }, limits: Object.fromEntries(Object.keys(required).map(name => [name, adapter.limits[name]])) };
  if (missing.length) {
    window.flockReport.unsupportedLimits = missing.map(([name]) => name);
    window.flockStatus('unsupported', 'This graphics device does not support the flock simulation. Try another desktop device.');
    return;
  }
  window.flockStatus('loading', 'Opening a small world…');
  const catalog = await fetch('./asset-list.json');
  if (!catalog.ok) throw new Error('The asset catalog could not be loaded.');
  for (const path of await catalog.json()) assets.add(path);
  window.flock = await import('./flock.js');
  await window.flock.default();
}

start().catch(fail);
