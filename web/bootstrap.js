// Fetch streams are decoded. A compressed Content-Length must never be their denominator.
export function decodedTotal(headers, expectedBytes) {
  if (Number.isSafeInteger(expectedBytes) && expectedBytes > 0) return expectedBytes;
  const encoding = headers.get('Content-Encoding')?.trim().toLowerCase();
  if (encoding && encoding !== 'identity') return null;
  const length = headers.get('Content-Length');
  if (!length || !/^\d+$/.test(length)) return null;
  const bytes = Number(length);
  return Number.isSafeInteger(bytes) && bytes > 0 ? bytes : null;
}

export function downloadProgress(received, total, done = false) {
  // Revoke a denominator if the server body contradicts it, rather than inventing 100%.
  const known = total !== null && received <= total && (!done || received === total);
  return { received, total: known ? total : null, done };
}

export function meterResponse(response, expectedBytes, update) {
  if (!response.ok) return response;
  const total = decodedTotal(response.headers, expectedBytes);
  let received = 0;
  let cancelled = false;
  if (!response.body) {
    update(downloadProgress(0, total, true));
    return response;
  }
  const reader = response.body.getReader();
  update(downloadProgress(0, total));
  const body = new ReadableStream({
    async pull(controller) {
      try {
        const { done, value } = await reader.read();
        if (cancelled) return;
        if (done) {
          update(downloadProgress(received, total, true));
          controller.close();
          reader.releaseLock();
        } else {
          received += value.byteLength;
          update(downloadProgress(received, total));
          controller.enqueue(value);
        }
      } catch (error) {
        update({ ...downloadProgress(received, null), failed: true });
        controller.error(error);
        reader.releaseLock();
      }
    },
    async cancel(reason) {
      cancelled = true;
      try { await reader.cancel(reason); }
      finally { reader.releaseLock(); }
    },
  });
  return new Response(body, {
    status: response.status, statusText: response.statusText, headers: response.headers,
  });
}

// The caller waits for an application-frame signal first. Then allow two paint opportunities.
export async function revealAfterFrames(nextFrame, isCurrent, animate, finish) {
  for (let frame = 0; frame < 2; frame++) {
    await nextFrame();
    if (!isCurrent()) return false;
  }
  await animate();
  if (!isCurrent()) return false;
  finish();
  return true;
}

export function formatBytes(bytes) {
  if (bytes < 1_000_000) return `${Math.round(bytes / 1000)} kB`;
  return `${(bytes / 1_000_000).toFixed(1)} MB`;
}

function boot() {
  const loading = document.querySelector('#loading');
  const message = document.querySelector('#message');
  const detail = document.querySelector('#detail');
  const progress = document.querySelector('#download-progress');
  const retry = document.querySelector('#retry');
  const canvas = document.querySelector('#flock');
  const main = document.querySelector('main');
  const started = performance.now();
  const downloads = new Map();
  let readyText = null;
  let revealStarted = false;
  let revealGeneration = 0;
  let applicationFrames = 0;
  let firstReadyFrame = 0;
  let animations = [];
  let progressScheduled = false;

  // Diagnostics stay outside the visitor HUD. Byte counts are decoded data, not wire traffic.
  const report = window.flockReport = {
    state: 'loading', started, events: [], frames: [], errors: [], downloads: {},
  };
  const now = () => performance.now() - started;
  const terminal = () => report.state === 'error' || report.state === 'unsupported';
  const nextFrame = () => new Promise(resolve => requestAnimationFrame(resolve));
  window.flockReducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

  function stage(text, description = 'Preparing animals, light and space.') {
    if (report.state !== 'loading' || readyText) return;
    const previous = report.events.at(-1);
    if (previous?.text !== text) report.events.push({ state: 'loading', text, at: now() });
    if (message.textContent !== text) {
      message.textContent = text;
      detail.textContent = description;
      progress.removeAttribute('value');
      progress.setAttribute('aria-label', text);
    }
    progress.hidden = false;
    report.progress = { stage: 'preparing', text, received: null, total: null };
  }

  function renderProgress() {
    progressScheduled = false;
    if (report.state !== 'loading' || readyText) return;
    // One named resource, never a guessed total for the changing set of startup requests.
    const active = [...downloads.values()].filter(item => !item.done && !item.failed)
      .sort((a, b) => (b.total ?? b.received) - (a.total ?? a.received))[0];
    if (!active) {
      stage('Preparing animals and light…');
      report.progress = { stage: 'preparing', received: null, total: null };
      return;
    }
    stage(active.label);
    report.progress = { stage: 'download', resource: active.path, received: active.received, total: active.total };
    if (active.total !== null) {
      progress.max = active.total;
      progress.value = active.received;
      detail.textContent = `${formatBytes(active.received)} of ${formatBytes(active.total)} · unpacked scene data`;
    } else {
      progress.removeAttribute('value');
      detail.textContent = `${formatBytes(active.received)} of scene data received`;
    }
  }

  function resourceLabel(path) {
    if (path === 'flock_bg.wasm') return 'Opening a living world…';
    if (path.endsWith('sunset.flockenv')) return 'Gathering the evening light…';
    if (path.endsWith('underwater.flockenv')) return 'Finding the ocean light…';
    if (path.endsWith('midday.flockenv')) return 'Gathering the daylight…';
    return 'Preparing the scene…';
  }

  function installDownloadMeter(catalog) {
    const base = new URL('.', document.baseURI);
    const sizes = catalog.decodedBytes ?? {};
    const known = new Map(['flock_bg.wasm', ...catalog.assets.map(path => `assets/${path}`)]
      .map(path => [new URL(path, base).href, path]));
    const originalFetch = window.fetch.bind(window);
    window.fetch = async (input, init) => {
      const response = await originalFetch(input, init);
      const url = new URL(input instanceof Request ? input.url : input, document.baseURI);
      const path = url.origin === base.origin ? known.get(url.href) : undefined;
      if (!path) return response;
      return meterResponse(response, sizes[path], sample => {
        const item = { ...sample, path, label: resourceLabel(path) };
        downloads.set(path, item);
        report.downloads[path] = { ...sample, unit: 'decoded-bytes' };
        if (!progressScheduled && report.state === 'loading' && !readyText) {
          progressScheduled = true;
          requestAnimationFrame(renderProgress);
        }
      });
    };
  }

  async function animateReveal() {
    if (typeof loading.animate !== 'function' || typeof canvas.animate !== 'function') return;
    const reduced = window.flockReducedMotion();
    animations = [
      loading.animate([{ opacity: 1 }, { opacity: 0 }], {
        duration: reduced ? 160 : 650, easing: 'ease-out', fill: 'forwards',
      }),
      canvas.animate(reduced
        ? [{ opacity: 0 }, { opacity: 1 }]
        : [{ opacity: 0, transform: 'scale(1.015)' }, { opacity: 1, transform: 'scale(1)' }], {
        duration: reduced ? 160 : 850, easing: 'cubic-bezier(.2,.7,.2,1)', fill: 'forwards',
      }),
    ];
    await Promise.allSettled(animations.map(animation => animation.finished));
  }

  function maybeReveal() {
    if (!readyText || revealStarted || terminal() || applicationFrames <= firstReadyFrame) return;
    revealStarted = true;
    const generation = revealGeneration;
    report.progress = { stage: 'starting', received: null, total: null };
    message.textContent = 'Your world is coming into view…';
    detail.textContent = 'A moment to settle into the wild.';
    progress.removeAttribute('value');
    const isCurrent = () => generation === revealGeneration && !terminal();
    revealAfterFrames(nextFrame, isCurrent, animateReveal, () => {
      canvas.style.opacity = '1';
      canvas.style.transform = 'none';
      loading.hidden = true;
      for (const animation of animations) animation.cancel();
      animations = [];
      main.inert = false;
      main.setAttribute('aria-busy', 'false');
      report.state = 'ready';
      report.events.push({ state: 'ready', text: readyText, at: now() });
      report.readyMs ??= now();
      report.reveal = { reducedMotion: window.flockReducedMotion(), applicationFrames, completedMs: now() };
      canvas.focus({ preventScroll: true });
    }).catch(fail);
  }

  window.flockStatus = (state, text) => {
    if (state === 'ready') {
      if (terminal() || report.state === 'ready' || readyText) return;
      report.rustReadyMs = now();
      readyText = text;
      firstReadyFrame = applicationFrames;
      maybeReveal();
      return;
    }
    if (state === 'loading') {
      stage(text);
      return;
    }
    // A late failure invalidates the pending reveal and keeps the recovery screen visible.
    revealGeneration++;
    for (const animation of animations) animation.cancel();
    animations = [];
    const previous = report.events.at(-1);
    report.state = state;
    if (previous?.state !== state || previous.text !== text) report.events.push({ state, text, at: now() });
    loading.hidden = false;
    main.inert = true;
    main.setAttribute('aria-busy', 'false');
    message.textContent = text;
    detail.textContent = state === 'error' ? 'Reload to start a fresh scene.' : 'Designed for desktop browsers with WebGPU.';
    progress.hidden = true;
    retry.hidden = state !== 'error' && state !== 'unsupported';
    if (document.activeElement === canvas) retry.focus({ preventScroll: true });
  };
  window.flockFrame = (milliseconds, count, paused, animated, hunters, uiVisible, lab, reducedMotion, help, name, simulationSeconds, camera, hunterCamera = false, firstPerson = false, cameraAnimalId = 0xffffffff) => {
    if (milliseconds > 0) report.frames.push(milliseconds);
    if (report.frames.length > 7200) report.frames.shift();
    report.scene = { count, paused, animated, hunters, uiVisible, lab, reducedMotion, help, name, simulationSeconds, camera, cameraSubject: hunterCamera ? 'hunter' : 'prey', firstPerson, cameraAnimalId: cameraAnimalId === 0xffffffff ? null : cameraAnimalId };
    applicationFrames++;
    maybeReveal();
  };
  const assets = new Set();
  window.flockHasAsset = path => assets.has(path);
  window.flockHasDirectory = path => [...assets].some(asset => asset.startsWith(`${path}/`));
  retry.addEventListener('click', () => location.reload());
  canvas.addEventListener('contextmenu', event => event.preventDefault());
  canvas.addEventListener('pointerdown', () => canvas.focus({ preventScroll: true }));

  function fail(error) {
    const text = String(error?.message || error);
    report.errors.push(text);
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
    stage('Finding your window into the wild…', 'Checking the graphics on your device.');
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
    report.adapter = { info: { vendor: adapter.info?.vendor, architecture: adapter.info?.architecture, description: adapter.info?.description }, limits: Object.fromEntries(Object.keys(required).map(name => [name, adapter.limits[name]])) };
    if (missing.length) {
      report.unsupportedLimits = missing.map(([name]) => name);
      window.flockStatus('unsupported', 'This graphics device does not support the flock simulation. Try another desktop device.');
      return;
    }
    stage('Opening a living world…');
    const response = await fetch('./asset-list.json');
    if (!response.ok) throw new Error('The asset catalog could not be loaded.');
    const catalog = await response.json();
    if (!Array.isArray(catalog.assets) || catalog.assets.some(path => typeof path !== 'string')) {
      throw new Error('The asset catalog is invalid.');
    }
    for (const path of catalog.assets) assets.add(path);
    installDownloadMeter(catalog);
    window.flock = await import('./flock.js');
    await window.flock.default();
  }
  start().catch(fail);
}

// Importing the helpers for dependency-free Node tests does not start a browser application.
if (typeof window !== 'undefined' && typeof document !== 'undefined') boot();
