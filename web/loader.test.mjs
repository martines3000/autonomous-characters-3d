import assert from 'node:assert/strict';
import test from 'node:test';
import { decodedTotal, downloadProgress, meterResponse, revealAfterFrames } from './bootstrap.js';

const bytes = (...values) => new Uint8Array(values);
function response(chunks, headers = {}) {
  return new Response(new ReadableStream({
    start(controller) {
      for (const chunk of chunks) controller.enqueue(chunk);
      controller.close();
    },
  }), { headers });
}

test('compressed Content-Length is not a decoded stream total', () => {
  const compressed = new Headers({ 'Content-Encoding': 'gzip', 'Content-Length': '2' });
  assert.equal(decodedTotal(compressed), null);
  assert.equal(decodedTotal(compressed, 100), 100);
  assert.equal(decodedTotal(new Headers({ 'Content-Length': '100' })), 100);
  assert.equal(decodedTotal(new Headers()), null);
  assert.equal(decodedTotal(new Headers({ 'Content-Length': 'not-a-size' })), null);
});

test('metering preserves decoded bytes with a raw manifest total under gzip', async () => {
  const samples = [];
  const original = response([bytes(1, 2, 3), bytes(4, 5, 6, 7)], {
    'Content-Encoding': 'gzip', 'Content-Length': '2', 'Content-Type': 'application/wasm',
  });
  const metered = meterResponse(original, 7, sample => samples.push(sample));
  assert.equal(metered.headers.get('Content-Type'), 'application/wasm');
  assert.deepEqual(new Uint8Array(await metered.arrayBuffer()), bytes(1, 2, 3, 4, 5, 6, 7));
  assert.deepEqual(samples.at(-1), { received: 7, total: 7, done: true });
  assert(samples.every(sample => sample.total === null || sample.received <= sample.total));
});

test('unknown totals retain actual bytes without inventing a percentage', async () => {
  const samples = [];
  const metered = meterResponse(response([bytes(1, 2, 3)], {
    'Content-Encoding': 'br', 'Content-Length': '1',
  }), undefined, sample => samples.push(sample));
  await metered.arrayBuffer();
  assert.deepEqual(samples.at(-1), { received: 3, total: null, done: true });
});

test('short and overlong bodies revoke an incorrect denominator', async () => {
  assert.deepEqual(downloadProgress(11, 10), { received: 11, total: null, done: false });
  for (const total of [2, 10]) {
    const samples = [];
    await meterResponse(response([bytes(1, 2, 3)]), total, sample => samples.push(sample)).arrayBuffer();
    assert.deepEqual(samples.at(-1), { received: 3, total: null, done: true });
  }
});

test('HTTP failures are preserved for the existing application error path', () => {
  const failed = new Response('missing', { status: 404 });
  assert.equal(meterResponse(failed, 7, () => assert.fail('not a successful download')), failed);
});

test('stream failures reach the consumer and discard the incomplete total', async () => {
  const samples = [];
  let first = true;
  const original = new Response(new ReadableStream({
    pull(controller) {
      if (first) { first = false; controller.enqueue(bytes(1, 2)); }
      else controller.error(new Error('connection interrupted'));
    },
  }));
  const metered = meterResponse(original, 10, sample => samples.push(sample));
  await assert.rejects(metered.arrayBuffer(), /connection interrupted/);
  assert.equal(samples.at(-1).failed, true);
  assert.equal(samples.at(-1).total, null);
});

test('cancelling a metered body cancels its underlying stream', async () => {
  let reason;
  const original = new Response(new ReadableStream({ cancel(value) { reason = value; } }));
  await meterResponse(original, 10, () => {}).body.cancel('scene superseded');
  assert.equal(reason, 'scene superseded');
});

test('ready waits for paint opportunities and the completed reveal', async () => {
  const paints = [];
  let finishAnimation;
  let finished = false;
  const reveal = revealAfterFrames(
    () => new Promise(resolve => paints.push(resolve)),
    () => true,
    () => new Promise(resolve => { finishAnimation = resolve; }),
    () => { finished = true; },
  );
  assert.equal(paints.length, 1);
  assert.equal(finished, false);
  paints[0]();
  await Promise.resolve();
  assert.equal(paints.length, 2);
  assert.equal(finished, false);
  paints[1]();
  await Promise.resolve();
  assert.equal(finished, false);
  finishAnimation();
  assert.equal(await reveal, true);
  assert.equal(finished, true);
});

test('an error before painting or during animation cannot reveal the scene', async () => {
  let current = true;
  const beforePaint = await revealAfterFrames(
    async () => { current = false; },
    () => current,
    () => assert.fail('error state must not animate'),
    () => assert.fail('error state must not publish ready'),
  );
  assert.equal(beforePaint, false);
  current = true;
  const duringAnimation = await revealAfterFrames(
    async () => {},
    () => current,
    async () => { current = false; },
    () => assert.fail('device loss must retain the recovery screen'),
  );
  assert.equal(duringAnimation, false);
});
