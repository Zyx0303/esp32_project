/* Behavioral tests for the embedded browser controller; no robot is contacted.
 * Run: node --test tests/test_phone_control.cjs
 */
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../components/robot_controller/control.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
class Target {
  constructor(value = '') {
    this.value = value; this.textContent = ''; this.disabled = false;
    this.dataset = {}; this.handlers = {}; this.classes = new Set();
    this.classList = {
      add: value => this.classes.add(value), remove: value => this.classes.delete(value),
      toggle: (value, yes) => yes ? this.classes.add(value) : this.classes.delete(value)
    };
  }
  addEventListener(name, fn) { (this.handlers[name] ||= []).push(fn); }
  emit(name, props = {}) {
    const event = {currentTarget: this, pointerId: 1, pointerType: 'touch', button: 0, preventDefault() {}, ...props};
    for (const fn of this.handlers[name] || []) fn(event);
  }
  setPointerCapture(id) { this.capture = id; }
}
async function flush() { for (let i = 0; i < 12; i++) await new Promise(resolve => setImmediate(resolve)); }
function harness() {
  const nodes = Object.fromEntries([...html.matchAll(/\bid="([^"]+)"/g)].map(match => [match[1], new Target()]));
  Object.assign(nodes.speed, {value: '10'}); Object.assign(nodes.angle, {value: '90'});
  nodes.leftMotor.value = nodes.rightMotor.value = '0';
  const directions = ['forward', 'left', 'right', 'backward'].map(direction => {
    const button = new Target(); button.dataset.direction = direction; return button;
  });
  const holds = directions;
  const actuators = [...holds, nodes.direct, nodes.servo, nodes.powerOn, nodes.powerOff];
  const document = new Target(), window = new Target();
  document.hidden = false;
  document.getElementById = id => nodes[id];
  document.querySelectorAll = selector => selector === '.hold' ? holds : actuators;
  const calls = [], intervals = [], timeouts = new Map();
  let now = 10000, timerId = 0, override = null;
  const status = {
    ok: true, armed: false, state: 'SAFE', faults: 0, drv8833_fault: false,
    motor_a: {output: 0}, motor_b: {output: 0}, imu: {valid: true}, power: false
  };
  const fetch = async (url, options) => {
    const call = {url, options, body: options.body ? JSON.parse(options.body) : undefined};
    calls.push(call);
    const intercepted = override && override(call);
    if (intercepted) return intercepted;
    if (url === '/api/v1/arm') { status.armed = true; status.state = 'READY'; }
    if (url === '/api/v1/disarm' || url === '/api/v1/fault/clear') { status.armed = false; status.state = 'SAFE'; }
    if (url === '/api/v1/estop') { status.armed = false; status.state = 'ESTOP'; }
    return {ok: true, status: 200, json: async () => url.endsWith('/status') ? structuredClone(status) : {ok: true, accepted: true, state: 'SAFE'}};
  };
  class Clock extends Date { static now() { return now; } }
  const context = vm.createContext({
    document, window, location: {host: '192.168.4.1'}, fetch, AbortController, Date: Clock,
    setInterval(fn, ms) { intervals.push({fn, ms}); },
    setTimeout(fn, ms) { timeouts.set(++timerId, {fn, ms}); return timerId; },
    clearTimeout(id) { timeouts.delete(id); }
  });
  vm.runInContext(script, context, {filename: 'control.html'});
  return {
    nodes, directions, document, window, calls, status,
    async arm() { nodes.arm.onclick(); await flush(); assert.equal(nodes.direct.disabled, false); },
    async tick(ms = 200) { now += ms; intervals.find(timer => timer.ms === ms).fn(); await flush(); },
    async poll() { intervals.find(timer => timer.ms === 500).fn(); await flush(); },
    override(fn) { override = fn; },
    advance(ms) { now += ms; },
    async delay() {
      for (const [id, timer] of timeouts) if (timer.ms === 100) { timeouts.delete(id); now += 100; timer.fn(); }
      await flush();
    },
    motors() { return calls.filter(call => call.url.endsWith('/drive') || call.url.endsWith('/motors')); }
  };
}
function deferredResponse() {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return {promise, resolve, reject};
}

test('load only reads status; locked controls cannot emit motion', async () => {
  const h = harness(); await flush();
  assert.deepEqual(h.calls.map(call => call.url), ['/api/v1/status']);
  assert.equal(h.nodes.direct.disabled, true);
  h.directions[0].emit('pointerdown'); await h.tick();
  assert.equal(h.motors().length, 0);
});
test('ARM waits for actual armed status instead of trusting enqueue ACK', async () => {
  const h = harness(); await flush();
  h.override(call => call.url.endsWith('/arm') ? Promise.resolve({ok: true, json: async () => ({ok: true, accepted: true})}) : null);
  h.nodes.arm.onclick(); await flush();
  for (let i = 0; i < 5; i++) await h.delay();
  assert.equal(h.nodes.direct.disabled, true);
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
});
test('ARM tolerates the control task processing its queue after the HTTP ACK', async () => {
  const h = harness(); await flush(); let statusRequests = 0;
  h.override(call => {
    if (call.url.endsWith('/arm')) return Promise.resolve({ok: true, json: async () => ({ok: true, accepted: true})});
    if (call.url.endsWith('/status') && ++statusRequests === 3) { h.status.armed = true; h.status.state = 'READY'; }
    return null;
  });
  h.nodes.arm.onclick(); await flush();
  assert.equal(h.nodes.direct.disabled, true);
  await h.delay(); await h.delay();
  assert.equal(h.nodes.direct.disabled, false);
  assert.equal(h.calls.filter(call => call.url.endsWith('/arm')).length, 1);
});
test('holding drives at 10%, repeats, and releasing immediately zeros without replay', async () => {
  const h = harness(); await flush(); await h.arm();
  h.directions[0].emit('pointerdown'); await flush(); await h.tick();
  assert.deepEqual(h.motors().map(call => call.body), [{throttle: 10, steering: 0}, {throttle: 10, steering: 0}]);
  h.window.emit('pointerup'); await flush();
  assert.deepEqual(h.motors().at(-1).body, {throttle: 0, steering: 0});
  const count = h.motors().length; await h.tick(); assert.equal(h.motors().length, count);
});
test('another finger cannot replace the held direction or release it', async () => {
  const h = harness(); await flush(); await h.arm();
  h.directions[0].emit('pointerdown'); await flush();
  h.directions[1].emit('pointerdown', {pointerId: 2});
  h.window.emit('pointerup', {pointerId: 2}); await h.tick();
  assert.deepEqual(h.motors().at(-1).body, {throttle: 10, steering: 0});
  h.window.emit('pointercancel'); await flush();
  assert.deepEqual(h.motors().at(-1).body, {throttle: 0, steering: 0});
});
test('a single click starts independent motors; lifting the finger keeps output until Stop', async () => {
  const h = harness(); await flush(); await h.arm();
  h.nodes.leftMotor.value = '-10'; h.nodes.rightMotor.value = '20';
  h.nodes.direct.onclick(); await flush();
  assert.equal(h.motors()[0].url, '/api/v1/motors');
  assert.deepEqual(h.motors()[0].body, {motor_a: -10, motor_b: 20});
  h.window.emit('pointerup'); h.window.emit('pointercancel'); h.nodes.direct.emit('lostpointercapture');
  await h.tick();
  assert.equal(h.motors().length, 2);
  assert.deepEqual(h.motors().at(-1).body, {motor_a: -10, motor_b: 20});
  h.nodes.directStop.onclick(); await flush();
  assert.deepEqual(h.motors().at(-1).body, {throttle: 0, steering: 0});
  assert.equal(h.nodes.direct.textContent, '开始输出');
  const count = h.motors().length; await h.tick(); assert.equal(h.motors().length, count);
});
test('sliders only change running independent targets after an Update click', async () => {
  const h = harness(); await flush(); await h.arm();
  h.nodes.leftMotor.value = '10'; h.nodes.rightMotor.value = '20';
  h.nodes.direct.onclick(); await flush();
  h.nodes.leftMotor.value = '-30'; h.nodes.rightMotor.value = '-40';
  await h.tick();
  assert.deepEqual(h.motors().at(-1).body, {motor_a: 10, motor_b: 20});
  h.nodes.direct.onclick(); await flush();
  assert.deepEqual(h.motors().at(-1).body, {motor_a: -30, motor_b: -40});
  h.nodes.stop.onclick(); await flush();
  assert.deepEqual(h.motors().at(-1).body, {throttle: 0, steering: 0});
});
test('independent continuous output still locks on background and never restarts on return', async () => {
  const h = harness(); await flush(); await h.arm();
  h.nodes.leftMotor.value = '10'; h.nodes.rightMotor.value = '10';
  h.nodes.direct.onclick(); await flush();
  h.document.hidden = true; h.document.emit('visibilitychange'); await flush();
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
  h.document.hidden = false; h.document.emit('visibilitychange'); await flush(); await h.tick();
  assert.equal(h.nodes.direct.disabled, true); assert.equal(h.motors().length, 1);
});
test('only one heartbeat is in flight; emergency stop aborts it', async () => {
  const h = harness(); await flush(); await h.arm();
  const pending = deferredResponse();
  h.override(call => {
    if (!call.url.endsWith('/drive')) return null;
    call.options.signal.addEventListener('abort', () => pending.reject(new Error('aborted')));
    return pending.promise;
  });
  h.directions[0].emit('pointerdown'); await h.tick(); await h.tick();
  assert.equal(h.motors().length, 1);
  h.nodes.estop.onclick(); await flush();
  assert.equal(h.motors()[0].options.signal.aborted, true);
  assert.equal(h.status.state, 'ESTOP'); assert.equal(h.nodes.direct.disabled, true);
  await h.poll(); assert.equal(h.nodes.arm.disabled, true);
  await h.tick(); assert.equal(h.motors().length, 1);
});
test('failed movement request locks and never resumes automatically after recovery', async () => {
  const h = harness(); await flush(); await h.arm();
  h.override(call => call.url.endsWith('/drive') ? Promise.reject(new Error('offline')) : null);
  h.directions[0].emit('pointerdown'); await flush();
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
  h.override(null); await h.poll(); await h.tick();
  assert.equal(h.nodes.direct.disabled, true); assert.equal(h.motors().length, 1);
});
test('stale telemetry stops movement even if motor requests still succeed', async () => {
  const h = harness(); await flush(); await h.arm();
  h.directions[0].emit('pointerdown'); await flush();
  h.advance(1600); await h.tick();
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
  assert.equal(h.nodes.direct.disabled, true);
});
test('hiding the page sends keepalive DISARM and requires explicit ARM on return', async () => {
  const h = harness(); await flush(); await h.arm();
  h.directions[0].emit('pointerdown'); await flush();
  h.document.hidden = true; h.document.emit('visibilitychange'); await flush();
  const disarm = h.calls.find(call => call.url.endsWith('/disarm'));
  assert.equal(disarm.options.keepalive, true);
  h.document.hidden = false; h.document.emit('visibilitychange'); await flush(); await h.tick();
  assert.equal(h.nodes.direct.disabled, true); assert.equal(h.motors().length, 1);
});
test('late ARM response after pagehide cannot authorize motion', async () => {
  const h = harness(); await flush(); const pending = deferredResponse();
  h.override(call => call.url.endsWith('/arm') ? pending.promise : null);
  h.nodes.arm.onclick(); h.window.emit('pagehide'); await flush();
  pending.resolve({ok: true, json: async () => ({ok: true, accepted: true})}); await flush();
  assert.equal(h.nodes.direct.disabled, true);
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
});
test('stale status from before ARM cannot undo a confirmed unlock', async () => {
  const h = harness(); await flush(); const pending = deferredResponse(); let intercept = true;
  h.override(call => {
    if (call.url.endsWith('/status') && intercept) { intercept = false; return pending.promise; }
    return null;
  });
  void h.poll(); await h.arm();
  pending.resolve({ok: true, json: async () => ({...structuredClone(h.status), armed: false, state: 'SAFE'})}); await flush();
  assert.equal(h.nodes.direct.disabled, false);
});
test('servo and GPIO use explicit validated payloads without automatic replay', async () => {
  const h = harness(); await flush(); await h.arm();
  h.nodes.angle.value = '45'; h.nodes.servo.onclick(); await flush();
  h.nodes.powerOn.onclick(); await flush(); h.nodes.powerOff.onclick(); await flush();
  assert.deepEqual(h.calls.filter(call => call.url.endsWith('/servo')).map(call => call.body), [{angle: 45}]);
  assert.deepEqual(h.calls.filter(call => call.url.endsWith('/power')).map(call => call.body), [{asserted: true}, {asserted: false}]);
  const count = h.calls.length; await h.tick(); assert.equal(h.calls.length, count);
});
test('HTTP errors are shown and cause a lock request', async () => {
  const h = harness(); await flush(); await h.arm();
  h.override(call => call.url.endsWith('/servo') ? Promise.resolve({ok: false, status: 503, json: async () => ({ok: false, error: 'command_queue_unavailable'})}) : null);
  h.nodes.servo.onclick(); await flush();
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
  assert.equal(h.nodes.direct.disabled, true);
});
test('failed release stop also attempts DISARM and drops local authorization', async () => {
  const h = harness(); await flush(); await h.arm();
  h.directions[0].emit('pointerdown'); await flush();
  h.override(call => call.url.endsWith('/drive') && call.body.throttle === 0 ? Promise.reject(new Error('stop failed')) : null);
  h.window.emit('pointerup'); await flush();
  assert(h.calls.some(call => call.url.endsWith('/disarm')));
  assert.equal(h.nodes.direct.disabled, true);
  assert.match(h.nodes.message.textContent, /停止请求未确认/);
});
test('an offline event locks even when telemetry still reports armed after reconnect', async () => {
  const h = harness(); await flush(); await h.arm();
  h.override(call => call.url.endsWith('/disarm') ? Promise.reject(new Error('offline')) : null);
  h.window.emit('offline'); await flush();
  h.override(null); await h.poll();
  assert.equal(h.status.armed, true);
  assert.equal(h.nodes.direct.disabled, true);
  h.directions[0].emit('pointerdown'); await h.tick();
  assert.equal(h.motors().length, 0);
});
