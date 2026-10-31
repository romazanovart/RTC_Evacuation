// Run real UI functions with a fake socket. Never contacts the robot.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const code = fs.readFileSync(require('node:path').join(__dirname, '../web/assets/app.js'), 'utf8');
const calls = [], sent = [];
const socket = {readyState: 1, bufferedAmount: 0, send: data => sent.push(JSON.parse(data))};
const state = {driveKeys: new Set(['KeyW']), joystick: {active: false}, driveClientId: 'test',
  driveSequence: 0, driveSocket: socket, driveTick: 10, driveTickAt: performance.now()};
state.driveHttpPending = false;
state.driveHttpTicket = null;
state.driveInputEpoch = 0;
const element = {style: {}, classList: {remove() {}}};
const context = vm.createContext({
  state, AbortSignal, performance, WebSocket: {OPEN: 1}, window: {setTimeout, clearTimeout},
  $: () => element, computeDriveCommand: () => ({linearX: .2, linearY: 0, angularZ: 0}),
  updateDrivePreview() {}, stopDriveLoop() {}, syncDriveKeyHighlights() {}, showToast() {},
  api: (path, options) => new Promise(resolve => calls.push({path, options, resolve})),
});
for (const [start, end] of [['function setDriveTransport(', 'function syncDriveKeyHighlights()'],
                            ['async function stopDrive()', 'async function issueGlobalStop()']]) {
  vm.runInContext(code.slice(code.indexOf(start), code.indexOf(end)), context);
}

(async () => {
  for (let i = 0; i < 20; i++) context.sendDriveCommand();
  assert.equal(sent.length, 20, 'Held key streams without waiting for replies');
  assert.equal(calls.length, 0, 'No HTTP request per motor update');
  socket.bufferedAmount = 1024;
  context.sendDriveCommand();
  assert.equal(sent.length, 20, 'Congestion does not queue additional motion');
  socket.bufferedAmount = 0;
  state.driveTickAt -= 500;
  context.sendDriveCommand();
  assert.equal(sent.length, 20, 'Stale connection does not send motion');
  assert.equal(calls[0].path, '/api/drive/ticket');
  const stop = context.stopDrive();
  assert.equal(sent[20].type, 'stop', 'STOP bypasses freshness check');
  assert.equal(calls.length, 2, 'STOP also uses an independent HTTP fallback');
  assert.equal(JSON.parse(calls[1].options.body).sequence, sent[20].sequence);
  context.sendDriveCommand();
  assert.equal(sent.length, 21, 'Released keys never resume');
  calls[0].resolve({ticket:'late-ticket'});
  calls[1].resolve({ok: true});
  await stop;
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(calls.length, 2, 'Ticket arriving after release must never create motion');
  assert.ok(!code.includes('Связь задерживается. Движение остановлено.'));
  assert.ok(!code.includes("button.addEventListener('pointerleave', release)"));
  assert.ok(!code.includes("if (state.driveKeys.size || state.joystick.active) stopDrive();"),
    'Failed WS reconnect must not clear a physically held key');
  console.log('PASS: continuous stream, bounded buffering, fresh input, stop priority, no warning popup');
  // Exercise actual keyboard handlers and combinations, not just the sender.
  const listeners = new Map(), elements = new Map();
  function el(selector) {
    if (!elements.has(selector)) elements.set(selector, {style:{}, value:selector === '#angular-speed' ? '.7' : '.2',
      classList:{remove(){},toggle(){},add(){}}, addEventListener(){}});
    return elements.get(selector);
  }
  Object.assign(context, {
    $: el, $$: () => [], localStorage:{setItem(){}}, connectDriveSocket(){},
    document:{activeElement:null,addEventListener(){}},
    window:{setTimeout,clearTimeout, addEventListener:(name,fn)=>listeners.set(name,fn)},
    startDriveLoop(){}, updateDriveOutputs(){},
  });
  state.page='drive'; state.driveMode='keyboard'; state.driveTickAt=performance.now();
  for (const [start,end] of [['function computeDriveCommand()', 'function updateDrivePreview('],
                            ['function shouldIgnoreDriveKeyEvent(', 'function bindRoutesPage()']]) {
    vm.runInContext(code.slice(code.indexOf(start),code.indexOf(end)),context);
  }
  context.bindDrivePage();
  assert.equal(context.shouldIgnoreDriveKeyEvent({tagName:'INPUT',type:'range'}),false,'Speed slider does not swallow WASD');
  for (const type of ['text','number','password']) {
    assert.equal(context.shouldIgnoreDriveKeyEvent({tagName:'INPUT',type}),true,'Typing must not drive');
  }
  const key=(code,repeat=false)=>({code,repeat,target:{tagName:'DIV'},preventDefault(){}});
  listeners.get('keydown')(key('KeyW'));
  assert.equal(sent.at(-1).linear_x,.2);
  const count=sent.length;
  listeners.get('keydown')(key('KeyW',true));
  assert.equal(sent.length,count,'OS key repeat does not flood control');
  listeners.get('keydown')(key('KeyA'));
  assert.equal(sent.at(-1).angular_z,.7);
  assert.equal(sent.at(-1).linear_x,.2);
  listeners.get('keyup')(key('KeyW'));
  assert.equal(sent.at(-1).linear_x,0);
  assert.equal(sent.at(-1).angular_z,.7);
  listeners.get('keyup')(key('KeyA'));
  assert.equal(sent.at(-1).type,'stop');
  calls.at(-1).resolve({ok:true});
  console.log('PASS: W, W+A, release W, release A, repeated keydown');

  state.driveSocket = null;
  state.driveTick = null;
  state.driveKeys.add('KeyW');
  const before = calls.length;
  context.sendDriveCommand();
  context.sendDriveCommand();
  assert.equal(calls.length,before+1,'Fallback requests bounded to one, no backlog');
  calls.at(-1).resolve({ticket:'fresh-test-ticket'});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(calls.at(-1).path,'/api/drive/command');
  const fallback = JSON.parse(calls.at(-1).options.body);
  assert.equal(fallback.http_ticket,'fresh-test-ticket');
  assert.equal(fallback.ui_version,'wasd-hold-v6');
  assert.equal(fallback.linear_x,.2);
  calls.at(-1).resolve({ok:true,ticket:'next-test-ticket'});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(state.driveHttpPending,false);
  assert.equal(state.driveHttpTicket,'next-test-ticket');
  const cachedStart = calls.length;
  context.sendDriveCommand();
  assert.equal(calls.length,cachedStart+1,'Next fallback command reuses server-issued ticket in one round trip');
  assert.equal(calls.at(-1).path,'/api/drive/command');
  calls.at(-1).resolve({ok:true,ticket:'third-test-ticket'});
  await new Promise(resolve=>setImmediate(resolve));
  console.log('PASS: unavailable WebSocket falls back to fresh sequenced HTTP; release cancels pending input');
})().catch(error => {console.error(error); process.exitCode = 1;});
