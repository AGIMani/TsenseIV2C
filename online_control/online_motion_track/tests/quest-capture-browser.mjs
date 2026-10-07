// Browser integration uses real ES modules and WebGL, with an XR API shim.
// It requires no headset, camera, serial port, or running robot backend.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';

const received = [];
async function waitReceived(predicate) {
  const until = Date.now() + 3000;
  while (!predicate(received) && Date.now() < until) await new Promise(resolve => setTimeout(resolve, 10));
  assert.ok(predicate(received), 'expected XR message was not published');
}
const files = new Map([
  ['/q', ['quest.html', 'text/html']],
  ['/quest.html', ['quest.html', 'text/html']],
  ['/quest-capture.js', ['quest-capture.js', 'text/javascript']],
  ['/quest-capture-core.js', ['quest-capture-core.js', 'text/javascript']],
]);
const server = createServer(async (req, res) => {
  if (req.url === '/api/webxr/pair' && req.method === 'POST') {
    let body='';for await (const chunk of req) body+=chunk;
    const valid=JSON.parse(body).code==='123456';
    res.writeHead(valid?200:400,{'Content-Type':'application/json'});
    res.end(JSON.stringify(valid?{token:'browser-secret'}:{error:'配对码无效'}));return;
  }
  if (req.url === '/api/webxr/publish' && req.method === 'POST') {
    let body = '';
    for await (const chunk of req) body += chunk;
    received.push(JSON.parse(body));
    res.setHeader('Content-Type', 'application/json');
    res.end(JSON.stringify({ok: true, accepted: true}));
    return;
  }
  const file = files.get(req.url);
  if (!file) { res.writeHead(404).end(); return; }
  res.setHeader('Content-Type', file[1]);
  res.end(await readFile(new URL(`../static/${file[0]}`, import.meta.url)));
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
  browser = await chromium.launch({
    executablePath: process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe',
    headless: true,
    args:process.env.HAND_BROWSER_SWIFTSHADER==='1'?['--use-angle=swiftshader','--enable-unsafe-swiftshader']:[],
  });
  const page = await browser.newPage();
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.addInitScript(() => {
    const identity = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
    window.xrTest = {requests: [], session: null, missingJoint: null, failNext: false, deferNext: false};
    class Session extends EventTarget {
      constructor() {
        super();
        this.visibilityState = 'visible';
        this.renderState = {};
        this.inputSources = [{handedness: 'left', hand: {get: name => name}}];
      }
      updateRenderState(state) { this.renderState = state; }
      requestReferenceSpace() { return Promise.resolve({}); }
      requestAnimationFrame(callback) { this.callback = callback; }
      end() { this.ended = true; this.dispatchEvent(new Event('end')); return Promise.resolve(); }
    }
    Object.defineProperty(navigator, 'xr', {value: {
      requestSession(mode, options) {
        window.xrTest.requests.push({mode, options: {requiredFeatures: options.requiredFeatures}, activation: navigator.userActivation.isActive});
        if (window.xrTest.failNext) { window.xrTest.failNext = false; return Promise.reject(new Error('unsupported mode')); }
        const session = new Session();
        window.xrTest.session = session;
        if (window.xrTest.deferNext) {
          window.xrTest.deferNext = false;
          return new Promise(resolve => { window.xrTest.resolveSession = () => resolve(session); });
        }
        return Promise.resolve(session);
      },
    }});
    WebGLRenderingContext.prototype.makeXRCompatible = () => Promise.resolve();
    window.XRWebGLLayer = class {
      constructor() { this.framebuffer = null; }
      getViewport() { return {x: 0, y: 0, width: 300, height: 150}; }
    };
    window.xrTest.frame = () => window.xrTest.session.callback?.(performance.now(), {
      getViewerPose: () => ({views: [{projectionMatrix: identity, transform: {inverse: {matrix: identity}}}]}),
      getJointPose: name => name === window.xrTest.missingJoint ? null : ({
        transform: {position: {x: 0.1, y: 0.2, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}, radius: 0.008,
      }),
    });
  });
  await page.addInitScript(()=>localStorage.setItem('quest-control-pairing','revoked-capability'));
  await page.goto(`http://127.0.0.1:${server.address().port}/q`);
  assert.equal(await page.locator('#pair-token').inputValue(), '123456');
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  await page.reload();
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  assert.equal(await page.locator('#pair-token').inputValue(), '123456');
  assert.equal(new URL(page.url()).hash, '');
  await page.locator('#start').click();
  await page.waitForFunction(() => Boolean(window.xrTest.session?.callback));
  await page.evaluate(() => window.xrTest.frame());
  await page.waitForFunction(() => document.querySelector('#status').textContent.includes('正在采集'));
  await waitReceived(items => items.some(item => item.message.type === 'hand_skeleton'));
  const first = received.find(item => item.message.type === 'hand_skeleton');
  assert.equal(first.token, 'browser-secret');
  assert.equal(first.message.hands[0].valid_joint_count, 25);
  assert.equal(first.message.hands[1].tracked, false);
  assert.deepEqual(await page.evaluate(() => window.xrTest.requests), [{mode: 'immersive-ar', options: {requiredFeatures: ['hand-tracking']}, activation: true}]);

  await page.evaluate(() => { window.xrTest.missingJoint = 'index-finger-tip'; window.xrTest.frame(); });
  await waitReceived(items => items.some(item => item.message.type === 'hand_skeleton' && item.message.hands[0].valid_joint_count === 24));
  const partial = received.filter(item => item.message.type === 'hand_skeleton').at(-1).message;
  assert.equal(partial.hands[0].tracked, false);
  assert.equal(partial.hands[0].joints[9].position, null);
  assert.ok(received.some(item => item.message.type === 'tracking_lost'));
  await page.locator('#stop').click();
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  const sentAtStop = received.length;
  await page.evaluate(() => window.xrTest.frame());
  await new Promise(resolve => setTimeout(resolve, 30));
  assert.equal(received.length, sentAtStop, 'ended frame callback cannot publish another pose');
  assert.equal(received.at(-1).message.type, 'session_ended');
  assert.equal(received.filter(item => item.message.type === 'session_ended').length, 1);

  // A browser permission failure is never followed by an automatic second XR request.
  await page.evaluate(() => { window.xrTest.failNext = true; });
  await page.locator('#start').click();
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  assert.equal(await page.evaluate(() => window.xrTest.requests.length), 2);
  assert.match(await page.locator('#status').innerText(), /启动失败/);
  await page.locator('#session-mode').selectOption('immersive-vr');
  await page.locator('#start').click();
  await page.waitForFunction(() => Boolean(window.xrTest.session?.callback));
  await page.evaluate(() => window.xrTest.frame());
  await waitReceived(items => items.some(item => item.message.type === 'hand_skeleton' && item.message.stream_id !== first.message.stream_id));
  assert.notEqual(received.filter(item => item.message.type === 'hand_skeleton').at(-1).message.stream_id, first.message.stream_id);
  await page.evaluate(() => {
    window.xrTest.session.visibilityState = 'visible-blurred';
    window.xrTest.session.dispatchEvent(new Event('visibilitychange'));
  });
  await waitReceived(items=>items.at(-1)?.message.type==='tracking_lost'
    && items.at(-1)?.message.reason==='xr_not_visible');
  assert.equal(await page.evaluate(() => Boolean(window.xrTest.session.ended)), false);
  assert.equal(await page.locator('#start').isDisabled(),true);
  const pausedCount=received.length;
  await page.evaluate(() => {window.xrTest.frame();window.xrTest.frame();});
  await new Promise(resolve=>setTimeout(resolve,30));
  assert.equal(received.length,pausedCount,'blurred XR cannot send poses');
  await page.evaluate(() => {
    window.xrTest.missingJoint=null;
    window.xrTest.session.visibilityState='visible';
    window.xrTest.session.dispatchEvent(new Event('visibilitychange'));
    window.xrTest.frame();
  });
  await waitReceived(items=>items.length>pausedCount && items.at(-1).message.type==='hand_skeleton');
  assert.equal(await page.evaluate(()=>window.xrTest.requests.length),3,'resume uses existing XR session');
  assert.equal(received.at(-1).message.hands[0].tracked,true);
  const beforeHidden=received.length;
  await page.evaluate(()=>{
    Object.defineProperty(document,'visibilityState',{configurable:true,value:'hidden'});
    document.dispatchEvent(new Event('visibilitychange'));
    window.xrTest.frame();
  });
  await waitReceived(items=>items.length>beforeHidden && items.at(-1).message.type==='tracking_lost');
  await page.evaluate(()=>{
    Object.defineProperty(document,'visibilityState',{configurable:true,value:'visible'});
    document.dispatchEvent(new Event('visibilitychange'));
    window.xrTest.frame();
  });
  await waitReceived(items=>items.at(-1).message.type==='hand_skeleton');
  await page.evaluate(()=>window.xrTest.session.end());
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  assert.equal(received.at(-1).message.type,'session_ended');

  // Cancelling during the browser permission prompt cannot revive a cancelled stream.
  await page.evaluate(() => { window.xrTest.deferNext = true; });
  await page.locator('#start').click();
  await page.locator('#stop').click();
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  const beforeLatePermission = received.length;
  await page.evaluate(() => window.xrTest.resolveSession());
  await page.waitForFunction(() => window.xrTest.session.ended);
  assert.equal(received.length, beforeLatePermission);
  assert.deepEqual(errors, []);
  console.log('Quest browser lifecycle passed: native25, loss, single user gesture, restart, visibility pause/resume, explicit end, cancelled permission.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
