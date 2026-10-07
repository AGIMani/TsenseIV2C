// Run against the isolated QA server with FakeDevice; never a real hand.
import {chromium} from '../online_motion_track/node_modules/@playwright/test/index.mjs';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const base = process.env.MOTION_QA_URL || 'http://127.0.0.1:8875';
const browser = await chromium.launch({headless:true,
  executablePath:process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe'});
const page = await browser.newPage();
const errors = [];
page.on('pageerror', e => errors.push(e.message));
const snapshot = async () => {
  const response = await page.request.get(base + '/api/state', {headers:{'X-Motion-Client':'a32-readonly-lifecycle-test'}});
  assert.equal(response.ok(), true);
  return response.json();
};
try {
  await page.goto(base);
  await page.waitForFunction(() => document.getElementById('control').textContent === '释放控制');
  await page.locator('#modelSelect').selectOption('TsenseI_v3_2_withid');
  await page.waitForFunction(() => !document.getElementById('a32Connection').hidden);
  await page.locator('#connect').click();
  await page.waitForFunction(() => document.getElementById('connection').textContent.includes('SDK 已连接'));
  const realtime = page.locator('#realtimeJointControl input');
  assert.equal(await realtime.isEnabled(), true);
  await realtime.check();
  await page.waitForFunction(() => document.getElementById('a32Telemetry').textContent.includes('等待关节或动作指令'));
  await new Promise(resolve => setTimeout(resolve, 250));
  assert.equal((await snapshot()).device.sample_ticks, 0);
  const editJoint = async value => {
    const input = page.locator('#joints input[type=number]').first();
    await input.fill(String(value)); await input.press('Enter');
  };
  await editJoint(.15);
  await page.waitForFunction(() => document.getElementById('a32Telemetry').textContent.includes('轨迹下发中'));
  await page.evaluate(() => {
    Object.defineProperty(document, 'hidden', {configurable:true, get:() => true});
    document.dispatchEvent(new Event('visibilitychange'));
  });
  await page.waitForFunction(() => document.getElementById('a32Message').textContent.includes('page_hidden'));
  let state = await snapshot();
  assert.equal(state.connected, true);
  assert.equal(state.device.armed, false);
  assert.equal(await realtime.isEnabled(), true);
  await page.evaluate(() => {
    Object.defineProperty(document, 'hidden', {configurable:true, get:() => false});
    document.dispatchEvent(new Event('visibilitychange'));
  });
  await realtime.check();
  await page.waitForFunction(() => document.getElementById('a32Telemetry').textContent.includes('等待关节或动作指令'));
  const pausedTicks = (await snapshot()).device.sample_ticks;
  await new Promise(resolve => setTimeout(resolve, 250));
  assert.equal((await snapshot()).device.sample_ticks, pausedTicks);
  await editJoint(.2);
  await page.waitForFunction(() => document.getElementById('a32Telemetry').textContent.includes('轨迹下发中'));
  // Drop only the owner's polls. Lease expiry must stop motion, retaining USB.
  await page.route('**/api/state', route => route.abort());
  await new Promise(resolve => setTimeout(resolve, 6200));
  state = await snapshot();
  assert.equal(state.connected, true);
  assert.equal(state.device.armed, false);
  assert.equal(state.device.stop_reason, 'controller_heartbeat_timeout');
  assert.equal(state.control.available, true);
  await page.unroute('**/api/state');
  await page.waitForFunction(() => document.getElementById('control').textContent === '获取控制');
  await page.locator('#control').click();
  await page.waitForFunction(() => document.getElementById('control').textContent === '释放控制');
  await realtime.check();
  await page.waitForFunction(() => document.getElementById('a32Telemetry').textContent.includes('等待关节或动作指令'));
  await editJoint(.25);
  await page.waitForFunction(() => document.getElementById('a32Telemetry').textContent.includes('轨迹下发中'));
  await page.locator('#connect').click();
  await page.waitForFunction(() => document.getElementById('connection').textContent === '○ SDK 未连接');
  const downloadEvent = page.waitForEvent('download');
  await page.locator('#a32DownloadLog').click();
  const download = await downloadEvent;
  const rows = (await readFile(await download.path(), 'utf8')).trim().split('\n').map(JSON.parse);
  assert.equal(rows[0].schema, 'hand-control-session/v2');
  assert(rows.every(r => Number.isFinite(r.wall_time_unix_ms) && /[+-]\d\d:\d\d$|Z$/.test(r.wall_time)));
  assert(rows.some(r => r.type === 'downlink' && r.sent));
  assert(!rows.some(r => r.source === 'a32_enable_seed'));
  assert(rows.some(r => r.type === 'sdk_action' && r.reason === 'page_hidden'));
  assert(rows.some(r => r.type === 'sdk_action' && r.reason === 'controller_heartbeat_timeout'));
  assert.equal(rows.at(-1).type, 'session_end');
  assert.equal((await page.request.get(base + '/control-log-viewer')).ok(), true);
  await page.goto(base + '/control-log-viewer');
  await page.locator('#usageModel').selectOption('A32');
  await page.locator('#usageRefreshFolder').click();
  await page.waitForFunction(() => document.querySelectorAll('#usageLogFile option').length > 1);
  await page.locator('#usageLogFile').selectOption(download.suggestedFilename());
  await page.locator('#usageOpenLog').click();
  await page.waitForFunction(name => document.getElementById('status').textContent.startsWith(name + ' ·'), download.suggestedFilename());
  assert((await page.locator('#sessionTimeRange').innerText()).length > 15);
  assert.equal(await page.locator('#controlPeriodCanvas').count(), 1);
  assert.deepEqual(errors, []);
  console.log('A32_LIFECYCLE_OK: hidden page pauses, connected recovery, lease expiry retains link, worker, viewer JSONL download');
} finally { await browser.close(); }
