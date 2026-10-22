// Browser integration against an isolated local service with empty motion data;
// never connects hardware. Uses Playwright Chromium unless CHROME_PATH is set.
import {chromium} from '../online_motion_track/node_modules/@playwright/test/index.mjs';
import assert from 'node:assert/strict';
import {mkdir, readFile} from 'node:fs/promises';

const base = process.env.MOTION_URL || 'http://127.0.0.1:8873';
const browser = await chromium.launch({
  ...(process.env.CHROME_PATH ? {executablePath:process.env.CHROME_PATH} : {}), headless: true,
  args:['--use-angle=swiftshader','--enable-unsafe-swiftshader'],
});
const page = await browser.newPage({viewport:{width:1500,height:1100}});
page.setDefaultTimeout(30000);
const deadline = async (promise, description) => {
  let timer;
  try {
    return await Promise.race([promise, new Promise((_, reject) => {
      timer = setTimeout(() => reject(Error('Timed out: ' + description)), 30000);
    })]);
  } finally { clearTimeout(timer); }
};
const errors = [], failedAssets = [];
let statePolls = 0;
const displayedModels = [];
page.on('pageerror', error => errors.push(error.message));
page.on('request', request => { if (request.url().endsWith('/api/state')) statePolls++; });
page.on('response', response => { if (/\/(assets|vendor|api)\//.test(response.url()) && !response.ok()) failedAssets.push(response.url()); });
const snapshot = async () => {
  const response = await page.request.get(base + '/api/state', {headers:{'X-Motion-Client':'a32-range-readonly-browser'}});
  assert.equal(response.ok(), true);
  return response.json();
};
const checkRanges = state => {
  assert.equal(state.connected, false, 'offline smoke must never connect a hand');
  assert.equal(state.joints.length, 20);
  for (const joint of state.joints) {
    const expected = joint.name.includes('_swing_') ? [-.35,.35] : /_(MCP|PIP|DIP)_/.test(joint.name) ? [.005,1.565] : null;
    if (expected) assert.deepEqual([joint.min,joint.max], expected);
  }
};
const ensureControl = async () => {
  assert.equal((await snapshot()).control.available, false, 'model loading must not starve the controller lease');
  assert.equal(await page.locator('#control').textContent(), '释放控制');
};
const waitModel = async () => {
  const state = await snapshot();
  await page.waitForFunction(url => document.querySelector('#modelNote').textContent === '' &&
    document.querySelector('#viewer canvas')?.dataset.modelUrl === url, state.urdf);
  displayedModels.push(state.urdf);
  await ensureControl();
};

try {
  await page.goto(base);
  await page.waitForFunction(() => document.querySelector('#control').textContent === '释放控制');
  for (const model of ['TsenseI_v3_2', 'TsenseI_v3_2_withid']) {
    console.log('MODEL', model);
    await page.locator('#modelSelect').selectOption(model);
    await page.waitForFunction(model => document.querySelector('#fileNote').textContent.includes(model.endsWith('_withid') ? 'motions_a32_withid_left' : 'motions_a32_left'), model);
    await waitModel();
    assert.equal(await page.locator('#joints input[type=range]').count(), 20);
    const state = await snapshot();
    checkRanges(state);
    for (const joint of state.joints) {
      if (joint.name.includes('_swing_')) assert.equal(joint.value, 0);
    }
    assert.match(await page.locator('#jointNote').textContent(), /\[-0\.35, 0\.35\].*\[-0\.34, 0\.34\]/);
    assert.equal(await page.locator('#a32Motors').isVisible(), true);
    assert.equal(await page.locator('#a30Motors').isVisible(), false);
    assert.equal(await page.locator('#ports').isVisible(), false);
    assert.equal(await page.locator('#a32Connection').isVisible(), true);
    assert.equal(await page.locator('#a32Enable button').isDisabled(), true);
    await page.locator('#a32Transport').selectOption('eth');
    assert.equal(await page.locator('#a32Host').isVisible(), true);
    await page.locator('#a32Transport').selectOption('usb');
    const joint = page.locator('#joints input[type=number]').first();
    await joint.fill('0.1'); await joint.press('Enter');
    await page.locator('#savemotion button').click();
    await page.locator('#dialog-items input[type=text]').fill('A32 浏览器验证');
    await page.getByRole('button', {name:'确认',exact:true}).click();
    await page.waitForFunction(() => document.querySelector('#motionCount').textContent === '1 个动作');
    await page.evaluate(() => {
      const select = document.querySelector('#choosemotion select');
      select.focus();
      window.motionSelectMutations = [];
      window.motionSelectObserver = new MutationObserver(records => window.motionSelectMutations.push(...records.map(r => r.type)));
      window.motionSelectObserver.observe(select, {attributes:true,childList:true,subtree:true});
    });
    const pollsBefore = statePolls;
    while (statePolls < pollsBefore + 5) await page.waitForTimeout(100);
    assert.deepEqual(await page.evaluate(() => {
      window.motionSelectObserver.disconnect();
      document.querySelector('#choosemotion select').blur();
      return window.motionSelectMutations;
    }), [], 'polling must not repaint an open motion selector');
    const downloadPromise = page.waitForEvent('download');
    await page.locator('#export').click();
    const download = await downloadPromise;
    assert(download.suggestedFilename().includes(model));
    const payload = JSON.parse(await readFile(await download.path(), 'utf8'));
    assert.equal(payload._model_id, model);
    assert.equal(payload._hand_side, 'left');
    assert.equal(payload._joint_coordinate_version, 'a32-signed-swing-v2');
    assert.equal(payload._swing_axis_alignment, 'id-cad-direction-v2');
    assert.equal(Object.keys(payload.motions['A32 浏览器验证'].pose).length, 20);
    assert.equal(payload.motions['A32 浏览器验证'].pose.left_hand_pinky_DIP_joint, .1);
    await page.locator('#motionstreamplay button').click();
    await page.waitForFunction(() => document.querySelector('#status').textContent.includes('正在播放'));
    await page.waitForFunction(() => document.querySelector('#status').textContent.includes('就绪'), null, {timeout:6000});
    await page.locator('#handSide select').selectOption('1');
    await page.waitForFunction(() => document.querySelector('#fileNote').textContent.includes('_right.json'));
    await waitModel();
    checkRanges(await snapshot());
    assert.equal(await page.locator('#joints input[type=range]').count(), 20);
    assert.equal(await page.locator('#connect').isDisabled(), false, 'both A3.2 hands support local SDK connection');
    assert.equal(await page.locator('#motionCount').innerText(), '0 个动作');
    await page.locator('#handSide select').selectOption('0');
    await page.waitForFunction(() => document.querySelector('#fileNote').textContent.includes('_left.json'));
    await waitModel();
    assert.equal(await page.locator('#motionCount').innerText(), '1 个动作');
  }
  const output = process.env.MOTION_QA_OUTPUT || '.work/a32-browser';
  await mkdir(output, {recursive:true});
  await page.screenshot({path:output + '/motion-play-withid.png',fullPage:true});
  for (const model of ['TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v2']) {
    console.log('MODEL', model);
    await page.locator('#modelSelect').selectOption(model);
    const file = {TsenseI_v3_0:'motions_a30_left.json', TsenseI_v3_0_withid:'motions_a30_withid_left.json', TsenseI_v2:'motions.json'}[model];
    await page.waitForFunction(file => document.querySelector('#fileNote').textContent.includes(file), file);
    await waitModel();
    assert.equal(await page.locator('#a32Motors').isVisible(), false);
  }
  // Hold a replacement load in flight, then return to the already displayed
  // model. The cancelled request must never replace the current hand or leave
  // a stale loading/error message behind.
  await page.locator('#modelSelect').selectOption('TsenseI_v3_2_withid');
  await page.waitForFunction(() => document.querySelector('#fileNote').textContent.includes('motions_a32_withid_left'));
  await waitModel();
  const selected = await snapshot(), rightURL = selected.urdf.replace('_left_', '_right_');
  let releaseRequest, requestStarted, requestFinished;
  const held = new Promise(resolve => { releaseRequest = resolve; });
  const started = new Promise(resolve => { requestStarted = resolve; });
  const finished = new Promise(resolve => { requestFinished = resolve; });
  const holdRight = async route => { requestStarted(); await held; await route.continue(); requestFinished(); };
  await page.route('**' + rightURL, holdRight);
  await page.locator('#handSide select').selectOption('1');
  await deadline(started, 'intercepting the right-hand model request');
  await page.waitForFunction(() => document.querySelector('#modelNote').textContent.includes('正在载入'));
  await page.locator('#handSide select').selectOption('0');
  await page.waitForFunction(() => document.querySelector('#fileNote').textContent.includes('_left.json'));
  await waitModel();
  releaseRequest();
  await deadline(finished, 'releasing the cancelled model request');
  await page.unroute('**' + rightURL, holdRight);
  const pollCount = statePolls;
  await page.waitForFunction(() => document.querySelector('#modelNote').textContent === '');
  // Wait for the ordinary poll to run again after the old request is released.
  await deadline(new Promise(resolve => {
    const listener = request => {
      if (request.url().endsWith('/api/state') && statePolls > pollCount) {
        page.off('request', listener); resolve();
      }
    };
    page.on('request', listener);
  }), 'state polling after the cancelled model request');
  await waitModel();
  assert.deepEqual(errors, []);
  assert.deepEqual(failedAssets, []);
  console.log(JSON.stringify({ok:true, models:5, a32_sides:4, save_export_playback:true, controlReacquisitions:0, rapid_switch:true, displayedModels:displayedModels.length, errors, failedAssets}));
} finally {
  await browser.close();
}
