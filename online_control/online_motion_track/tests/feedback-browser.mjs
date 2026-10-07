// Real models and feedback API, with test-only telemetry injection; no device access.
import {chromium} from '@playwright/test';
import {spawn, execFileSync} from 'node:child_process';
import {mkdtemp, mkdir, rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';

const root = fileURLToPath(new URL('../../../', import.meta.url));
const python = process.env.HAND_PYTHON || 'python';
const logs = await mkdtemp(join(tmpdir(), 'hand-feedback-'));
const fixtures = JSON.parse(execFileSync(python, ['-c', `
import json, sys, struct
sys.path[:0]=['.', 'src']
from tests.test_a30_control import feedback
from a30_kinematics import A30StrokeMapper
from a30_kinematics.downlink import channel_mapping
from a32_driver import A32Device
from hand_profiles import load_hand_profile
mapper=A30StrokeMapper('left')
pose=load_hand_profile('TsenseI_v3_0_withid').init_pose('left')
pose['left_hand_index_PIP_joint']=.55
pose['left_hand_ring_MCP_joint']=.3
partial_pose={**pose,'left_hand_index_PIP_joint':.8,'left_hand_ring_MCP_joint':1.2}
partial_packet=bytearray(feedback(mapper,partial_pose))
slot=next(r['slot'] for r in channel_mapping('left') if r['joint']=='left_hand_ring_MCP_joint')
struct.pack_into('<I',partial_packet,8,0x3ffff & ~(1 << slot))
device=A32Device(model_id='TsenseI_v3_2_withid')
a32pose=load_hand_profile('TsenseI_v3_2_withid').init_pose('left')
a32pose['left_hand_index_PIP_joint']=.55
targets=device.prepare_targets(a32pose)
state={'connected':True,'telemetry_age_ms':0,'hand_side':'left',
 'self':[{'online':True,'position':v} for v in targets['self']],
 'mit':[{'online':True,'valid':True,'fault':0,'position':v} for v in targets['mit_pos']]}
print(json.dumps({'payload':feedback(mapper,pose).hex(),'partialPayload':partial_packet.hex(),
 'joint':'left_hand_index_PIP_joint','sdkState':state}))
`], {cwd: root, encoding: 'utf8'}));
const server = spawn(python, ['-u', '-c', `
import sys
from online_control.online_motion_track.main import make_server
s=make_server(0,sys.argv[1],host='127.0.0.1',collision_mode='off')
print('FEEDBACK_URL=http://127.0.0.1:'+str(s.server_port),flush=True)
s.serve_forever()
`, logs], {cwd: root, stdio: ['ignore', 'pipe', 'pipe']});
let browser;
try {
  const url = await new Promise((resolve, reject) => {
    let output = '';
    const timer = setTimeout(() => reject(Error('Feedback fixture startup timeout')), 30000);
    server.stdout.on('data', chunk => {
      output += chunk;
      const match = output.match(/FEEDBACK_URL=(http:\/\/\S+)/);
      if (match) { clearTimeout(timer); resolve(match[1]); }
    });
    server.stderr.on('data', chunk => process.stderr.write(chunk));
    server.once('exit', code => { clearTimeout(timer); reject(Error(`Fixture exited ${code}`)); });
  });
  browser = await chromium.launch({headless: true,
    ...(process.env.CHROME_PATH ? {executablePath: process.env.CHROME_PATH} : {}),
    args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
  const page = await browser.newPage({viewport: {width: 1715, height: 1150}});
  page.setDefaultTimeout(30000);
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  // Expose closures only in this browser's intercepted test copy of app.js.
  await page.route('**/app.js', async route => {
    const response = await route.fetch();
    await route.fulfill({response, body: await response.text() + `
window.feedbackTest = {get control(){return localControl;}, get viewer(){return viewer;},
  get feedback(){return feedbackViewer;}, sdk:a32Panel, refresh:refreshFeedback};
`});
  });
  await page.goto(url + '/?model=TsenseI_v3_0_withid');
  await page.waitForFunction(() => window.feedbackTest && !document.querySelector('#start').disabled);
  assert.deepEqual(await page.locator('.workspace h2').allTextContents(),
    ['原始关键点输入', '原始模型跟随', '真实模型反馈']);
  assert.equal(await page.locator('#feedback-stage canvas').count(), 1);
  assert.equal(await page.evaluate(() => feedbackTest.feedback.robot.root.visible), false);
  const rects = await page.locator('.workspace > .panel').evaluateAll(nodes => nodes.map(n => {
    const r=n.getBoundingClientRect(); return {x:r.x,y:r.y,width:r.width,height:r.height};
  }));
  assert(rects.every(r => r.y === rects[0].y && Math.abs(r.width-rects[0].width)<1 && r.height===rects[0].height));
  await page.evaluate(({payload}) => {
    window.feedbackPayload=payload;
    window.feedFeedback = () => {
      const payload=window.feedbackPayload;
      const c=feedbackTest.control;
      c.connected=true;
      c.latestPayload=payload;
      c.latest=c.protocol.telemetry(Uint8Array.from(payload.match(/../g), b=>parseInt(b,16)));
      c.telemetryAt=performance.now();
      feedbackTest.refresh();
    };
    feedFeedback(); window.feedbackFeedTimer=setInterval(feedFeedback,100);
  }, fixtures);
  await page.waitForFunction(() => document.querySelector('#feedback-dot').classList.contains('online'));
  const readAngle = () => page.evaluate(joint => {
    const q=feedbackTest.feedback.robot.joints.get(joint).motion.quaternion;
    return 2*Math.acos(Math.min(1,Math.abs(q.w)));
  }, fixtures.joint);
  assert(Math.abs(await readAngle()-.55)<1e-4);
  await page.evaluate(joint => feedbackTest.viewer.setPose({[joint]: 1.1}), fixtures.joint);
  assert(Math.abs(await readAngle()-.55)<1e-4, 'target changes must not drive feedback');
  await mkdir(join(root,'logs/feedback_validation'),{recursive:true});
  await page.locator('.workspace').screenshot({path:join(root,'logs/feedback_validation/three-columns.png')});
  await page.evaluate(payload => {window.feedbackPayload=payload;feedFeedback();}, fixtures.partialPayload);
  await page.waitForFunction(() => {
    const q=feedbackTest.feedback.robot.joints.get('left_hand_index_PIP_joint').motion.quaternion;
    return Math.abs(2*Math.acos(Math.min(1,Math.abs(q.w)))-.8)<1e-4;
  });
  await page.waitForFunction(() => document.querySelector('#feedback-status').textContent.includes('缺失关节保持'));
  assert(Math.abs(await page.evaluate(() => {
    const q=feedbackTest.feedback.robot.joints.get('left_hand_ring_MCP_joint').motion.quaternion;
    return 2*Math.acos(Math.min(1,Math.abs(q.w)));
  })-.3)<1e-4, 'offline joint must keep its own last angle while other joints update');
  await page.evaluate(() => clearInterval(window.feedbackFeedTimer));
  await page.waitForFunction(() => document.querySelector('#feedback-status').textContent.includes('过期'));
  assert(Math.abs(await readAngle()-.8)<1e-4, 'stale feedback holds its last measured pose');
  await page.evaluate(() => {feedbackTest.control.connected=false;feedbackTest.refresh();});
  await page.waitForFunction(() => document.querySelector('#feedback-status').textContent.includes('未连接'));
  assert.equal(await page.evaluate(() => feedbackTest.feedback.robot.root.visible), true);
  assert(Math.abs(await readAngle()-.8)<1e-4, 'disconnect keeps the model and last angles visible');
  await page.locator('#side').selectOption('right');
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  assert.match(await page.locator('#feedback-caption').innerText(), /RIGHT/);
  assert.equal(await page.evaluate(() => feedbackTest.feedback.robot.root.visible), false);
  await page.locator('#hand-model').selectOption('TsenseI_v3_2_withid');
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  await page.waitForFunction(() => document.querySelector('#feedback-status').textContent.includes('待标定'));
  await page.locator('#side').selectOption('left');
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  await page.evaluate(state => {
    window.feedSdk = () => feedbackTest.sdk.update({...state});
    feedSdk(); window.feedbackFeedTimer=setInterval(feedSdk,100);
  }, fixtures.sdkState);
  await page.waitForFunction(() => document.querySelector('#feedback-dot').classList.contains('online'));
  assert(Math.abs(await readAngle()-.55)<1e-4, 'SDK positions must independently drive the feedback URDF');
  await page.evaluate(() => {
    clearInterval(window.feedbackFeedTimer);
    feedbackTest.sdk.update({connected:false});
    feedbackTest.refresh();
  });
  await page.locator('#input-source').selectOption('wuji_glove');
  assert.equal(await page.locator('#camera-title').innerText(), '原始关键点输入');
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(() => document.documentElement.scrollWidth<=innerWidth));
  await page.locator('.workspace').screenshot({path:join(root,'logs/feedback_validation/three-columns-mobile.png')});
  assert.deepEqual(errors, []);
  console.log('PASS: three columns, A30/SDK URDF feedback conversion, independent target/measurement, stale/disconnect, model/side switch, mobile');
} finally {
  await browser?.close();
  server.kill();
  await new Promise(resolve => {if(server.exitCode!==null)resolve();else server.once('exit',resolve);});
  await rm(logs,{recursive:true,force:true});
}
