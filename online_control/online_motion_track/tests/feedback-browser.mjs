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
import json, sys
sys.path[:0]=['.', 'src']
from a32_driver import A32Device
from hand_profiles import load_hand_profile
device=A32Device(model_id='TsenseI_v3_2_withid')
pose=load_hand_profile('TsenseI_v3_2_withid').init_pose('left')
pose['left_hand_index_PIP_joint']=.55
swing_joints=[name for name in pose if '_swing_' in name]
sdk_states=[]
for angle in (-.2,0.,.2):
 pose.update({name:angle for name in swing_joints})
 targets=device.prepare_targets(pose,clamp=False)
 state={'connected':True,'telemetry_age_ms':0,'hand_side':'left','model_id':'TsenseI_v3_2_withid',
  'self':[{'online':True,'position':v if i in (6,7,14,15) else -v,'target':12.}
          for i,v in enumerate(targets['self'])],
  'mit':[{'online':True,'valid':True,'fault':0,'position':v} for v in targets['mit_pos']]}
 sdk_states.append({'angle':angle,'state':state})
print(json.dumps({'joint':'left_hand_index_PIP_joint','sdkStates':sdk_states,'swingJoints':swing_joints}))
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
  const page = await browser.newPage({viewport: {width: 1715, height: 1150}, deviceScaleFactor: .5});
  page.setDefaultTimeout(30000);
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  // Expose closures only in this browser's intercepted test copy of app.js.
  await page.route('**/app.js', async route => {
    const response = await route.fetch();
    await route.fulfill({response, body: await response.text() + `
window.feedbackTest = {get viewer(){return viewer;},
  get feedback(){return feedbackViewer;}, sdk:a32Panel, refresh:refreshFeedback};
`});
  });
  await page.goto(url + '/?model=TsenseI_v3_2_withid');
  await page.waitForFunction(() => window.feedbackTest && !document.querySelector('#start').disabled);
  assert.deepEqual(await page.locator('.workspace h2').allTextContents(),
    ['原始关键点输入', '手势骨架（优化后）', '原始模型跟随', '真实模型反馈']);
  assert.equal(await page.locator('#feedback-stage canvas').count(), 1);
  assert.equal(await page.evaluate(() => feedbackTest.feedback.robot.root.visible), false);
  const rects = await page.locator('.workspace > .panel').evaluateAll(nodes => nodes.map(n => {
    const r=n.getBoundingClientRect(); return {x:r.x,y:r.y,width:r.width,height:r.height};
  }));
  assert(rects.every(r => Math.abs(r.width-rects[0].width)<1));
  assert(rects[0].y===rects[1].y && rects[2].y===rects[3].y && rects[2].y>rects[0].y);
  const readAngle = () => page.evaluate(joint => {
    const q=feedbackTest.feedback.robot.joints.get(joint).motion.quaternion;
    return 2*Math.acos(Math.min(1,Math.abs(q.w)));
  }, fixtures.joint);
  await mkdir(join(root,'logs/feedback_validation'),{recursive:true});
  for (const model of ['TsenseI_v3_2_withid']) {
    await page.locator('#hand-model').selectOption(model);
    await page.waitForFunction(() => !document.querySelector('#start').disabled);
    await page.evaluate(state => {
      window.sdkFeedbackState = state;
      window.feedSdk = () => feedbackTest.sdk.update({...window.sdkFeedbackState});
      feedSdk(); feedbackTest.refresh(); window.feedbackFeedTimer=setInterval(()=>{feedSdk();feedbackTest.refresh();},100);
    }, fixtures.sdkStates[0].state);
    for (const {angle, state} of fixtures.sdkStates) {
      await page.evaluate(state => {window.sdkFeedbackState=state;feedSdk();feedbackTest.refresh();}, state);
      await page.waitForFunction(({names, angle}) => {
        if (!document.querySelector('#feedback-dot').classList.contains('online')) return false;
        return names.every(name => {
          const joint=feedbackTest.feedback.robot.joints.get(name);
          const q=joint.motion.quaternion, axis=joint.vector;
          const measured=2*Math.atan2(q.x*axis.x+q.y*axis.y+q.z*axis.z,q.w);
          return Math.abs(measured-angle)<1e-4;
        });
      }, {names:fixtures.swingJoints, angle});
    }
    assert(Math.abs(await readAngle()-.55)<1e-4, 'SDK positions must independently drive the feedback URDF');
    await page.evaluate(joint => feedbackTest.viewer.setPose({[joint]: 1.1}), fixtures.joint);
    assert(Math.abs(await readAngle()-.55)<1e-4, 'target changes must not drive feedback');
    await page.evaluate(() => clearInterval(window.feedbackFeedTimer));
    await page.waitForFunction(() => document.querySelector('#feedback-status').textContent.includes('过期'));
    assert(Math.abs(await readAngle()-.55)<1e-4, 'stale feedback holds its last measured pose');
    await page.evaluate(() => {
      feedbackTest.sdk.update({connected:false});
      feedbackTest.refresh();
    });
    await page.waitForFunction(() => document.querySelector('#feedback-status').textContent.includes('未连接'));
    assert(Math.abs(await readAngle()-.55)<1e-4, 'disconnect holds the last measured pose');
  }
  await page.locator('#side').selectOption('right');
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  assert.match(await page.locator('#feedback-caption').innerText(), /RIGHT/);
  await page.locator('#input-source').selectOption('wuji_glove');
  assert.equal(await page.locator('#camera-title').innerText(), '原始关键点输入');
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(() => document.documentElement.scrollWidth<=innerWidth));
  await page.locator('.workspace').screenshot({path:join(root,'logs/feedback_validation/four-panels-mobile.png')});
  assert.deepEqual(errors, []);
  console.log('PASS: four panels, A3.2 with ID SDK URDF feedback conversion, signed swing, independent target/measurement, stale/disconnect, model/side switch, mobile');
} finally {
  await browser?.close();
  server.kill();
  await new Promise(resolve => {if(server.exitCode!==null)resolve();else server.once('exit',resolve);});
  await rm(logs,{recursive:true,force:true});
}
