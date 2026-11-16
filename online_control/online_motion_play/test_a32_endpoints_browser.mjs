// Run against an isolated play service. Real HTML range controls must reach
// irrational radian endpoints, and rounded number fields must commit them.
import {chromium, expect} from '../online_motion_track/node_modules/@playwright/test/index.mjs';
import assert from 'node:assert/strict';

const base = process.env.MOTION_URL || 'http://127.0.0.1:8873';
const browser = await chromium.launch({headless:true,
  ...(process.env.CHROME_PATH ? {executablePath:process.env.CHROME_PATH} : {}),
  args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page = await browser.newPage();
page.setDefaultTimeout(30000);
const errors = [];
page.on('pageerror', error => errors.push(error.message));
const snapshot = async () => {
  const response = await page.request.get(base + '/api/state', {headers:{'X-Motion-Client':'cad-endpoint-readonly'}});
  assert(response.ok());
  return response.json();
};
try {
  await page.addInitScript(() => localStorage.setItem('motionPlayJointDisplay', 'rad'));
  await page.goto(base);
  await expect(page.locator('#control')).toHaveText('释放控制');
  for (const [side, selection] of [['left','0'], ['right','1']]) {
    if (await page.locator('#handSide select').inputValue() !== selection) {
      await page.locator('#handSide select').selectOption(selection);
      await expect(page.locator('#fileNote')).toContainText(`_${side}.json`);
    }
    await expect.poll(async () => (await snapshot()).side).toBe(side);
    await expect(page.locator('#joints input[type=range]')).toHaveCount(20);
    const state = await snapshot();
    assert.equal(state.connected, false);
    for (const suffix of ['thumb_rota_block_joint','thumb_rotaback_joint2','thumb_MCP_joint','thumb_PIP_joint','index_swing_joint']) {
      const name = `${side}_hand_${suffix}`;
      const joint = state.joints.find(j => j.name === name);
      const part = joint.short.split('_').slice(1).join('_');
      const slider = page.locator('#finger-' + joint.short.split('_')[0]).getByRole('slider', {name:new RegExp(' ' + part + '$')});
      for (const [key, endpoint] of [['End', joint.max], ['Home', joint.min]]) {
        console.log('ENDPOINT', side, name, key, endpoint);
        await slider.press(key);
        assert(Math.abs(Number(await slider.inputValue()) - endpoint) < 1e-12);
        await expect.poll(async () => (await snapshot()).pose[name], {timeout:10000}).toBeCloseTo(endpoint, 12);
      }
      const number = slider.locator('..').locator('input[type=number]');
      await number.fill(String(Number(joint.max.toFixed(6))));
      await number.press('Enter');
      await expect.poll(async () => Math.abs((await snapshot()).pose[name] - joint.max) < 1e-12).toBe(true);
    }
    assert.equal((await snapshot()).connected, false);
  }
  assert.deepEqual(errors, []);
  console.log('CAD ENDPOINT BROWSER PASSED: both sides, slider Home/End and rounded numeric endpoints');
} finally {
  await browser.close();
}
