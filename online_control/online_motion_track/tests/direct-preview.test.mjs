import test from 'node:test';
import assert from 'node:assert/strict';
import {BrowserHandControl} from '../static/browser-hand-control.js';
import {A30BrowserHandControl} from '../static/a30-browser-hand-control.js';
import {armBlockers} from '../static/control-readiness.js';

test('direct mode allows arm, but unconfirmed modes and missing collision still block',()=>{
  const ready={allowed:true,connected:true,active:true,directMode:true,
    hardware:{target_ready:true,direct_mode:true,ruckig_enabled:false},mode:'stop',handAgeMs:0};
  assert.deepEqual(armBlockers(ready),[]);
  assert(armBlockers({...ready,mode:'off'}).some(s=>s.includes('碰撞')));
  assert(armBlockers({...ready,hardware:{...ready.hardware,mode_confirmed:false}}).some(s=>s.includes('尚未确认')));
  assert(armBlockers({...ready,hardware:{...ready.hardware,direct_mode:false}}).some(s=>s.includes('同步')));
});
for(const Controller of [BrowserHandControl,A30BrowserHandControl]) {
  test(`${Controller.name}: mode changes cancel queued writes until confirmed and manually armed`,async()=>{
    const c=new Controller();let writes=0,rebases=0;
    c.connected=true;c.armed=true;c.writer={write:async()=>{writes++;}};
    const queued=c.write(new Uint8Array(108),{stream:true,token:c.job});
    c.setDirectMode(true,false);
    await assert.rejects(queued,/取消/);
    await assert.rejects(c.arm(async()=>{rebases++;}),/尚未确认/);
    assert.equal(writes,0);assert.equal(rebases,0);assert.equal(c.armed,false);
    c.setDirectMode(true);
    assert.equal(c.armed,false,'mode confirmation never automatically arms');
    c.armed=true;
    await c.write(new Uint8Array(108),{stream:true,token:c.job});
    assert.equal(writes,1,'direct stream writes are allowed');
    c.setDirectMode(false);
    assert.equal(c.armed,false);
  });
}
