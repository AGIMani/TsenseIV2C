import test from "node:test";
import assert from "node:assert/strict";
import {armBlockers} from "../static/control-readiness.js";
const ready={allowed:true,connected:true,homing:false,busy:false,active:true,
  hardware:{target_ready:true,ruckig_enabled:true,sample_collision_ready:true},
  telemetryIssue:"",handAgeMs:20,side:"left",mode:"mink"};
test("readiness does not require homing or all motor channels online",()=>{
  assert.deepEqual(armBlockers({...ready,hardware:{...ready.hardware,local_homed:false,all_online:false}}),[]);
});

test("A3 can arm a ready holding target without a detected hand; faults still block",()=>{
  const a30={...ready,requireHandConfirmation:false,handAgeMs:Infinity,
    hardware:{...ready.hardware,tracking_transition:{phase:"holding"}}};
  assert.deepEqual(armBlockers(a30),[]);
  assert(armBlockers({...a30,telemetryIssue:"首次开启请先接收一包有效的 A3 手部遥测"}).length);
  assert(armBlockers({...a30,hardware:{...a30.hardware,target_ready:false,last_error:"碰撞"}}).some(r=>r.includes("碰撞")));
});
test("all actual arm blockers have an actionable message; server errors are preserved",()=>{
  for(const [change,expected]of [
    [{allowed:false},/Chrome/],[{connected:false},/连接/],[{homing:true},/找零/],
    [{busy:true},/操作/],[{active:false},/识别/],[{telemetryIssue:"未收到遥测"},/未收到遥测/],
    [{handAgeMs:401,side:"right"},/右手/],[{mode:"off"},/碰撞停止/],[{mode:"monitor"},/碰撞停止/],
    [{hardware:{...ready.hardware,target_ready:false,last_error:"采样碰撞检查阻断运动"}},/采样碰撞检查阻断运动/],
    [{hardware:{...ready.hardware,ruckig_enabled:false}},/Ruckig/],
    [{hardware:{...ready.hardware,sample_collision_ready:false}},/采样碰撞/],
    [{hardware:{...ready.hardware,power_busy:true}},/操作/]]){
    assert(armBlockers({...ready,...change}).some(r=>expected.test(r)),JSON.stringify(change));
  }
});

test('latched A3 faults permit manual feedback rebase while other prerequisites still block',()=>{
  const fault={...ready,requireHandConfirmation:false,hardware:{...ready.hardware,
    model_id:'TsenseI_v3_0',target_ready:false,rebase_required:true,last_error:'控制采样超时'}};
  assert.deepEqual(armBlockers(fault),[]);
  assert(armBlockers({...fault,telemetryIssue:'外接电机故障'}).some(r=>r.includes('外接电机故障')));
  assert(armBlockers({...fault,connected:false}).some(r=>r.includes('连接')));
  assert.deepEqual(armBlockers({...fault,mode:'off'}),[]);
  assert(armBlockers({...fault,hardware:{...fault.hardware,model_id:'TsenseI_v2'}}).some(r=>r.includes('未就绪')));
});

for (const model_id of ['TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v3_2', 'TsenseI_v3_2_withid']) {
  for (const mode of ['off', 'monitor']) {
    test(`${model_id} permits ${mode} with motion limits but no sample collision guard`, () => {
      const state = {...ready, mode, hardware: {...ready.hardware, model_id, sample_collision_ready: false}};
      assert.deepEqual(armBlockers(state), []);
      assert(armBlockers({...state, connected: false}).some(r => r.includes('连接')));
      assert(armBlockers({...state, telemetryIssue: '外接电机故障'}).some(r => r.includes('故障')));
      assert(armBlockers({...state, hardware: {...state.hardware, ruckig_enabled: false}}).some(r => r.includes('Ruckig')));
      assert(armBlockers({...state, mode: 'stop'}).some(r => r.includes('采样碰撞')));
    });
  }
}
