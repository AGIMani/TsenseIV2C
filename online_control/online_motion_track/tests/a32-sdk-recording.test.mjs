import test from 'node:test';
import assert from 'node:assert/strict';
import {A32SdkPanel} from '../static/a32-sdk-control.js';
function panel(){
  const rows=[],current={session:'fixture',hardware:{}};
  const p=Object.assign(Object.create(A32SdkPanel.prototype),{enabled:true,serial:1,side:'left',state:{connected:true},lastTick:0,
    currentSession:()=>current,ensureSession:async()=>current,record:(type,data)=>rows.push({type,...data}),
    notice(){},render(){},update(){},api:async()=>{throw Error('synthetic I/O failure');}});
  return {p,rows,current};
}
test('failed SDK actions remain in the downloadable control log',async()=>{
  const {p,rows}=panel();await p.command('emergency_stop');
  assert.equal(rows[0].type,'a32_sdk_error');assert.equal(rows[0].action,'emergency_stop');
  assert.equal(rows[0].error,'synthetic I/O failure');assert.equal(p.busy,false);
});
test('failed SDK polling records its source action and does not schedule another poll',async()=>{
  const {p,rows}=panel();const noticed=new Promise(resolve=>{p.notice=resolve;});
  p.begin();await noticed;clearTimeout(p.timer);
  assert.equal(rows[0].type,'a32_sdk_error');assert.equal(rows[0].action,'status');assert.equal(p.state.armed,false);
});
test('SDK polling preserves ampere metadata and values in browser downloads',async()=>{
  const {p,rows}=panel();
  const hardware={connected:true,telemetry_format:'a32-native-sdk-state/v2',
    telemetry_units:{self_current:'A',external_current:'A'},self:[{iq:-.1374,iq_ref:.2748,current_unit:'A'}]};
  p.api=async()=>hardware;
  const captured=new Promise(resolve=>{p.record=(type,data)=>{rows.push({type,...data});p.serial++;resolve();};});
  p.begin();await captured;clearTimeout(p.timer);
  assert.equal(rows[0].type,'a32_sdk_sample');
  assert.equal(rows[0].telemetry_format,hardware.telemetry_format);
  assert.deepEqual(rows[0].telemetry_units,hardware.telemetry_units);
  assert.equal(rows[0].hardware.self[0].iq,-.1374);
  assert.equal(rows[0].hardware.self[0].iq_ref,.2748);
});
test('stop/disconnect success and failure are recorded before the session is sealed',async()=>{
  const {p,rows,current}=panel();await assert.rejects(p.finish(current),/synthetic I\/O failure/);
  assert.equal(rows[0].action,'disconnect');assert.equal(rows[0].type,'a32_sdk_error');
  p.api=async()=>({connected:false});await p.finish(current);
  assert.equal(rows[1].type,'a32_sdk_action');assert.equal(rows[1].hardware.connected,false);
});
test('direct SDK samples retain successful commands without labeling them Ruckig output',()=>{
  const {p,rows}=panel(),poses=[];p.display=pose=>poses.push(pose);
  p.update=A32SdkPanel.prototype.update;
  p.update({armed:true,direct_mode:true,command_joint_angles_rad:{q:.8},ruckig_joint_angles_rad:{q:.1},
    command_samples:[{tick:1,joints:{q:.8}}]},{display:true});
  assert.equal(poses[0].q,.8);
  assert.equal(rows[0].type,'h7');assert.deepEqual(rows[0].joints_ruckig_ui,{});
  assert.equal(rows[0].ruckig_enabled,false);assert.equal(rows[1].type,'direct_command');
  p.update({armed:false,direct_mode:true,command_joint_angles_rad:{q:0}},{display:true});
  assert.equal(poses.length,1,'unarmed polling must not overwrite live preview');
});
