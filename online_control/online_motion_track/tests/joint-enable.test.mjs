import test from "node:test";
import assert from "node:assert/strict";
import {execFileSync} from "node:child_process";
import {fileURLToPath} from "node:url";
import * as v2 from "../static/serial-protocol.js";
import * as a30 from "../static/a30-serial-protocol.js";
import {BrowserHandControl} from "../static/browser-hand-control.js";
import {A30BrowserHandControl} from "../static/a30-browser-hand-control.js";

function setup(protocol, side="left") {
  const writes=[],events=[],Controller=protocol===v2?BrowserHandControl:A30BrowserHandControl;
  const c=new Controller({now:()=>1000,recorder:{record:(type,data)=>events.push({type,...data})}});
  c.side=side;c.connected=true;c.handAt=1000;c.telemetryAt=1000;c.latestPayload="measured";
  c.latest={all_online:true,linear:Array.from({length:14},()=>({pos_act_mm:2})),rotary:Array.from({length:7},()=>({pos_actual_mm:3})),
    self:Array.from({length:18},()=>({position:2,online:true})),external:[{position:3,online:true,ident_ok:true},{position:4,online:true,ident_ok:true}]};
  c.writer={write:async b=>writes.push(b.slice())};
  c.jointEnabled=Object.fromEntries(Object.keys(protocol.positionSlots(side)).map(n=>[n,true]));
  const pose=Object.fromEntries(Object.keys(c.jointEnabled).map(n=>[n,.01]));
  c.limits=Object.fromEntries(Object.keys(pose).map(n=>[n,[-2,2]]));
  const maps=protocol.slotMaps(side);
  const targets=Object.fromEntries(Object.entries(maps).map(([kind,map])=>[kind,protocol===v2
    ?Object.values(map).map(i=>i+1.25):Object.fromEntries(Object.entries(map).map(([n,i])=>[n,i+1.25]))]));
  const state={browser_epoch:1,hand_side:side,collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,
    target_ready:true,tracking_transition:{phase:"tracking"},ruckig_joint_angles_rad:pose,hardware_joint_angles_rad:pose,
    linear_strokes_mm:targets.linear,rotary_positions_deg:targets.rotary,sample_ticks:1,frame_id:3};
  if(protocol===a30)c.feedbackPositionLimits=Object.fromEntries(Object.entries(targets).map(([kind,values])=>
    [kind,Object.fromEntries(Object.keys(values).map(n=>[n,[-100,100]]))]));
  return {c,writes,events,state,targets};
}
for(const protocol of [v2,a30])for(const side of ["left","right"]){
  const label=`${protocol===v2?"V2":"A3.0"} ${side}`;
  test(`${label}: each power command addresses the right motor; disable never sends a zero position`,async()=>{
    const {c,writes}=setup(protocol,side);
    for(const [joint,ch]of Object.entries(protocol.powerChannels(side))){
      writes.length=0;c.drive=[[1,1],[1,1],[1,0],[1,1]];
      await c.setJointEnabled(joint,false);
      assert.equal(writes.length,1);
      const expected=ch.kind==="self"?a30.power(ch.bus,ch.motor,2):ch.kind==="linear"?v2.linearPower(ch.bus,ch.motor,false)
        :v2.rotaryDrive(ch.bus,...Array.from({length:2},(_,i)=>i===ch.motor||ch.bus===2&&i===1?0:1));
      assert.equal(v2.hex(writes[0]),v2.hex(expected));
      assert.equal(c.jointEnabled[joint],false);
      c.latest=null;c.telemetryAt=0; // Power-on must work even before any feedback.
      writes.length=0;await c.setJointEnabled(joint,true);
      assert.equal(writes.length,1,"enable sends only the power command");
      assert.notEqual(writes[0][5],0x20,"power switching must not inject position targets");
      assert.equal(writes[0][ch.kind==="rotary"?7+ch.motor:8],1);
      assert.equal(c.jointEnabled[joint],true);
    }
  });
  test(`${label}: no-home arm rebases with feedback, selected motors stay off, position bytes are not masked`,async()=>{
    const {c,writes,state,targets,events}=setup(protocol,side),joint=Object.keys(protocol.powerChannels(side))[0];
    c.jointEnabled[joint]=false;
    assert.equal(c.homed,false);assert.equal(c.held,undefined);
    await c.arm(async(p,feedback)=>{assert.equal(feedback,"measured");return state;});
    assert(c.armed);assert.equal(c.homed,false);assert.equal(c.powerSent[joint],false);
    assert(!writes.some(b=>protocol===v2?[0x12,0x25].includes(b[5]):b[5]===0x22),"no autonomous homing");
    writes.length=0;c.lastSentAt=970;
    await c.sample(state,1000);assert.equal(c.sent,1);
    const expected=protocol.positions(writes[0][6],targets.linear,targets.rotary,side);
    assert.equal(v2.hex(writes[0]),v2.hex(expected));
    const conversion=events.find(e=>e.type==="conversion"),decoded=protocol.decodePositions(writes[0]);
    assert.deepEqual(conversion.linear_mm,decoded.linear_mm);
    assert.deepEqual(conversion.rotary_deg,decoded.rotary_deg);
  });
  test(`${label}: toggling power cancels queued motion and re-arm follows model telemetry policy`,async()=>{
    const {c,writes,state}=setup(protocol,side),joint=Object.keys(protocol.powerChannels(side))[0];
    await c.arm(async()=>state);writes.length=0;c.lastSentAt=970;
    let release;c.tail=new Promise(r=>release=r);
    const queued=c.sample(state,1000),power=c.setJointEnabled(joint,false);
    release();await queued;await power;
    assert(!c.armed);assert(!writes.some(b=>b[5]===0x20));
    c.telemetryAt=0;
    if(protocol===v2)await assert.rejects(c.arm(async()=>state),/遥测|手势/);
    else {await c.arm(async()=>state);assert(c.armed);}
  });
}
test("A3 external axes reject unsupported power commands instead of sending zero",async()=>{
  const {c,writes}=setup(a30);
  for(const n of a30.EXTERNAL)await assert.rejects(c.setJointEnabled(`left_hand_${n}`,false),/不支持/);
  assert.equal(writes.length,0);
});
test("USB failure does not report successful power change",async()=>{
  const {c,events}=setup(a30),joint="left_hand_index_DIP_joint";
  c.writer={write:async()=>{throw new Error("disconnected");}};
  await assert.rejects(c.setJointEnabled(joint,false),/disconnected/);
  assert.equal(c.jointEnabled[joint],true);assert.equal(c.powerSent[joint],undefined);
  assert(!events.some(e=>e.type==="joint_power"&&e.sent));assert(!c.armed);
});
test("all 18 A3 power addresses match the user-supplied Python encoder byte for byte",()=>{
  const root=fileURLToPath(new URL("../../../",import.meta.url));
  const expected=JSON.parse(execFileSync(process.env.HAND_PYTHON||"python",["-X","utf8","-c",
    `import ast,json,struct
from pathlib import Path
tree=ast.parse(Path('tests/fixtures/a30_protocol_reference.py').read_text(encoding='utf-8-sig'))
names={'pack_self_pwr','pack_aa55','crc16_modbus','_crc16_modbus_table'}
nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
m={'struct':struct,'DEV_TELEM':3,'FUN_SELF_PWR':0x21}
exec(compile(ast.Module(body=nodes,type_ignores=[]),'A3_USB_Downlink.pyw','exec'),m)
m['CRC16_MODBUS_TAB']=m['_crc16_modbus_table']()
print(json.dumps([m['pack_self_pwr'](b,n,c).hex() for b,k in [(0,8),(1,8),(2,2)] for n in range(1,k+1) for c in [1,2]]))`],{cwd:root,encoding:"utf8"}));
  const actual=Object.values(a30.powerChannels("left")).flatMap(({bus,motor})=>[1,2].map(cmd=>v2.hex(a30.power(bus,motor,cmd))));
  assert.deepEqual(actual,expected);
});
