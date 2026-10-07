import test from "node:test";
import assert from "node:assert/strict";
import {execFileSync} from "node:child_process";
import {readFileSync} from "node:fs";
import {fileURLToPath} from "node:url";
import {frame,hex} from "../static/serial-protocol.js";
import * as a3 from "../static/a30-serial-protocol.js";
import {A30BrowserHandControl} from "../static/a30-browser-hand-control.js";
const root=fileURLToPath(new URL("../../../",import.meta.url));

test("all 18 self slots agree with the supplied Python struct, including every MCP/PIP",()=>{
  const p=payload(),v=new DataView(p.buffer);
  for(let i=0;i<18;i++){
    const o=12+i*24;p.set([0,0,0,2],o);
    for(const [off,val] of [[4,-i-.25],[8,i+.5],[12,-i-.75],[16,-i-1.25]])v.setFloat32(o+off,val,true);
    v.setInt16(o+20,-1500+i,true);v.setInt16(o+22,i===0?0x7fff:2300+i,true);
  }
  const expected=JSON.parse(execFileSync(process.env.HAND_PYTHON||"python",["-X","utf8","-c",`import ast,struct,json
from pathlib import Path
source=ast.parse(Path('tests/fixtures/a30_protocol_reference.py').read_text(encoding='utf-8-sig'))
scope={'struct':struct}
for node in source.body:
 if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SELF_TELEM_SLOT' for t in node.targets):
  exec(compile(ast.Module(body=[node],type_ignores=[]),'source','exec'),scope)
b=bytes.fromhex('${hex(p)}')
print(json.dumps([scope['SELF_TELEM_SLOT'].unpack_from(b,12+i*24) for i in range(18)]))`],{cwd:root,encoding:"utf8"}));
  const decoded=a3.telemetry(p);
  decoded.self.forEach((s,i)=>{
    const ref=expected[i];assert.equal(s.home_state,2);
    assert.deepEqual([s.position,s.iq,s.iq_ref,s.pos_ref],ref.slice(1,5));
    assert.equal(s.grav_iq,ref[5]/1000);assert.equal(s.mcu_temp_c,i===0?null:ref[6]/100);
    const group=a3.ROTARY.has(i)?decoded.rotary:decoded.linear;
    const mapped=group.find(x=>x.joint_suffix===a3.SELF[i]+'_joint');assert(mapped);
    assert.equal(mapped[a3.ROTARY.has(i)?'pos_actual_mm':'pos_act_mm'],ref[1]*(a3.ROTARY.has(i)?1:-1));
  });
  assert.equal(decoded.decode_issues.length,0);
  assert.equal(a3.telemetry(p.slice(0,444)).rotary[9].pos_actual_mm,null);
  v.setFloat32(12+5*24+16,NaN,true);
  assert(a3.telemetry(p).decode_issues.some(x=>x.joint_suffix==='pinky_MCP_joint'&&x.field==='pos_ref'));
});

test("offline viewer embeds exactly the shared telemetry decoder",()=>{
  const source=readFileSync(new URL('../../shared/a30-telemetry.js',import.meta.url),'utf8').replaceAll('export ','');
  const viewer=readFileSync(new URL('../../../tools/hand_control_log_viewer.html',import.meta.url),'utf8');
  assert(viewer.replaceAll('\r\n','\n').includes(source.replaceAll('\r\n','\n')));
});

test("recorded wire evidence reconstructs the exact CRC-checked uplink including extensions",()=>{
  const extended=new Uint8Array(490);extended.set(payload());extended.set([1,2,3,4,5,6],484);
  const wire=frame(2,extended),parser=new a3.UplinkParser();
  assert.equal(parser.push(wire.slice(0,45)).length,0);
  const [p]=parser.push(wire.slice(45)),info=p.wire_info;
  assert(info.crc_verified);assert.equal(info.payload_bytes,490);
  const crc=Buffer.alloc(2);crc.writeUInt16LE(info.crc16);
  assert.equal(info.header_hex+hex(p)+info.extension_hex+crc.toString('hex'),hex(wire));
});

test("gesture and motion playback import the same protocol implementation", async()=>{
  const playback=await import("../../online_motion_play/a30-serial-protocol.js");
  assert.equal(a3.positions,playback.positions);
  assert.equal(a3.MIT_GEAR_RATIOS,playback.MIT_GEAR_RATIOS);
});

test("all 20 joints match motion-playback bytes and ranges for both hands",()=>{
  const cases=JSON.parse(execFileSync(process.env.HAND_PYTHON||"python",["-B","-X","utf8","-c",`import sys,json
from dataclasses import replace
sys.path[:0]=['.','src']
from a30_kinematics import A30StrokeMapper
from online_control.online_motion_play.downlink_policy import encode_online_targets,a30_ranges
cases=[]
for side in ('left','right'):
 m=A30StrokeMapper(side); ranges=a30_ranges(m)
 base={n:lo for n,(lo,hi) in zip(m.joint_names,m._limits)}
 for name,(lo,hi) in zip(m.joint_names,m._limits):
  for fraction in (0,.25,.5,.75,1):
   t=m.convert({**base,name:lo+(hi-lo)*fraction})
   for sign in (1,-1):
    t=replace(t,linear_strokes_mm={n:sign*abs(v) for n,v in t.linear_strokes_mm.items()})
    cases.append(dict(side=side,linear=t.linear_strokes_mm,rotary=t.rotary_positions_deg,ranges=ranges,
                      wire=encode_online_targets(t,hand_side=side,seq=257).hex()))
print(json.dumps(cases))`],{cwd:root,encoding:"utf8",maxBuffer:8*1024*1024}));
  for(const c of cases){
    const bytes=a3.positions(257,c.linear,c.rotary,c.side),v=new DataView(bytes.buffer);
    assert.equal(hex(bytes),c.wire);
    for(const [name,offset] of Object.entries(a3.positionSlots(c.side))){
      const value=v.getFloat32(offset,true),range=c.ranges[name];
      assert(value>=range.min-1e-5&&value<=range.max+1e-5,`${name}: ${value}`);
    }
  }
  assert.equal(cases.length,400);
});

export function payload(seq=1,tick=100){
  const p=new Uint8Array(484),v=new DataView(p.buffer);
  v.setUint16(0,seq,true);v.setUint16(2,3,true);v.setUint32(4,tick,true);v.setUint32(8,0x3ffff,true);
  for(let i=0;i<18;i++)p[12+i*24+3]=2;
  for(let i=0;i<2;i++){const o=444+i*20;p[o]=5+4*i;p[o+1]=5;v.setUint32(o+2,0x80000000,true);v.setUint16(o+6,0x8000,true);v.setUint16(o+8,0x8000,true);p[o+16]=4;}
  return p;
}
function targets(side="left"){
  const linear={},rotary={};
  a3.SELF.forEach((n,i)=>(a3.ROTARY.has(i)?rotary:linear)[`${side}_hand_${n}_joint`]=i*.1);
  a3.EXTERNAL.forEach((n,i)=>rotary[`${side}_hand_${n}`]=(i+1)*2.5);
  return {linear,rotary};
}

for(const side of ["left","right"])test(`A3 ${side} applies 50/30 and linear sign once through feedback hold and fallback`,async()=>{
  const t=targets(side),p=payload(),v=new DataView(p.buffer);
  // Exact binary fractions keep this test independent of float32 rounding.
  a3.SELF.forEach((n,i)=>{
    const name=`${side}_hand_${n}_joint`,value=(i+1)/4;
    (a3.ROTARY.has(i)?t.rotary:t.linear)[name]=value;
    v.setFloat32(16+i*24,a3.ROTARY.has(i)?value:-value,true);
    v.setFloat32(28+i*24,a3.ROTARY.has(i)?value:-value,true);
  });
  a3.EXTERNAL.forEach((n,i)=>{
    t.rotary[`${side}_hand_${n}`]=180/[50,30][i];
    v.setUint32(446+i*20,0x80000000+1048576,true);
  });
  const bytes=a3.positions(7,t.linear,t.rotary,side),d=a3.decodePositions(bytes);
  assert.deepEqual(d.self_pos,Array.from({length:18},(_,i)=>(i+1)/4*(a3.ROTARY.has(i)?1:-1)));
  assert.deepEqual(d.mit_pos_deg,[180,180]);
  assert.deepEqual(d.linear_mm,Object.values(t.linear));
  const telemetry=a3.telemetry(p);
  assert.deepEqual(telemetry.linear.map(s=>s.pos_act_mm),Object.values(t.linear));
  assert.deepEqual(telemetry.linear.map(s=>s.pos_ref_mm),Object.values(t.linear));
  assert.equal(hex(a3.holdTelemetry(7,telemetry,side,t)),hex(bytes));
  const c=new A30BrowserHandControl();c.side=side;c.connected=true;
  c.writer={async write(){}};
  await c.write(bytes);
  assert.deepEqual(c.fallbackTargets,t);
  telemetry.self.forEach(s=>s.online=false);telemetry.external.forEach(s=>s.online=false);
  assert.equal(hex(a3.holdTelemetry(7,telemetry,side,c.fallbackTargets)),hex(bytes));
});

test("external telemetry agrees with the user-supplied A3_USB_Downlink decoder",()=>{
  const p=payload(),v=new DataView(p.buffer);
  for(let i=0;i<2;i++){
    const o=444+i*20;
    v.setUint32(o+2,0x80000000+(i? -1048576:1048576),true);
    v.setUint16(o+6,0x8000-123,true);v.setUint16(o+8,0x8000+512,true);
    v.setUint16(o+10,243,true);v.setUint32(o+12,i?5:0,true);
    p[o+17]=2;p[o+18]=2;p[o+19]=4;
  }
  const expected=JSON.parse(execFileSync(process.env.HAND_PYTHON||"python",["-X","utf8","-c",`import ast,json,struct,typing
from pathlib import Path
source=ast.parse(Path('tests/fixtures/a30_protocol_reference.py').read_text(encoding='utf-8-sig'))
scope={'Optional':typing.Optional,'MIT_STATUS_BYTES':20,'MIT_POS_COUNTS':2097152,'MIT_FAULT_BITS':[(0,'驱动'),(1,'过压'),(2,'欠压'),(3,'过流'),(4,'编码器')]}
for node in source.body:
 if isinstance(node,ast.FunctionDef) and node.name in ('mit_ident_ok','parse_mit_status20'):
  exec(compile(ast.Module(body=[node],type_ignores=[]),'source','exec'),scope)
b=bytes.fromhex('${hex(p)}')
print(json.dumps([dict(scope['parse_mit_status20'](b[444+i*20:464+i*20]),ident_ok=scope['mit_ident_ok'](b[444+i*20:464+i*20],i)) for i in range(2)]))`],{cwd:root,encoding:"utf8"}));
  a3.telemetry(p).external.forEach((s,i)=>{
    const ref=expected[i];assert.equal(s.ident_ok,true);assert.equal(s.ident_ok,ref.ident_ok);
    assert.equal(s.position,ref.pos_deg/[50,30][i]);assert.equal(s.velocity,ref.vel_rpm/[50,30][i]);assert.equal(s.iq,ref.iq_a);
    for(const key of ["ident0","ident1","vbus_v","fault","faults","state","calib","mode","mtype"])assert.deepEqual(s[key],ref[key]);
  });
});

test("self slot union distinguishes homing, theta and open-spin frequency from temperature",()=>{
  const p=payload(),v=new DataView(p.buffer);
  v.setFloat32(16,12.5,true);v.setFloat32(20,-3.25,true);v.setFloat32(24,2,true);v.setFloat32(28,13,true);
  v.setInt16(32,-1250,true);v.setInt16(34,-2300,true);
  p[36]=2;v.setUint16(37,450,true);v.setFloat32(56,15.25,true);
  v.setFloat32(60,1.25,true);v.setInt16(82,0x7fff,true);
  const [home,spin,theta]=a3.telemetry(p).self;
  assert.equal(home.home_state,2);assert.equal(home.position,12.5);assert.equal(home.iq,-3.25);
  assert.equal(home.iq_ref,2);assert.equal(home.pos_ref,13);assert.equal(home.grav_iq,-1.25);assert.equal(home.mcu_temp_c,-23);
  assert.equal(spin.home_state,null);assert.equal(spin.open_spin_state,2);assert.equal(spin.open_spin_remaining_ms,450);
  assert.equal(spin.open_spin_frequency_hz,15.25);assert.equal(spin.grav_iq,null);assert.equal(spin.mcu_temp_c,null);
  assert.equal(theta.theta_rad,1.25);assert.equal(theta.mcu_temp_c,null);
});

test("board and extended hand frames follow source prefix lengths and separate freshness",async()=>{
  const b=new Uint8Array(12),v=new DataView(b.buffer);v.setUint16(0,12,true);v.setInt16(2,-1234,true);
  v.setInt16(4,0x7fff,true);v.setUint16(6,24123,true);
  assert.deepEqual(a3.boardTelemetry(b),{telemetry_seq:12,ntc_c:-12.34,ntc_b:null,vin_v:24.123});
  v.setUint16(6,0xffff,true);assert.equal(a3.boardTelemetry(b).vin_v,null);
  const extended=new Uint8Array(500);extended.set(payload());
  const parser=new a3.UplinkParser();
  assert.equal(parser.push(frame(2,extended))[0].length,484);
  assert.equal(parser.push(frame(2,extended.slice(0,460)))[0].length,444);
  const events=[],c=new A30BrowserHandControl({now:()=>2000,recorder:{record:(type,data)=>events.push({type,...data})}});
  c.connected=true;c.side="left";c.parser=parser;c.latest=a3.telemetry(payload());c.latestPayload="original";c.telemetryAt=1000;
  let sent=false;c.reader={async read(){if(!sent){sent=true;return {done:false,value:frame(3,b)};}c.connected=false;return {done:true};},releaseLock(){}};
  await c.readLoop();
  assert.equal(c.boardTelemetry.ntc_c,-12.34);assert.equal(c.telemetryAt,1000);assert.equal(c.latestPayload,"original");
  assert.equal(c.acceptedPackets,0);assert(!c.freshTelemetry());assert.equal(events[0].type,"board_telemetry");
});

test("online mask authorizes external status, while identifier-only cached status stays stale",()=>{
  const p=payload();p[444]=0;
  const t=a3.telemetry(p);assert(!t.external[0].ident_ok);assert(a3.externalFeedbackValid(t.external[0]));
  p[456]=1;assert(!a3.externalFeedbackValid(a3.telemetry(p).external[0]));
  p[444]=5;p[456]=0;p[2]=0;
  const stale=a3.telemetry(p).external[0];assert(stale.valid);assert(!a3.externalFeedbackValid(stale));
});
test("A3 browser frames exactly match the extracted Python encoder",()=>{
  const python=execFileSync(process.env.HAND_PYTHON||"python",["-B","-X","utf8","-c",`import sys,json
sys.path[:0]=['.','src']
from a30_kinematics.downlink import channel_mapping,pack_self_homing,pack_self_pwr
from a30_kinematics.policy import encode_control_targets as encode_targets
from online_control.online_motion_track.pipeline_v3_0.actuation import ActuatorTargets
rows=channel_mapping('left'); l={};r={}
for i,row in enumerate(rows):
 (l if row['unit']=='mm' else r)[row['joint']]=i*.1 if i<18 else (i-17)*2.5
print(json.dumps([encode_targets(ActuatorTargets(linear_strokes_mm=l,rotary_positions_deg=r),seq=5).hex(),pack_self_homing(2,2,1,700).hex(),pack_self_pwr(1,4,2).hex()]))`],{cwd:root,encoding:"utf8"});
  const [batch,home,power]=JSON.parse(python),t=targets();
  assert.equal(hex(a3.positions(5,t.linear,t.rotary,"left")),batch);
  assert.equal(hex(a3.home(2,2,1,700)),home);assert.equal(hex(a3.power(1,4,2)),power);
});
test("A3 telemetry accepts AA55 only, handles fragments/CRC and validates all axes",()=>{
  const p=payload(),b=frame(2,p),parser=new a3.UplinkParser();
  assert.deepEqual(parser.push(b.slice(0,200)),[]);assert.equal(parser.push(b.slice(200)).length,1);
  const bad=b.slice();bad[24]^=1;assert.equal(parser.push(bad).length,0);
  assert.equal(parser.push(b).length,1);assert(a3.telemetry(p).all_online);
  p[456]=1;assert(!a3.telemetry(p).all_online);
  const old=b.slice();old[0]=0x55;old[1]=0xaa;assert.equal(new a3.UplinkParser().push(old).length,0);
});

test("444-byte A3 self-only firmware is decoded without requiring external telemetry",()=>{
  const p=payload().slice(0,444),wire=frame(2,p),parser=new a3.UplinkParser();
  const decoded=parser.push(wire);
  assert.equal(decoded.length,1);assert.equal(decoded[0].length,444);
  const t=a3.telemetry(decoded[0]);
  assert.equal(t.payload_bytes,444);assert.equal(t.self.length,18);
  assert(t.external.every(s=>!s.online&&!s.ident_ok));
  assert.equal(a3.missingFeedback(t,"left").length,2);
  const c=new A30BrowserHandControl({now:()=>1000});c.latest=t;c.telemetryAt=1000;c.side="left";
  assert(c.freshTelemetry());assert.equal(c.telemetryIssue(),"");
});

test("telemetry resumes after large sequence gaps and board reboot; duplicate packets do not refresh it",async()=>{
  const events=[],c=new A30BrowserHandControl({now:()=>1000,recorder:{record:(type,data)=>events.push({type,...data})}});
  c.connected=true;c.side="left";c.parser=new a3.UplinkParser();c.lastTelemetrySeq=-1;c.lastTelemetryTick=null;
  const packets=[payload(1,100),payload(700,1000),payload(0,1),payload(0,1)];
  c.reader={async read(){if(packets.length)return {value:frame(2,packets.shift()),done:false};c.connected=false;return {done:true};},releaseLock(){}};
  await c.readLoop();
  assert.equal(c.parsedPackets,4);assert.equal(c.acceptedPackets,3);assert.equal(c.rxBytes,4*492);
  assert.deepEqual(events.filter(e=>e.type==="telemetry").map(e=>e.accepted),[true,true,true,false]);
});
function mockController(failedSlot=-1){
  let now=1000;const writes=[],events=[],c=new A30BrowserHandControl({now:()=>now,recorder:{record:(type,data)=>events.push({type,...data})},wait:async ms=>{
    now+=ms;c.telemetryAt=now;c.lastTelemetrySeq++;
  }});
  c.connected=true;c.side="left";c.latest=a3.telemetry(payload());c.latestPayload=hex(payload());c.telemetryAt=now;c.lastTelemetrySeq=1;
  c.feedbackPositionLimits=Object.fromEntries(Object.entries(targets()).map(([kind,values])=>
    [kind,Object.fromEntries(Object.keys(values).map(n=>[n,[-100,100]]))]));
  c.write=async bytes=>{
    writes.push(bytes);
    if(bytes[5]===0x22){const bus=bytes[6],node=bytes[7],slot=bus===0?node-1:bus===1?node+7:node+15;
      c.latest.self[slot].home_state=bytes[8]===2?0:slot===failedSlot?3:2;}
  };
  return {c,writes,events,advance:ms=>now+=ms};
}
test("18 self motors home with A3 commands; unsupported external axes are reported, partial homing permits rebase",async()=>{
  const {c,writes}=mockController(4),pose=Object.fromEntries(a3.SELF.map(n=>[`left_hand_${n}_joint`,0]));
  a3.EXTERNAL.forEach(n=>pose[`left_hand_${n}`]=0);
  const limits=Object.fromEntries(Object.keys(pose).map(n=>[n,[-2,2]]));
  const report=await c.home(700,pose,limits);
  assert.equal(report.total,20);assert.equal(report.failures.length,3);assert(c.homed&&c.requiresTelemetrySeed);
  assert.equal(writes.filter(b=>b[5]===0x22&&b[8]===1).length,18);
  assert(writes.every(b=>[0x20,0x21,0x22].includes(b[5])));
  c.handAt=c.now();
  await c.arm(async (held,telem)=>{
    assert.equal(telem,c.latestPayload);
    return {collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,browser_epoch:2,
      linear_strokes_mm:targets().linear,rotary_positions_deg:targets().rotary,
      ruckig_joint_angles_rad:pose};
  });
  assert(c.armed);await c.emergency();assert(!c.armed&&!c.homed);
  assert.equal(writes.filter(b=>b[5]===0x21&&b[8]===2).length,18);
});
test("A3 arms without a hand or fresh telemetry, holds through receive gaps and resumes",async()=>{
  const {c,writes,advance}=mockController();const t=targets();
  const pose=Object.fromEntries([...Object.keys(t.linear),...Object.keys(t.rotary)].map(n=>[n,0]));
  c.held=pose;c.limits=Object.fromEntries(Object.keys(pose).map(n=>[n,[-2,2]]));
  advance(1000);assert(!c.freshTelemetry());assert.equal(c.controlTelemetryIssue(),"");
  await c.arm(async()=>({collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,browser_epoch:1,ruckig_joint_angles_rad:pose,
    linear_strokes_mm:t.linear,rotary_positions_deg:t.rotary}));
  c.lastSentAt=c.now()-25;
  const state={browser_epoch:1,hand_side:"left",collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,
    target_ready:true,tracking_transition:{phase:"holding"},sample_ticks:1,ruckig_joint_angles_rad:pose,
    linear_strokes_mm:t.linear,rotary_positions_deg:t.rotary};
  await c.sample(state,c.now());assert.equal(c.sent,1);assert.equal(writes.at(-1).length,108);
  c.latest.all_online=false;c.latest.self[0].online=false;c.watchdog();assert(c.armed);
  for(let i=0;i<40;i++){
    advance(25);c.observeHand(false,c.now());
    await c.sample({...state,sample_ticks:i+2},c.now());assert(c.armed);
  }
  assert.equal(c.sent,41);assert(!c.freshTelemetry());
  advance(25);c.observeHand(true,c.now());
  await c.sample({...state,tracking_transition:{phase:"tracking"},sample_ticks:42},c.now());
  assert(c.armed);assert.equal(c.sent,42);
  advance(151);c.watchdog();assert(!c.armed); // Actual compute/USB stalls still stop.
  const count=writes.length;await c.sample({...state,sample_ticks:43},c.now());assert.equal(writes.length,count);
});

test("A3 receive-gap policy keeps first-feedback, external fault and side checks",()=>{
  const {c,advance}=mockController();
  advance(1000);assert.equal(c.controlTelemetryIssue(),"");
  c.latest.external[0].online=true;c.latest.external[0].fault=1;
  assert.match(c.controlTelemetryIssue(),/故障/);
  c.armed=true;c.watchdog();assert(!c.armed);
  c.latest.external[0].fault=0;c.latest.hand_side="right";
  assert.match(c.controlTelemetryIssue(),/左右手/);
  c.latest=null;assert.match(c.controlTelemetryIssue(),/首次开启/);
});

test("partial hold uses fallback only for unavailable axes, never zero-filled external bytes",()=>{
  const t=targets(),p=payload(),v=new DataView(p.buffer);
  v.setUint32(8,0x3fffe,true);v.setUint16(2,0,true);p.fill(0,444);
  v.setFloat32(16,NaN,true);v.setFloat32(40,12.5,true);
  const b=a3.holdTelemetry(1,a3.telemetry(p),"left",t),d=a3.decodePositions(b);
  assert.equal(d.self_pos[0],t.rotary.left_hand_pinky_DIP_joint);
  assert.equal(d.self_pos[1],12.5);
  assert.deepEqual(d.mit_pos_deg,[125,150]);
  assert.equal(a3.missingFeedback(a3.telemetry(p),"left").length,3);
});

test("A3 missing-all feedback still permits arm with supplied fallback and cannot mark homing successful",async()=>{
  const {c}=mockController(),t=targets(),p=payload(),v=new DataView(p.buffer);
  v.setUint32(8,0,true);v.setUint16(2,0,true);p.fill(0,444);
  c.latest=a3.telemetry(p);c.latestPayload=hex(p);c.fallbackTargets=t;
  const pose=Object.fromEntries([...Object.keys(t.linear),...Object.keys(t.rotary)].map(n=>[n,0]));
  c.limits=Object.fromEntries(Object.keys(pose).map(n=>[n,[-2,2]]));c.handAt=c.now();
  await c.arm(async()=>({collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,browser_epoch:1,
    ruckig_joint_angles_rad:pose,linear_strokes_mm:t.linear,rotary_positions_deg:t.rotary}));
  assert(c.armed);
  const report=await c.home(700,pose,c.limits);
  assert.equal(report.failures.length,20);assert.equal(report.results.filter(r=>r.skipped).length,18);
});
test("invalid or wrong-model targets never produce a frame",()=>{
  const t=targets();t.rotary.left_hand_thumb_PIP_joint=NaN;
  assert.throws(()=>a3.positions(0,t.linear,t.rotary,"left"));
  assert.throws(()=>a3.positions(0,Array(14).fill(0),Array(7).fill(0),"left"));
});

test("homing hold bounds every actuator while retaining unmodified raw telemetry",()=>{
  const t=targets(),p=payload(),v=new DataView(p.buffer);
  for(let i=0;i<18;i++)v.setFloat32(16+i*24,(i%2?1000:-.16)*(a3.ROTARY.has(i)?1:-1),true);
  v.setUint32(446,0,true);v.setUint32(466,0xffffffff,true);
  const limits=Object.fromEntries(Object.entries(t).map(([kind,values])=>
    [kind,Object.fromEntries(Object.keys(values).map(n=>[n,[0,10]]))]));
  const raw=a3.telemetry(p),d=a3.decodePositions(a3.holdTelemetry(1,raw,"left",t,limits));
  assert.deepEqual(d.self_pos,Array.from({length:18},(_,i)=>i%2?10*(a3.ROTARY.has(i)?1:-1):0));
  assert.deepEqual(d.mit_pos_deg,[0,300]);assert(raw.self[0].position<0);
});

test("arming first hold uses the same clipped pose as rebase, even if telemetry changes meanwhile",async()=>{
  const {c,writes,events}=mockController(),t=targets(),joint="left_hand_pinky_DIP_joint";
  t.rotary[joint]=0;c.latest.self[0].position=-.16;
  const pose=Object.fromEntries([...Object.keys(t.linear),...Object.keys(t.rotary)].map(n=>[n,0]));
  c.limits=Object.fromEntries(Object.keys(pose).map(n=>[n,[-2,2]]));c.handAt=c.now();
  const clipped=[{joint,raw:-.16,applied:0,unit:"deg",lower:0,upper:90}];
  await c.arm(async()=>{
    c.latest.self[0].position=25;
    return {collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,browser_epoch:1,
      ruckig_joint_angles_rad:pose,linear_strokes_mm:t.linear,rotary_positions_deg:t.rotary,feedback_clipped_joints:clipped};
  });
  assert(c.armed);assert.equal(a3.decodePositions(writes.find(b=>b[5]===0x20)).self_pos[0],0);
  assert.deepEqual(events.find(e=>e.type==="feedback_seed").clipped_joints,clipped);
  assert.equal(c.latest.self[0].position,25);assert.equal(c.rebasedTargets,null);
});
