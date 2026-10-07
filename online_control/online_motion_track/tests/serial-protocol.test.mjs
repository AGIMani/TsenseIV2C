import test from "node:test";
import assert from "node:assert/strict";
import {execFileSync} from "node:child_process";
import {fileURLToPath} from "node:url";
import {frame, crc16, hex, positions, handType, linearHome, rotaryDrive, rotaryZero, UplinkParser, telemetry} from "../static/serial-protocol.js";
import {BrowserHandControl} from "../static/browser-hand-control.js";

const fixtures = JSON.parse(execFileSync(process.env.HAND_PYTHON || "python", ["-X", "utf8", "-c", `
import sys,json,struct
sys.path[:0]=['src']
from usb.offline import build_downlink,pack_hand_pos_batch,pack_rm_hand_type_set,pack_lm_mech_homing,pack_rm_drive_sp,pack_rm_zero_sp
from usb.telemetry import parse_hand_telem_v2,parse_hand_telem_legacy
frames=[build_downlink(3,0x20,pack_hand_pos_batch(254,[i*.125-1 for i in range(14)],[i*5.25 for i in range(7)])),build_downlink(3,0x2b,pack_rm_hand_type_set(1)),build_downlink(3,0x12,pack_lm_mech_homing(2,2,1,700)),build_downlink(3,0x22,pack_rm_drive_sp(3,1,0)),build_downlink(3,0x25,pack_rm_zero_sp(1,0,1))]
linear=struct.pack('<BBBBfffifIhBB',2,1,0,0,-0.,12.5,-3.5,2500,4.25,0x123456,2650,4,5)
rotary=struct.pack('<BB2x10f',1,1,*[i+.25 for i in range(10)])
modern=struct.pack('<HHI',65535,513,12345)+linear*14+rotary*7+struct.pack('<HHfff',1,2,3.5,4.5,5.5)
legacy=struct.pack('<HHI',23,0,23456)+rotary*21
def safe(f):
 return {'linear':[vars(x) for x in f['linear']],'rotary':[vars(x) for x in f['rotary']]}
print(json.dumps({'frames':[f.hex() for f in frames],'modern':modern.hex(),'legacy':legacy.hex(),'parsed':safe(parse_hand_telem_v2(modern)),'old':safe(parse_hand_telem_legacy(legacy))}))
`], {cwd: fileURLToPath(new URL("../../../", import.meta.url)), encoding: "utf8"}));
const bytes = s => new Uint8Array(Buffer.from(s,"hex"));
test("all outgoing commands exactly match Python H7 encoders", () => {
  const actual = [positions(254,Array.from({length:14},(_,i)=>i*.125-1),Array.from({length:7},(_,i)=>i*5.25)),
    handType("right"),linearHome(2,2,1,700),rotaryDrive(3,1,0),rotaryZero(1,1)];
  assert.deepEqual(actual.map(hex), fixtures.frames);
  assert.throws(()=>positions(0,[NaN],[]));
});
test("modern and legacy telemetry agree with Python field decoders", () => {
  for (const [name, expected] of [["modern",fixtures.parsed],["legacy",fixtures.old]]) {
    const actual = telemetry(bytes(fixtures[name]));
    for (const kind of ["linear","rotary"]) for (let i=0;i<actual[kind].length;i++)
      for (const [key,value] of Object.entries(actual[kind][i])) {
        if (key in expected[kind][i]) assert.equal(value,expected[kind][i][key],`${name}/${kind}/${key}`);
      }
  }
  assert.equal(telemetry(bytes(fixtures.modern)).hand_side,"right");
});
test("CRC, split packets, noise, duplicate byte stream and oversize headers", () => {
  const p=bytes(fixtures.modern), uplink=frame(2,p); uplink.set([0x55,0xaa]);
  new DataView(uplink.buffer).setUint16(uplink.length-2,crc16(uplink.subarray(0,-2)),true);
  const bad=uplink.slice(); bad[11]^=1;
  const parser=new UplinkParser();
  assert.equal(parser.push(new Uint8Array([1,2,0x55,0xaa,0xff,0xff])).length,0);
  assert.equal(parser.push(bad).length,0);
  assert.equal(parser.push(uplink.slice(0,70)).length,0);
  assert.deepEqual(parser.push(uplink.slice(70)),[p]);
});
function control() {
  let now=1000; const log=[];
  const c=new BrowserHandControl({now:()=>now,recorder:{record:(type,data)=>{log.push({type,...data});return log.length;}}});
  c.side="left";c.connected=true;c.homed=true;c.held={left_hand_index_MCP_joint:0};
  c.limits={left_hand_index_MCP_joint:[0,1.57]};c.latest={hand_side:"left",linear:Array.from({length:14},()=>({pos_act_mm:2})),rotary:Array.from({length:7},()=>({pos_actual_mm:3}))};c.latestPayload="measured";c.telemetryAt=now;c.handAt=now;
  c.writer={write:async b=>log.push({wire:hex(b)})};
  const state={browser_epoch:1,hand_side:"left",collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,
    target_ready:true,tracking_transition:{phase:"tracking"},ruckig_joint_angles_rad:{left_hand_index_MCP_joint:.01},
    hardware_joint_angles_rad:{left_hand_index_MCP_joint:.01},linear_strokes_mm:Array(14).fill(0),rotary_positions_deg:Array(7).fill(0),sample_ticks:1,frame_id:3};
  return {c,state,log,advance:ms=>{now+=ms;},now:()=>now};
}
test("arm uses feedback; stale replies and jumps never send",async()=>{
  const {c,state,log,advance,now}=control();
  await c.arm(async (pose, feedback)=>{assert.equal(feedback,"measured");return state;});
  advance(25);await c.sample(state,now()-5);assert.equal(c.sent,1);
  assert.equal(log.filter(r=>r.type==="downlink"&&r.sent&&r.source==="browser/WebSerial").length,1);
  advance(25);await c.sample({...state,sample_ticks:2,ruckig_joint_angles_rad:{left_hand_index_MCP_joint:1.5}},now());
  assert.equal(c.armed,false);assert.equal(c.sent,1);
  await c.arm(async()=>state);advance(120);await c.sample({...state,sample_ticks:3},now()-110);
  assert.equal(c.armed,false);assert.equal(c.sent,1);
});
test("homing is optional; missing telemetry, old epoch and lost camera gate output",async()=>{
  const {c,state,advance,now}=control();
  c.homed=false;await c.arm(async()=>state);assert(c.armed);c.disarm();
  c.latestPayload=null;await assert.rejects(c.arm(async()=>state),/遥测/);c.latestPayload="measured";
  await c.arm(async()=>state);advance(25);await c.sample({...state,browser_epoch:0},now());assert.equal(c.sent,0);
  c.observeHand(false,now());assert.equal(c.armed,false);
  await c.arm(async()=>state);advance(501);c.watchdog();assert.equal(c.armed,false);
});
test("disarm cancels queued writes before USB and slow writer never builds position backlog",async()=>{
  const {c,state,advance,now}=control();await c.arm(async()=>state);advance(25);
  let release;c.tail=new Promise(resolve=>release=resolve);
  const send=c.sample(state,now());c.disarm();release();await send;assert.equal(c.sent,0);
  await c.arm(async()=>state);advance(25);
  c.writer={write:()=>new Promise(resolve=>release=resolve)};
  const first=c.sample({...state,sample_ticks:2},now());await Promise.resolve();
  await c.sample({...state,sample_ticks:3},now());assert(c.drops>0);
  release();await first;assert.equal(c.sent,1);
});
test("unconfirmed axes are reported, all remaining axes run, and partial homing permits feedback-seeded output",async()=>{
  let now=0;const writes=[];
  const c=new BrowserHandControl({now:()=>now, wait:async ms=>{now+=ms;c.telemetryAt=now;c.lastTelemetrySeq++;}});
  c.connected=true;c.side="left";c.writer={write:async b=>writes.push(b)};
  c.freshTelemetry=()=>true;c.lastTelemetrySeq=5;
  c.latest={linear:Array.from({length:14},()=>({mech_home_state:2,pos_act_mm:1})),
    rotary:Array.from({length:7},()=>({home_label:1,pos_actual_mm:0,iq_ref:0}))};
  c.latest.linear[0].mech_home_state=3;
  const report=await c.home(700,{q:0},{q:[0,1]});
  assert.equal(report.total,20);assert.equal(report.failures.length,7);
  assert.equal(writes.filter(b=>b[5]===0x25).length,6,"timeouts do not skip the remaining axes");
  assert.equal(c.homed,true);assert.equal(c.armed,false);assert.equal(c.homeResult,"partial");
  assert(report.failures.some(r=>r.joint==="left_hand_ring_MCP_joint"&&r.channel==="CAN1 M1"));
  assert(writes.some(b=>b[5]===0x12&&b[8]===2),"unfinished linear seek is stopped");
  assert(!writes.some(b=>b[5]===0x20),"homing never streams positions");
  c.handAt=now;c.latestPayload="measured";
  await c.arm(async(pose,feedback)=>{
    assert.equal(feedback,"measured");
    return {collision_mode:"stop",sample_collision_ready:true,ruckig_enabled:true,browser_epoch:1,
      ruckig_joint_angles_rad:{q:.2},rotary_positions_deg:Array(7).fill(10)};
  });
  assert.equal(c.armed,true);assert.equal(c.held.q,.2);
  assert.equal(writes.filter(b=>b[5]===0x20).length,1,"feedback hold precedes restoring drive power");
});
test("lost telemetry aborts homing and cannot mark it ready",async()=>{
  const c=new BrowserHandControl({wait:async()=>{c.freshTelemetry=()=>false;}});
  c.connected=true;c.side="left";c.writer={write:async()=>{}};c.freshTelemetry=()=>true;
  await assert.rejects(c.home(700,{q:0},{q:[0,1]}),/遥测超时/);
  assert.equal(c.homed,false);assert.equal(c.armed,false);assert.equal(c.homeReport,null);
});
test("a stalled serial write fails within a bound and invalidates the homed pose",async()=>{
  const {c,state,advance,now}=control();await c.arm(async()=>state);advance(25);
  let aborted=false;c.writer={write:()=>new Promise(()=>{}),abort:async()=>{aborted=true;}};
  await c.sample(state,now());
  assert.equal(c.armed,false);assert.equal(c.homed,false);assert.equal(c.sent,0);assert(aborted);
});
test("stop during pending port.open cannot reopen a serial connection",async()=>{
  const c=new BrowserHandControl();let release,closed=0;
  const port={open:()=>new Promise(resolve=>release=resolve),close:async()=>closed++};
  const opening=c.connect(port,"left");c.disarm("stop");release();
  await assert.rejects(opening,/取消/);assert.equal(c.connected,false);assert.equal(closed,1);
});
