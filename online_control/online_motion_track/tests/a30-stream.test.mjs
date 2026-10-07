import test from 'node:test';
import assert from 'node:assert/strict';
import {A30BrowserHandControl} from '../static/a30-browser-hand-control.js';
import * as protocol from '../static/a30-serial-protocol.js';

function controller() {
  let now=1000;
  const writes=[],events=[];
  const c=new A30BrowserHandControl({now:()=>now,wait:async ms=>{now+=ms;},recorder:{active:true,record:(type,data)=>{
    events.push({type,...data});return events.length;
  }}});
  const names=[...protocol.SELF.map(n=>`left_hand_${n}_joint`),...protocol.EXTERNAL.map(n=>`left_hand_${n}`)];
  c.side='left';c.connected=true;c.armed=true;c.epoch=1;c.job=7;
  c.latest=protocol.telemetry(new Uint8Array(484));
  c.held=Object.fromEntries(names.map(n=>[n,0]));
  c.limits=Object.fromEntries(names.map(n=>[n,[-2,2]]));
  c.motionLimits=Object.fromEntries(names.map(n=>[n,{velocity:n.includes('swing')?1.5:n.includes('rota')?2.5:4,acceleration:30,jerk:200}]));
  c.clockAnchor={server:0,local:0};c.lastSample=-1;c.lastPollAt=now;c.lastSentAt=now;
  c.writer={async write(bytes){writes.push(bytes.slice());}};
  c.streamAbort=new AbortController();
  const abort=c.streamAbort;
  function record(flags,tick=1,angle=0) {
    const bytes=new Uint8Array(flags===1?216:108),v=new DataView(bytes.buffer);
    bytes.set([71,84,82,49]);v.setUint32(4,1,true);v.setUint32(8,tick,true);
    v.setFloat64(16,now/1000,true);v.setUint32(24,flags,true);
    v.setFloat32(28,angle,true);
    if(flags===1){
      const linear={},rotary={};
      protocol.SELF.forEach((n,i)=>(protocol.ROTARY.has(i)?rotary:linear)[`left_hand_${n}_joint`]=i===0?angle*180/Math.PI:0);
      protocol.EXTERNAL.forEach(n=>rotary[`left_hand_${n}`]=0);
      bytes.set(protocol.positions(tick,linear,rotary,'left'),108);
    }
    return bytes;
  }
  return {c,names,writes,events,abort,record,advance:ms=>now+=ms,
    send:async(flags,tick=1,angle=0)=>c.streamSample(record(flags,tick,angle),names,7)};
}

test('direct mode sends a full target without velocity pacing and still pauses on collision',async()=>{
  const {c,writes,events,send,advance}=controller();
  c.directMode=true;
  await send(1,1);advance(2);
  const before=c.now();
  await send(1,2,.7);
  assert.equal(writes.length,2);assert(c.armed);assert.equal(c.now(),before);
  assert(Math.abs(Object.values(c.held)[0]-.7)<1e-6);
  assert.equal(events.filter(e=>e.type==='direct_command'&&e.sent).length,2);
  assert.deepEqual(events.findLast(e=>e.type==='direct_command').joints,c.lastWrittenSample.pose);
  assert.deepEqual(events.findLast(e=>e.type==='h7').joints_ruckig_ui,{});
  advance(2);await send(4,2);
  assert(c.collisionPaused);assert.equal(writes.length,2);
});

test('direct mode retains CRC and joint range validation',async()=>{
  const {c,names,writes,send,record,advance}=controller();
  c.directMode=true;
  await send(1,1);advance(2);
  const corrupt=record(1,2,.7);corrupt[120]^=1;
  await assert.rejects(c.streamSample(corrupt,names,7));
  c.rebaseControl=async()=>{throw Error('expected range recovery');};
  await send(1,2,3);
  assert.equal(writes.length,1);
  assert(!c.armed);
});

for(const directMode of [false,true])
test(`batched stationary positions keep 2 ms USB spacing (direct=${directMode})`,async()=>{
  const {c,names,record,advance,writes}=controller();c.directMode=directMode;
  const packets=Array.from({length:8},(_,i)=>{
    const bytes=record(1,i+1);new DataView(bytes.buffer).setFloat64(16,1+i*.002,true);return bytes;
  });
  advance(20);
  const starts=[];
  c.writer.write=async bytes=>{starts.push(c.now());writes.push(bytes.slice());};
  for(const packet of packets)await c.streamSample(packet,names,7);
  assert.equal(writes.length,8);assert(c.connected&&c.armed);
  assert.deepEqual(starts.slice(1).map((at,i)=>at-starts[i]),Array(7).fill(2));
});

test('USB write duration does not accumulate on top of every planner period',async()=>{
  const {c,names,record,advance,writes}=controller();
  const write=c.writer.write;
  c.writer.write=async bytes=>{await write(bytes);advance(.6);};
  for(let tick=0;tick<40;tick++){
    const arrival=1000+2*tick;
    advance(Math.max(0,arrival-c.now()));
    const packet=record(1,tick+1,.008*tick);
    new DataView(packet.buffer).setFloat64(16,arrival/1000,true);
    await c.streamSample(packet,names,7);
    assert(c.armed);
    assert(c.now()<=arrival+.601,`extra pacing delay at tick ${tick}: ${c.now()-arrival}`);
    assert.equal(c.lastSentAt,c.now());
  }
  assert.equal(writes.length,40);
});

test('timer overshoot does not accumulate into a stale-stream recovery loop',async()=>{
  const {c,names,record,advance,writes,events}=controller();
  c.wait=async ms=>advance(ms+.4); // Recorded USB starts were typically 2.4–2.8 ms apart.
  const write=c.writer.write;
  c.writer.write=async bytes=>{await write(bytes);advance(.2);};
  advance(20); // Start behind the producer, as with a batched network chunk.
  for(let tick=0;tick<1000;tick++){
    const arrival=1000+2*tick;
    advance(Math.max(0,arrival-c.now()));
    const packet=record(1,tick+1,.0003*tick);
    new DataView(packet.buffer).setFloat64(16,arrival/1000,true);
    await c.streamSample(packet,names,7);
    assert(c.now()-arrival<25,`pacing drift at ${tick}: ${c.now()-arrival} ms`);
  }
  assert.equal(writes.length,1000);
  assert(!events.some(e=>e.type==='trajectory_recovery'));
});

test('recovery starts its new stream immediately without waiting for status polling',async()=>{
  const {c,advance}=controller();
  c.streamSession='session';let started;
  c.rebaseControl=async()=>{
    advance(250);
    return {browser_epoch:2,server_time:1.25,ruckig_joint_angles_rad:{...c.held},motion_limits:c.motionLimits};
  };
  c.startStream=(session,state,requestedAt)=>{started={session,state,requestedAt,recovering:c.recovering};};
  await c.recoverTrajectory('stale_sample',{age_ms:110},7);
  assert.equal(started.session,'session');
  assert.equal(started.state.browser_epoch,2);
  assert.equal(started.requestedAt,1000);
  assert.equal(started.recovering,false);
  assert(c.armed);
  assert.equal(c.recoveryStarted,1000); // Cleared only by an accepted sample.
});

test('written and rendered samples carry the same epoch, tick and frame identity',async()=>{
  const {c,send,events}=controller();
  await send(1,15,.001);
  for(const type of ['conversion','h7','downlink']){
    const event=events.findLast(e=>e.type===type);
    assert.equal(event.sample_ticks,c.lastWrittenSample.tick);
    assert.equal(event.browser_epoch,c.lastWrittenSample.epoch);
    assert.equal(event.frame_id,c.lastWrittenSample.frame_id);
    assert.equal(event.written_at,c.lastWrittenSample.written_at);
  }
});

test('displayed send rate is a recent window and falls to zero when sending stops',async()=>{
  const {c,send,advance}=controller();
  for(let i=0;i<20;i++){await send(1,i+1);advance(2);}
  assert.equal(c.snapshot().local_send_hz,20);
  advance(1001);
  assert.equal(c.snapshot().local_send_hz,0);
  assert(c.snapshot().local_average_send_hz>0);
});

test('display sample is published only after successful USB completion',async()=>{
  const {c,send,advance}=controller();
  await send(1,1);
  const first=c.lastWrittenSample;
  let release;
  c.writer.write=()=>new Promise(resolve=>{release=resolve;});
  advance(2);
  const pending=send(1,2,.001);
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(c.lastWrittenSample,first);
  release();await pending;
  assert.equal(c.lastWrittenSample.tick,2);
  assert.deepEqual(c.lastWrittenSample.pose,c.held);
  const last=c.lastWrittenSample;
  c.writer.write=async()=>{throw Error('USB rejected');};
  advance(2);
  await assert.rejects(send(1,3,.002),/USB rejected/);
  assert.equal(c.lastWrittenSample,last);
});

test('disarm during a pending write cannot publish a new display sample',async()=>{
  const {c,send,advance}=controller();
  await send(1,1);const first=c.lastWrittenSample;
  let release;c.writer.write=()=>new Promise(resolve=>{release=resolve;});
  advance(2);const pending=send(1,2,.001);
  await new Promise(resolve=>setImmediate(resolve));
  c.disarm('manual');release();await pending;
  assert.equal(c.lastWrittenSample,first);
});

test('collision pauses USB for seconds and resumes on a checked sample without rearming',async()=>{
  const {c,writes,events,abort,send,advance}=controller();
  await send(1,1);
  assert.equal(c.sent,1);
  for(let i=0;i<100;i++){
    advance(20);await send(4,1);c.watchdog();
    assert(c.armed&&c.collisionPaused);
  }
  assert.equal(writes.length,1);assert.equal(c.lastSample,1);
  assert.equal(c.lastSentAt,1000); // Heartbeats are not fabricated USB writes.
  assert(!abort.signal.aborted);
  advance(2);await send(1,2,.001);
  assert(c.armed&&!c.collisionPaused);assert.equal(c.sent,2);
  assert.equal(writes.length,2);assert.equal(c.lastError,'');
  assert.equal(events.filter(e=>e.type==='control_pause').length,1);
  assert.equal(events.filter(e=>e.type==='control_resume').length,1);
  assert.equal(events.filter(e=>e.type==='control_disarm').length,0);
  assert.equal(events.filter(e=>e.type==='downlink').length,2);
  // Repeated collisions can pause/resume the same connection again.
  advance(2);await send(4,2);advance(2);await send(1,3,.001);
  assert.equal(writes.length,3);assert(c.armed);
});

test('pause before the first position frame also recovers',async()=>{
  const {c,writes,send,advance}=controller();
  await send(4,0);advance(20);await send(4,0);
  assert(c.armed&&c.collisionPaused);assert.equal(writes.length,0);
  advance(2);await send(1,1);assert.equal(writes.length,1);assert(c.armed);
});

for(const reason of ['manual','hard fault','lost stream','motor fault'])
test(`collision pause does not auto-rearm after ${reason}`,async()=>{
  const {c,writes,abort,send,advance}=controller();
  await send(1,1);advance(2);await send(4,1);
  if(reason==='manual')c.disarm('实时下发已关闭');
  if(reason==='hard fault')await send(2,1);
  if(reason==='lost stream'){advance(151);c.watchdog();}
  if(reason==='motor fault'){c.latest.external[0].online=true;c.latest.external[0].fault=1;c.watchdog();}
  assert(!c.armed&&!c.collisionPaused);assert(abort.signal.aborted);
  await send(1,2);
  assert.equal(writes.length,1);
});

test('a resumed stream still detects missing USB writes',async()=>{
  const {c,send,advance}=controller();
  await send(1,1);advance(2);await send(4,1);advance(20);await send(1,2);
  advance(151);c.lastPollAt=c.now();c.watchdog();
  assert(!c.armed);assert.match(c.lastError,/超时/);
});

for(const kind of ['epoch','crc','flags','pause payload'])
test(`collision resume retains ${kind} validation`,async()=>{
  const {c,names,writes,send,record,advance}=controller();
  await send(1,1);advance(2);await send(4,1);
  const bytes=record(1,2),v=new DataView(bytes.buffer);
  v.setFloat64(16,(c.now()+.002)/1000,true);
  if(kind==='stale')advance(101);
  if(kind==='epoch')v.setUint32(4,2,true);
  if(kind==='jump')v.setFloat32(28,1.5,true);
  if(kind==='crc')bytes[120]^=1;
  if(kind==='flags')v.setUint32(24,5,true);
  if(kind==='pause payload')v.setUint32(24,4,true);
  await assert.rejects(c.streamSample(bytes,names,7));
  assert.equal(writes.length,1);
});

test('server sampling time validates batched samples and USB output is paced',async()=>{
  const {c,names,writes,record,advance}=controller();
  const first=record(1,1),second=record(1,11,.025);
  new DataView(second.buffer).setFloat64(16,1.020,true);
  advance(20);
  await c.streamSample(first,names,7);
  advance(1);
  const before=c.now();
  await c.streamSample(second,names,7);
  assert(c.armed);assert.equal(writes.length,2);
  assert(c.now()-before>=5.24); // .025 / 4 rad/s = 6.25 ms, minus 1 ms elapsed.
});

test('browser uses the advertised per-joint limit, not an old hardcoded speed',async()=>{
  const {c,send,advance}=controller();
  await send(1,1);advance(20);
  await send(1,11,.08); // 4 rad/s would fail the former 2.5 rad/s policy.
  assert.equal(c.sent,2);assert(c.armed);
});

for(const kind of ['stale','position_limit','sample_jump'])
test(`${kind} discards the suspect sample and rebases without disabling or toggling power`,async()=>{
  const {c,names,writes,events,send,record,advance}=controller();
  await send(1,1);advance(2);
  const bytes=record(1,2),v=new DataView(bytes.buffer);
  if(kind==='stale')advance(101);
  if(kind==='position_limit')v.setFloat32(28,3,true);
  if(kind==='sample_jump')v.setFloat32(28,1,true);
  c.rebaseControl=async(pose,telemetry,options)=>{
    assert.deepEqual(options,{resume_from_commanded_pose:true});
    assert.deepEqual(pose,c.held);
    return {ruckig_joint_angles_rad:pose,browser_epoch:2,motion_limits:c.motionLimits};
  };
  await c.streamSample(bytes,names,7);
  assert(c.armed&&!c.recovering);assert.equal(c.epoch,2);assert.equal(writes.length,1);
  assert.equal(events.filter(e=>e.type==='trajectory_recovery').length,1);
  advance(2);
  const safe=record(1,3);new DataView(safe.buffer).setUint32(4,2,true);
  await c.streamSample(safe,names,7);assert.equal(writes.length,2);
  assert(writes.every(b=>b[5]===0x20));
});

test('manual stop during automatic rebase cannot be undone by its late response',async()=>{
  const {c,send,record,names,advance,writes}=controller();
  await send(1,1);advance(2);const invalid=record(1,2,1);
  let resolve;c.rebaseControl=()=>new Promise(r=>resolve=r);
  const pending=c.streamSample(invalid,names,7);
  assert(c.recovering);c.disarm('manual');
  resolve({browser_epoch:2,ruckig_joint_angles_rad:c.held,motion_limits:c.motionLimits});
  await pending;
  assert(!c.armed);assert.equal(c.epoch,1);assert.equal(writes.length,1);
});

test('an old hard-fault packet cannot trigger automatic recovery',async()=>{
  const {c,record,names,advance}=controller();
  const bytes=record(2,1);advance(101);
  c.rebaseControl=()=>assert.fail('hard faults must remain latched');
  await c.streamSample(bytes,names,7);
  assert(!c.armed);
});

test('953 ms stall can recover even when the watchdog runs before the rebase response',async()=>{
  const {c,send,record,names,advance,writes,events}=controller();
  await send(1,1);advance(2);
  const stale=record(1,2);advance(953);
  let resolve;c.rebaseControl=()=>new Promise(r=>resolve=r);
  const pending=c.streamSample(stale,names,7);
  c.watchdog();assert(c.armed&&c.recovering);
  advance(350);c.watchdog();assert(c.armed);
  resolve({browser_epoch:2,ruckig_joint_angles_rad:c.held,motion_limits:c.motionLimits});
  await pending;
  advance(200);c.watchdog();assert(c.armed); // Allow reopening the HTTP stream too.
  const fresh=record(1,3);new DataView(fresh.buffer).setUint32(4,2,true);
  await c.streamSample(fresh,names,7);
  assert(c.armed);assert.equal(writes.length,2);assert.equal(c.recoveryStarted,null);
  assert.equal(events.filter(e=>e.type==='trajectory_recovered').length,1);
});

test('watchdog-first stall starts only one recovery and cancels the old pacing continuation',async()=>{
  const {c,send,record,names,advance,writes}=controller();
  await send(1,1);advance(1);
  const sample=record(1,2,.006);new DataView(sample.buffer).setFloat64(16,1.002,true);
  let release;c.wait=()=>new Promise(r=>release=r);
  let resolve,calls=0;c.rebaseControl=()=>{calls++;return new Promise(r=>resolve=r);};
  const pending=c.streamSample(sample,names,7);
  advance(953);c.watchdog();c.watchdog();
  assert(c.armed&&c.recovering);assert.equal(calls,1);
  release();await pending;assert.equal(writes.length,1);
  resolve({browser_epoch:2,ruckig_joint_angles_rad:c.held,motion_limits:c.motionLimits});
  await new Promise(r=>setImmediate(r));
  assert(c.armed&&!c.recovering);
});

for(const phase of ['HTTP response','first fresh stream sample'])
test(`recovery has a bounded deadline while waiting for ${phase}`,async()=>{
  const {c,send,advance}=controller();await send(1,1);
  let resolve;c.rebaseControl=()=>new Promise(r=>resolve=r);
  const pending=c.recoverTrajectory('stale_sample',{},7);
  const state={browser_epoch:2,ruckig_joint_angles_rad:c.held,motion_limits:c.motionLimits};
  if(phase==='first fresh stream sample'){resolve(state);await pending;}
  advance(2001);c.watchdog();assert(!c.armed);assert.match(c.lastError,/恢复超过 2 秒/);
  if(phase==='HTTP response'){resolve(state);await pending;assert(!c.armed);assert.equal(c.epoch,1);}
});

test('motor faults are still checked during recovery',async()=>{
  const {c,send}=controller();await send(1,1);
  let resolve;c.rebaseControl=()=>new Promise(r=>resolve=r);
  const pending=c.recoverTrajectory('stale_sample',{},7);
  c.latest.external[0].online=true;c.latest.external[0].fault=1;c.watchdog();
  assert(!c.armed);assert.match(c.lastError,/故障/);
  resolve({});await pending;
});

test('an in-flight USB write cannot start a recovery from an uncertain last position',async()=>{
  const {c,send,advance}=controller();await send(1,1);
  c.rebaseControl=()=>assert.fail('must not rebase an in-flight USB command');
  c.positionWritePending=true;advance(151);c.watchdog();
  assert(!c.armed);
});
