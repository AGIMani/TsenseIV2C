import test from 'node:test';
import assert from 'node:assert/strict';
import {LeapMotionReceiver, leapMotionStatus} from '../static/leapmotion-receiver.js';

function harness({combined=true,background=true}={}) {
  const requests=[],frames=[],states=[],errors=[];
  let clock=100, delay=0;
  const state={source:'leapmotion',source_epoch:3,revision:8,update_revision:2,
    age_ms:5,connected:true,hand:{label:'Left',world:[]},raw_leapmotion:{frame_id:123},
    update:{sequence:12,tracking:true,retarget_joint_angles_rad:{q:.2}}};
  const receiver=new LeapMotionReceiver({session:'leap',combined,background,now:()=>clock,
    fetcher:async(url,options)=>{requests.push([url,JSON.parse(options.body)]);clock+=delay;
      return {ok:true,json:async()=>({...state})};},
    onFrame:frame=>frames.push(frame),onStatus:status=>states.push(status),onError:error=>errors.push(error)});
  const poll=async()=>{await receiver.poll();clearTimeout(receiver.timer);};
  return {receiver,requests,frames,states,errors,state,poll,delay:value=>{delay=value;}};
}

test('Leap background reads acknowledge solved results once without uploading a pose or controlling cadence',async()=>{
  const h=harness();await h.poll();await h.poll();
  h.state.revision++;h.state.update=null;await h.poll();
  h.state.update_revision++;h.state.update={sequence:13,tracking:true};await h.poll();h.receiver.close();
  assert.deepEqual(h.frames.map(frame=>frame.update.sequence),[12,13]);
  assert(h.requests.every(([url])=>url==='/api/leapmotion/frame'));
  assert.deepEqual(h.requests.map(([,body])=>body.after_revision),[-1,2,2,2]);
  assert(h.requests.every(([,body])=>!('hand' in body)&&!('sequence' in body)&&!('timestamp_ms' in body)));
  assert.equal(h.frames[0].raw_leapmotion.frame_id,123);assert.equal(h.frames[0].source_epoch,3);
});

test('Leap calibration reads observations, with the caller retaining its input revision',async()=>{
  const h=harness({combined:false,background:false});await h.poll();await h.poll();h.receiver.close();
  assert.equal(h.requests[0][0],'/api/leapmotion/latest');
  assert.deepEqual(h.requests[0][1],{session:'leap'});
  assert.equal(h.frames.length,1);assert.equal(h.frames[0].revision,8);
  assert.equal(h.frames[0].combined_frame,false);
});

test('Leap nonbackground solve requests keep the sequence on the server',async()=>{
  const h=harness({background:false});await h.poll();h.receiver.close();
  assert.equal(h.requests[0][0],'/api/leapmotion/frame');
  assert.deepEqual(h.requests[0][1],{session:'leap'});
  assert.equal(h.frames[0].update.sequence,12);
});

test('a late Leap response clears live tracking while preserving raw solve for recording',async()=>{
  const h=harness();h.state.age_ms=300;h.delay(60);await h.poll();h.receiver.close();
  assert.equal(h.frames[0].hand,null);assert.equal(h.frames[0].update.tracking,false);
  assert.equal(h.frames[0].update.display_discarded,true);
  assert.equal(h.frames[0].update.retarget_joint_angles_rad.q,.2);
});

test('network interruption invalidates Leap input and recovers only with a new solve',async()=>{
  const h=harness();await h.poll();const fetcher=h.receiver.fetcher;
  h.receiver.fetcher=async()=>{throw new TypeError('offline');};await h.poll();
  assert.equal(h.frames.at(-1).source,'leapmotion');assert.equal(h.frames.at(-1).hand,null);
  assert.equal(h.receiver.closed,false);assert.equal(h.errors.length,0);
  h.receiver.fetcher=fetcher;await h.poll();assert.equal(h.frames.at(-1).hand,null);
  h.state.revision++;h.state.update_revision++;await h.poll();h.receiver.close();
  assert.equal(h.frames.at(-1).hand.label,'Left');
});

test('stopping Leap input ignores an in-flight response',async()=>{
  const h=harness();let finish;
  h.receiver.fetcher=()=>new Promise(resolve=>{finish=resolve;});
  const pending=h.poll();h.receiver.close();finish({ok:true,json:async()=>h.state});await pending;
  assert.equal(h.frames.length,0);assert.equal(h.states.length,0);
});

test('Leap status identifies SDK errors, data expiry and the selected hand',()=>{
  assert.match(leapMotionStatus({hand:null,age_ms:0}),/所选手/);
  assert.match(leapMotionStatus({hand:null,age_ms:400}),/过期/);
  assert.match(leapMotionStatus({hand:null,source_metadata:{error:'LeapC unavailable'}}),/LeapC unavailable/);
  assert.match(leapMotionStatus({connection_error:'network'}),/正在重试/);
  assert.match(leapMotionStatus({hand:{},age_ms:4}),/已连接/);
});

test('a no-hand response is not reported as an expired skeleton',async()=>{
  const h=harness();
  h.state.hand=null;h.state.age_ms=null;
  h.state.source_metadata={event:'tracking_lost'};
  await h.poll();h.receiver.close();
  assert.equal(h.states[0].age_ms,Infinity);
  assert.match(leapMotionStatus(h.states[0]),/未检测到所选手/);
  assert.doesNotMatch(leapMotionStatus(h.states[0]),/过期/);
  assert.match(leapMotionStatus({hand:null,age_ms:Infinity,source_metadata:{event:'connected'}}),/等待设备输出追踪帧/);
});
