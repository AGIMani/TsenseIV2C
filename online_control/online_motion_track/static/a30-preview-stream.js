import {consumeRecords} from '../../shared/stream.js';
import {StreamClock} from './stream-clock.js';

// Display-only samples contain no USB payload. Never opens or writes a serial port.
export class A30PreviewStream {
  constructor({now=()=>performance.now(),sampled=()=>{}}={}) { this.now=now;this.sampled=sampled;this.latest=null; }
  stop() { this.abort?.abort();this.abort=null;this.key=null;this.latest=null; }
  start(session,state,requestedAt) {
    const key=`${session}:${state.browser_epoch}`;
    // Refresh the server/browser clock mapping on every status poll. A slow
    // first response must not make every later sample look permanently stale.
    this.state=state;this.requestedAt=requestedAt;
    this.clockAnchor=(this.clock??=new StreamClock()).update(session,state.server_time,requestedAt,this.now());
    if(this.key===key&&this.abort)return;
    this.stop();this.key=key;
    const abort=this.abort=new AbortController();
    this.task=(async()=>{
      const response=await fetch('/api/a30/stream',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({session,epoch:state.browser_epoch,preview:true}),signal:abort.signal,cache:'no-store'});
      await consumeRecords(response,bytes=>{
        if(this.abort===abort)this.accept(bytes,this.state,this.requestedAt);
      },{signal:abort.signal});
    })().catch(()=>{}).finally(()=>{if(this.abort===abort){this.abort=null;this.latest=null;}});
  }
  accept(bytes,state,requestedAt) {
    if(bytes.length!==108||String.fromCharCode(...bytes.subarray(0,4))!=='GTR1')throw Error('预览样本格式错误');
    const v=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
    const epoch=v.getUint32(4,true),tick=v.getUint32(8,true),at=v.getFloat64(16,true),flags=v.getUint32(24,true);
    if(epoch!==state.browser_epoch||flags!==1||tick<=(this.latest?.tick??-1))return;
    const now=this.now(),anchor=this.clockAnchor||{server:state.server_time,local:requestedAt};
    const age=now-anchor.local-(at-anchor.server)*1000;
    if(!Number.isFinite(age)||age>100)return;
    if(state.control_joint_names?.length!==20)throw Error('预览关节列表错误');
    const pose={};
    state.control_joint_names.forEach((n,i)=>{
      const q=v.getFloat32(28+i*4,true);
      if(!Number.isFinite(q))throw Error('预览角度无效');
      pose[n]=q;
    });
    this.latest={pose,tick,epoch,frame_id:v.getInt32(12,true),received_at:now,planner_at:at};
    this.sampled(this.latest,state);
  }
}
