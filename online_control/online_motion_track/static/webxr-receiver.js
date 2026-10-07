// The PC consumes each Quest observation once. Age uses server monotonic time
// plus request duration, so clocks on the headset and PC need not agree.
export class WebXRReceiver {
  constructor({session, onFrame, onStatus, onError, fetcher=(...args)=>globalThis.fetch(...args),
    now=()=>performance.now(), staleAfterMs=350, retryMs=100, onDiagnostic=()=>{},
    endpoint='/api/webxr/latest', source='webxr', pollMs=16, requestData=()=>({}), acceptSnapshot=null,
    fixedCadence=true, combined=false, background=false}) {
    Object.assign(this,{session,onFrame,onStatus,onError,fetcher,now,staleAfterMs,retryMs,onDiagnostic,endpoint,source,pollMs,requestData,acceptSnapshot,fixedCadence,combined,background});
    this.closed=false; this.revision=-1; this.lastLoss=-Infinity;
    this.updateRevision=-1;
    this.failures=0; this.polling=false;
  }
  async poll() {
    if(this.closed || this.polling)return;
    this.polling=true;
    const began=this.now();
    const controller=new AbortController();
    this.controller=controller;
    const deadline=setTimeout(()=>controller.abort(),this.staleAfterMs);
    try {
      const response=await this.fetcher(this.endpoint,{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({session:this.session,...this.requestData(began),
          ...(this.background?{after_revision:this.updateRevision}:{})}),signal:controller.signal,cache:'no-store',
      });
      const snapshot=await response.json();
      if(this.closed)return;
      if(!response.ok){
        const error=Error(snapshot.error||`骨架接收失败 (${response.status})`);
        error.fatal=!response.status || (response.status>=400 && response.status<500
          && ![408,429].includes(response.status));
        throw error;
      }
      if(this.failures)this.onDiagnostic(`${this.source}_transport_recovered`,{failures:this.failures});
      this.failures=0;
      const age=Number.isFinite(snapshot.age_ms)?snapshot.age_ms+this.now()-began:Infinity;
      const hand=snapshot.connected&&age<this.staleAfterMs?snapshot.hand:null;
      this.onStatus({...snapshot,hand,age_ms:age});
      // Input can advance before its solve finishes. Only acknowledge a
      // completed result, so a later solve of that input is never skipped.
      const updateRevision=snapshot.update_revision??snapshot.revision;
      const accepted=this.combined
        ?!!snapshot.update&&updateRevision!==this.updateRevision
        :this.acceptSnapshot?this.acceptSnapshot(snapshot):snapshot.revision!==this.revision;
      if(accepted || (!hand&&this.now()-this.lastLoss>=100)) {
        this.revision=snapshot.revision;
        if(accepted&&this.combined)this.updateRevision=updateRevision;
        if(!hand)this.lastLoss=this.now();
        // A late response must not display tracking or renew browser control.
        // Preserve the completed raw solve for recording even when rejected.
        const update=this.combined&&accepted?snapshot.update:null;
        this.onFrame({...snapshot,hand,age_ms:age,request_started_ms:began,
          combined_frame:this.combined,
          ...(this.combined?{update:update&&!hand&&update.tracking
            ?{...update,tracking:false,display_discarded:true,message:'输入数据已过期，等待新数据'}:update}:{}),
        });
      }
    } catch(error) {
      if(!this.closed){
        if(error.fatal){this.close();await this.onError(error);}
        else {
          this.failures++;
          // Invalidate input immediately without destroying the pairing. An
          // invalid revision prevents the solver from reusing any old pose.
          const loss={revision:-1,hand:null,tracking:false,connected:false,age_ms:null,combined_frame:this.combined,
            source:this.source,raw_webxr:null,connection_error:error.name==='AbortError'?'timeout':'network',
            source_metadata:{source:this.source,tracking:false,stale:true,event:'transport_error'}};
          this.onStatus(loss);this.onFrame(loss);this.lastLoss=this.now();
          if(this.failures===1)this.onDiagnostic(`${this.source}_transport_interrupted`,{
            reason:loss.connection_error,message:error.message});
        }
      }
    } finally {
      clearTimeout(deadline);
      this.polling=false;
      if(this.controller===controller)this.controller=null;
      if(!this.closed)this.timer=setTimeout(()=>this.poll(),this.failures?this.retryMs:
        Math.max(0,this.pollMs-(this.fixedCadence?this.now()-began:0)));
    }
  }
  close(){this.closed=true;clearTimeout(this.timer);this.controller?.abort();}
}

export function questStatus(snapshot) {
  if(snapshot.connection_error)return '电脑接收请求中断，正在重试';
  if(snapshot.hand)return `已连接 · 数据龄期 ${Math.round(snapshot.age_ms)} ms`;
  const metadata=snapshot.source_metadata||{};
  const raw=snapshot.raw_webxr||{};
  if(metadata.event==='waiting')return '等待 Quest 配对并进入 XR（更换手型后需重新配对）';
  if(metadata.event==='session_ended')return 'Quest XR 已结束，请在头显重新进入 XR';
  if(metadata.event==='tracking_lost' && raw.reason==='xr_not_visible')return 'Quest 暂时失去前台，返回 XR 后自动恢复';
  if(['client_hello','session_started'].includes(metadata.event))return 'Quest 已连接，等待 XR 手势帧';
  if(metadata.discard_reason==='late_transport')return 'Quest 上传延迟过大，已丢弃过期帧，等待新数据';
  if(!snapshot.connected || snapshot.age_ms>=metadata.stale_after_ms)return 'Quest 手势数据已过期，等待新数据';
  return 'Quest 已连接，未检测到所选手';
}
