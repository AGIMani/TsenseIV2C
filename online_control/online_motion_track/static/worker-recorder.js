// Assign timestamps/sequence IDs at the event source, then transfer a snapshot
// to a worker. Motor commands never wait for JSON formatting or storage.
const ANGLE_STAGES = new Set(['retarget', 'direct', 'ruckig', 'mink', 'sdk']);
const CONTEXT_KEYS = ['frame_id', 'tracking_session_id', 'hand_side', 'model_id',
  'sample_ticks', 'tick', 'browser_epoch', 'planner_at', 'sample_at', 'source',
  'sent_at_unix_ms', 'written_at'];

// Normalize only the joint-angle stages. Telemetry, conversion and transport
// records remain available independently of the joint comparison curves.
// A status poll or failed write is never an SDK angle sample.
export function jointAngleRecords(type, data = {}) {
  const rows = [];
  const add = (stage, joints) => {
    if (!ANGLE_STAGES.has(stage) || !joints || typeof joints !== 'object' || Array.isArray(joints)) return;
    const entries = Object.entries(joints);
    if (!entries.length || entries.some(([, angle]) => !Number.isFinite(angle))) return;
    if (stage === 'sdk' && data.sent !== true) return;
    const row = {stage, joints: Object.fromEntries(entries)};
    for (const key of CONTEXT_KEYS) {
      const value = data[key];
      if (typeof value === 'string' || typeof value === 'number' && Number.isFinite(value)) row[key] = value;
    }
    if (stage === 'sdk') row.sent = true;
    rows.push(row);
  };
  if (type === 'joint_angles') add(data.stage, data.joints);
  else if (['retarget', 'mink', 'ruckig'].includes(type)) add(type, data.joints);
  else if (['direct_command', 'direct_filter'].includes(type)) add('direct', data.joints);
  else if (type === 'h7') {
    if (!data.direct_mode) add('ruckig', data.ruckig_output_joint_angles_rad);
    add('sdk', data.joints_ui);
  } else if (type === 'planner_sample' && !data.direct_mode) {
    add('ruckig', data.ruckig_output_joint_angles_rad);
  }
  return rows;
}

export class WorkerRecorder {
  constructor(onError=()=>{}, {makeWorker=()=>new Worker(new URL('./recording.worker.js',import.meta.url),{type:'module'})}={}){
    this.onError=onError;this.makeWorker=makeWorker;this.active=false;this.seq=0;
    this.pending=new Map();this.request=0;this.sent=0;this.accepted=0;this.bytes=0;
  }
  connect(){
    if(this.worker)return;
    this.worker=this.makeWorker();
    this.worker.onmessage=({data:m})=>{
      if(m.accepted!=null)this.accepted=m.accepted;
      if(m.bytes!=null)this.bytes=m.bytes;
      if(m.event==='error'){this.fail(Error(m.message));return;}
      if(m.request){const p=this.pending.get(m.request);this.pending.delete(m.request);
        if(p)m.error?p.reject(Error(m.error)):p.resolve(m.result);}
    };
    this.worker.onerror=e=>this.broken(Error(e.message||'记录线程已停止'));
    this.worker.onmessageerror=()=>this.broken(Error('记录线程数据传输失败'));
  }
  broken(error){
    this.workerFailure=error;
    for(const p of this.pending.values())p.reject(error);
    this.pending.clear();this.fail(error);
  }
  fail(error){if(this.error)return;this.error=error;this.onError(error);}
  call(method,args=[],extra={}){
    try{this.connect();if(this.workerFailure)throw this.workerFailure;}
    catch(error){return Promise.reject(error);}
    const request=++this.request;
    return new Promise((resolve,reject)=>{
      this.pending.set(request,{resolve,reject});
      try{this.worker.postMessage({request,method,args,...extra});}
      catch(error){this.pending.delete(request);reject(error);}
    });
  }
  start(metadata){
    if(this.active)return Promise.resolve();
    if(this.starting)return this.starting;
    this.starting=(async()=>{
      await this.sealing;
      this.error=null;this.t0=performance.now();this.wallStart=Date.now();
      this.stageSamples=new Map();this.sent=this.accepted=this.bytes=this.sourceDropped=0;
      const state=await this.call('start',[{...metadata, joint_angle_stages:[...ANGLE_STAGES]}],{origin:performance.timeOrigin+this.t0});
      Object.assign(this,state);this.active=true;this.deferredExport=false;this.sealing=null;
    })().finally(()=>{this.starting=null;});
    return this.starting;
  }
  record(type,data={}){
    if(!this.active||this.error)return null;
    let seq = null;
    const angleOnly = ['joint_angles', 'mink', 'ruckig', 'direct_command', 'direct_filter'].includes(type);
    if (!angleOnly) {
      let auxiliary = data;
      if (['retarget', 'h7', 'planner_sample', 'control_target', 'direct_filter_detail'].includes(type)) {
        auxiliary = {...data};
        for (const key of ['joints', 'joints_ui', 'joints_hw', 'joints_ruckig_ui', 'target',
          'ruckig_output_joint_angles_rad', 'direct_filter', 'input', 'candidate', 'output',
          'input_joint_angles_rad', 'filtered_joint_angles_rad', 'checked_joint_angles_rad']) delete auxiliary[key];
      }
      seq = this.postRecord(type, auxiliary);
    }
    for (const row of jointAngleRecords(type, data)) {
      if (this.error) break;
      // Repeated status responses are not new planner samples.
      const tick = row.sample_ticks ?? row.tick;
      if (Number.isFinite(tick)) {
        const key = `${row.stage}/${row.tracking_session_id||''}/${row.hand_side||''}/${row.model_id||''}/${row.source||''}`;
        const sample = `${row.browser_epoch??''}/${tick}`;
        if (this.stageSamples.get(key) === sample) continue;
        this.stageSamples.set(key, sample);
      }
      seq = this.postRecord('joint_angles', row);
    }
    return seq;
  }
  postRecord(type,data){
    // Bound queued structured clones when a worker/storage thread stalls.
    // Stop recording rather than letting log allocation starve USB control.
    if(this.sent-this.accepted>=2000){this.sourceDropped++;this.fail(Error('记录线程积压，已封存记录'));return null;}
    const seq=++this.seq;
    let at = (performance.now()-this.t0)/1000;
    // A batch of SDK samples arrives in one browser callback. Its transport
    // rows and angle rows must share the original per-write sample clock.
    if (['joint_angles', 'h7', 'downlink'].includes(type)) {
      if (Number.isFinite(data.written_at)) at=(data.written_at-this.t0)/1000;
      else if (Number.isFinite(data.sent_at_unix_ms)) at=(data.sent_at_unix_ms-this.wallStart)/1000;
    }
    try{this.worker.postMessage({method:'record',type,data,t:at});this.sent++;}
    catch(error){this.sourceDropped++;this.fail(error);return null;}
    return seq;
  }
  async stop(reason='stopped',{exportFile=true}={}){
    if(this.starting)await this.starting;
    if(this.active){
      this.active=false;this.deferredExport=true;
      this.sealing=this.call('stop',[reason,{exportFile:false,sourceDropped:this.sourceDropped}],{recordingError:this.error?.message});
    }else if(!this.deferredExport)return null;
    await this.sealing;
    if(!exportFile)return null;
    const file=await this.export(this.id);this.deferredExport=false;return file;
  }
  flush(){return this.worker?this.call('flush'):Promise.resolve();}
  export(id,partIndex=0){return this.call('export',[id,partIndex]);}
  list(){return this.call('list');}
  setDirectory(directory){return this.call('setDirectory',[directory]);}
  getDirectory(){return this.call('getDirectory');}
  remove(id){return this.call('remove',[id]);}
}
