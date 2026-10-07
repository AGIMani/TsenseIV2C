// Assign timestamps/sequence IDs at the event source, then transfer a snapshot
// to a worker. Motor commands never wait for JSON formatting or storage.
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
      this.error=null;this.t0=performance.now();this.sent=this.accepted=this.bytes=this.sourceDropped=0;
      const state=await this.call('start',[metadata],{origin:performance.timeOrigin+this.t0});
      Object.assign(this,state);this.active=true;this.deferredExport=false;this.sealing=null;
    })().finally(()=>{this.starting=null;});
    return this.starting;
  }
  record(type,data={}){
    if(!this.active||this.error)return null;
    // Bound queued structured clones when a worker/storage thread stalls.
    // Stop recording rather than letting log allocation starve USB control.
    if(this.sent-this.accepted>=2000){this.sourceDropped++;this.fail(Error('记录线程积压，已封存记录'));return null;}
    const seq=++this.seq;
    try{this.worker.postMessage({method:'record',type,data,t:(performance.now()-this.t0)/1000});this.sent++;}
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
