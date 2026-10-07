import {LocalRecorder} from "./local-recorder.js";
const SAMPLED=new Set(["telemetry","board_telemetry","planner_sample","mediapipe_raw","webxr_raw","retarget","retarget_skipped","control_target_skipped","usb_tx","downlink","api_slow"]);
const CRITICAL=new Set(["serial_connect","serial_disconnect","control_arm","control_disarm","control_pause","control_resume","trajectory_recovery","trajectory_recovered","emergency_stop","feedback_seed",
  "joint_power","joint_enable","homing_start","homing_complete","homing_axis_failed","homing_axis_complete"]);

export function errorDetails(error){
  return {name:error?.name||"Error",message:error?.message||String(error),stack:error?.stack||""};
}
function clean(value,depth=0){
  if(value instanceof Error)return errorDetails(value);
  if(typeof value==="string")return value.slice(0,8000);
  if(typeof value==="number")return Number.isFinite(value)?value:String(value);
  if(value===null||typeof value==="boolean")return value;
  if(depth>7)return "[depth limit]";
  if(Array.isArray(value))return value.slice(0,80).map(v=>clean(v,depth+1));
  if(value&&typeof value==="object")return Object.fromEntries(Object.entries(value).slice(0,100)
    .filter(([k])=>!/password|authorization|cookie|token/i.test(k)).map(([k,v])=>[k,clean(v,depth+1)]));
  return value===undefined?null:String(value);
}

// Independent of angle recording. Never writes to the server or changes motion state.
export class DiagnosticLogger {
  constructor({context=()=>({}),changed=()=>{}}={}){
    this.context=context;this.changed=changed;this.seq=0;this.history=[];this.historyBytes=0;this.dropped=0;
    this.beforeReady=[];this.lastSample=new Map();this.signatures=new Map();this.diskPending=[];this.diskTail=Promise.resolve();
    this.started=performance.now();this.pageId=crypto.randomUUID();
    this.storage=new LocalRecorder(e=>this.storageFailed(e),{dbName:"tsensei-diagnostic-logs",prefix:"hand_diagnostic",schema:"hand-diagnostic/v1"});
    this.ready=this.storage.start({page_id:this.pageId,origin:location.origin,user_agent:navigator.userAgent})
      .then(()=>{this.name=this.storage.info.name;this.stored=true;
        for(const item of this.beforeReady)this.storage.record(item.event,item.data);this.beforeReady=[];this.changed();})
      .catch(e=>{this.beforeReady=[];this.storageFailed(e);});
    this.record("page_open",{schema:"hand-diagnostic/v1",user_agent:navigator.userAgent,
      origin:location.origin,secure_context:isSecureContext,web_serial:!!navigator.serial});
    this.timer=setInterval(()=>{this.storage.flush();this.flushFile();},1000);
  }
  storageFailed(error){this.storageError=error.message||String(error);this.changed();}
  record(event,data={}){
    try{
      const entry=clean({...data,event,diagnostic_seq:++this.seq,wall_time:new Date().toISOString(),
        elapsed_ms:Math.round(performance.now()-this.started),page_id:this.pageId,context:this.context()});
      const line=JSON.stringify(entry)+"\n";
      this.history.push(line);this.historyBytes+=line.length;
      while(this.history.length>2000||this.historyBytes>4*1024*1024){this.historyBytes-=this.history.shift().length;this.dropped++;}
      if(this.fileHandle)this.diskPending.push(line);
      if(this.diskPending.length>2000){this.fileHandle=null;this.diskPending=[];this.fileError="日志文件写入滞后，已改存浏览器，请下载诊断日志";this.changed();}
      if(this.stored)this.storage.record(event,entry);
      else if(!this.storageError){this.beforeReady.push({event,data:entry});if(this.beforeReady.length>2000)this.beforeReady.shift();}
    }catch(e){this.storageFailed(e);}
  }
  change(event,data){const signature=JSON.stringify(data);if(this.signatures.get(event)===signature)return;
    this.signatures.set(event,signature);this.record(event,data);}
  controlEvent(type,data={}){
    if(!type.startsWith('serial_')&&!SAMPLED.has(type)&&!CRITICAL.has(type))return;
    if(SAMPLED.has(type)&&data.sent!==false&&!String(data.source||"").includes("hold")){
      const now=performance.now();if(now-(this.lastSample.get(type)??-Infinity)<1000)return;this.lastSample.set(type,now);
    }
    this.record(type,data);
  }
  async chooseDirectory(){
    if(!window.showDirectoryPicker)throw new Error("此浏览器不支持选择日志目录，请使用下载诊断日志");
    const directory=await window.showDirectoryPicker({id:"tsensei-diagnostics",mode:"readwrite"});
    await this.ready;await this.flushFile();
    const name=this.name||`hand_diagnostic_${this.pageId}.jsonl`;
    this.fileHandle=await directory.getFileHandle(name,{create:true});this.fileError=null;this.directoryName=directory.name;
    this.diskPending=[JSON.stringify({event:"file_history",page_id:this.pageId,omitted_earlier_events:this.dropped})+"\n",...this.history];
    // Every selection starts a new complete snapshot of this page's retained history.
    this.fileOffset=0;this.record("file_logging_started",{directory:directory.name,name});await this.flushFile();this.changed();
  }
  flushFile(){
    if(!this.fileHandle||!this.diskPending.length||this.fileFlushing)return this.diskTail;
    const handle=this.fileHandle,lines=this.diskPending.splice(0).join("");
    this.fileFlushing=true;
    const task=this.diskTail.then(async()=>{
      if(handle!==this.fileHandle)return;
      let writer;
      try{
        writer=await handle.createWritable({keepExistingData:true});
        await writer.seek(this.fileOffset);await writer.write(lines);
        const size=new TextEncoder().encode(lines).length;
        await writer.truncate(this.fileOffset+size);await writer.close();this.fileOffset+=size;
      }catch(error){await writer?.abort().catch(()=>{});throw error;}
    });
    this.diskTail=task.catch(e=>{if(handle===this.fileHandle){this.fileHandle=null;this.diskPending=[];this.fileError=e.message;this.changed();}})
      .finally(()=>{this.fileFlushing=false;});
    return this.diskTail;
  }
  async export(id){
    await this.ready;await this.storage.flush();await this.flushFile();
    if(id&&id!==this.storage.id)return this.storage.export(id);
    if(this.stored&&!this.storageError)return this.storage.export(this.storage.id);
    return {name:this.name||`hand_diagnostic_${this.pageId}.jsonl`,url:URL.createObjectURL(new Blob([
      JSON.stringify({event:"memory_fallback",omitted_earlier_events:this.dropped,storage_error:this.storageError})+"\n",...this.history],{type:"application/x-ndjson"}))};
  }
  async list(){await this.ready;return this.storage.list();}
  flush(){this.storage.flush();return this.flushFile();}
}
