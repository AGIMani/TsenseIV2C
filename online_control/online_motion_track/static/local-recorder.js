// Only browser storage is used: no recording upload or server log endpoint.
const request = req => new Promise((resolve, reject) => { req.onsuccess = () => resolve(req.result); req.onerror = () => reject(req.error); });
const complete = tx => new Promise((resolve, reject) => { tx.oncomplete = resolve; tx.onerror = tx.onabort = () => reject(tx.error || new Error("本地存储失败")); });
const encoder=new TextEncoder();
export const LOG_PART_BYTES=512*1024*1024;
export function logModelFolder(model='') {
  if(/v3[_\. ]?0|a3\.?0/i.test(model))return 'A30';
  if(/v3[_\. ]?2|a3\.?2/i.test(model))return 'A32';
  if(/v2/i.test(model))return 'V2';
  throw new Error('日志缺少有效手部型号，无法选择 A30 / A32 / V2 目录');
}
export class LocalRecorder {
  constructor(onError = () => {}, options = {}) { this.onError = onError; this.options = options; this.active = false; this.chain = Promise.resolve(); this.pending = []; }
  async open() {
    if (this.db) return;
    if(this.opening)return this.opening;
    this.opening=(async()=>{
    const req = indexedDB.open(this.options.dbName || "tsensei-local-recordings", 2);
    req.onupgradeneeded = () => {
      if(!req.result.objectStoreNames.contains('sessions'))req.result.createObjectStore("sessions", {keyPath: "id"});
      if(!req.result.objectStoreNames.contains('chunks'))req.result.createObjectStore("chunks", {keyPath: ["id", "index"]});
      req.result.createObjectStore('settings');
    };
    this.db = await request(req);
    this.db.onversionchange=()=>{this.db.close();this.db=null;};
    this.directoryName=await request(this.db.transaction('settings').objectStore('settings').get('directoryName'));
    })();
    try{await this.opening;}finally{this.opening=null;}
  }
  async setDirectory(directory) {
    await this.open();
    const tx=this.db.transaction('settings','readwrite');tx.objectStore('settings').put(directory?.name||'','directoryName');await complete(tx);
    this.directory=directory;this.directoryName=directory?.name;this.savedFiles=new Map();return directory?.name||null;
  }
  async getDirectory(){await this.open();return this.directory?{name:this.directory.name,permission:await this.directory.queryPermission({mode:'readwrite'})}
    :this.directoryName?{name:this.directoryName,permission:'prompt'}:null;}
  async start(metadata) {
    if (this.active) return;
    await this.sealing;
    await this.open();
    // Do not discard recoverable in-memory chunks when starting another run.
    if(this.failedChunks?.length){
      try{
        const tx=this.db.transaction('chunks','readwrite');
        for(const chunk of this.failedChunks)tx.objectStore('chunks').put(chunk);
        await complete(tx);
      }catch{throw new Error("上一段记录仍有未落盘数据，请先下载全部分段并释放本机缓存");}
    }
    this.id = crypto.randomUUID(); this.seq = 0; this.index = 0; this.bytes = 0; this.error = null;
    this.t0 = performance.now(); this.pending = []; this.failedChunks = []; this.chain = Promise.resolve();
    this.deferredExport=false;this.sealing=null;
    this.queuedBytes=0;this.pendingBytes=0;this.counts={};this.dropped=0;
    const date = new Date(), pad = (n, size = 2) => String(n).padStart(size, "0");
    const stamp = `${date.getFullYear()}${pad(date.getMonth()+1)}${pad(date.getDate())}_${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}_${pad(date.getMilliseconds()*1000,6)}`;
    this.info = {id: this.id, name: `${this.options.prefix || "hand_control"}_${stamp}.jsonl`, started: Date.now(), ended: false, parts: []};
    const tx = this.db.transaction("sessions", "readwrite"); tx.objectStore("sessions").put(this.info); await complete(tx);
    this.active = true;
    const a32 = ["TsenseI_v3_2", "TsenseI_v3_2_withid"].includes(metadata.model_id);
    this.record("session", {schema: this.options.schema || "hand-control-session/v2", session_id:this.id, started_at: new Date().toISOString(),
      joint_unit: "rad", landmark_unit: "m", units: {linear: "mm", rotary: "deg", temperature: "degC", current: a32 ? "per_sample" : "firmware_native"},
      feedback_correlation: "latest successful host write before receive; not a firmware acknowledgement", metadata: {
      ...metadata, timezone: Intl.DateTimeFormat().resolvedOptions().timeZone,
      ...(a32 ? {current_unit_source: "per-motor current_unit / sample telemetry_units; unmarked legacy SELF remains native",
        homing_current_unit: "counts"} : {}),
      hardware_location: metadata.hardware_location || "visitor browser Web Serial", recording_location: "visitor IndexedDB/download",
      sampling: metadata.sampling || (this.options.schema==='hand-diagnostic/v1'
        ? "diagnostic events; selected streams sampled at most once per second; not a full control recording"
        : "all received retarget results and explicit skipped-frame reasons; received planner samples and local USB writes; not every server control tick")}});
    this.timer = setInterval(() => this.flush(), 250);
  }
  record(type, data = {}, atSeconds = (performance.now()-this.t0)/1000) {
    if (!this.active || this.error && type !== "session_end") {if(this.error)this.dropped=(this.dropped||0)+1;return null;}
    const seq = ++this.seq;
    let line;
    try{line=JSON.stringify({...data, type, seq, t: atSeconds})+"\n";}
    catch(error){this.dropped=(this.dropped||0)+1;this.fail(error);return null;}
    if(type==='session')this.info.header=line;
    this.counts??={};this.counts[type]=(this.counts[type]||0)+1;
    const size=encoder.encode(line).byteLength;
    const limit=this.options.partBytes??LOG_PART_BYTES;
    const reserve=encoder.encode(this.info.header||'').byteLength+2048;
    if(size+reserve>limit){this.dropped=(this.dropped||0)+1;this.fail(new Error('单条日志超过分段容量限制'));return null;}
    if(this.pendingBytes&&this.pendingBytes+size+reserve>limit)this.flush();
    this.pending.push(line); this.bytes += size;this.pendingBytes=(this.pendingBytes||0)+size;
    if (this.pending.length >= 100 || this.pendingBytes>=1024*1024) this.flush();
    if (type !== "session_end" && this.bytes > (this.options.maxBytes??LOG_PART_BYTES)) this.fail(new Error("本次记录达到容量限制，请结束会话下载后继续"));
    return seq;
  }
  fail(error) { if (this.error) return; this.error = error; this.onError(error); }
  flush() {
    if (!this.pending.length || !this.db) return this.chain;
    const chunk = {id: this.id, index: this.index++, text: this.pending.join("")}; this.pending = [];
    // Bound each export and the uncommitted backlog, not the recording duration.
    // JSON/storage work runs in the recording worker for full control capture.
    const size=this.pendingBytes;this.pendingBytes=0;
    const parts=this.info.parts??=[];
    let part=parts.at(-1);
    const reserve=encoder.encode(this.info.header||'').byteLength+2048;
    if(!part || part.bytes+size+reserve>(this.options.partBytes??LOG_PART_BYTES)){
      part={first:chunk.index,last:chunk.index,bytes:0};parts.push(part);
    }
    part.last=chunk.index;part.bytes+=size;
    const info=structuredClone(this.info);
    this.queuedBytes=(this.queuedBytes||0)+size;
    this.chain = this.chain.then(async () => {
      const tx = this.db.transaction(["chunks","sessions"], "readwrite");
      tx.objectStore("chunks").put(chunk);tx.objectStore("sessions").put(info);await complete(tx);
    }).catch(error => { this.failedChunks.push(chunk); this.fail(error); })
      .finally(()=>{this.queuedBytes-=size;});
    if(this.queuedBytes>(this.options.maxPendingBytes??64*1024*1024))this.fail(new Error("本机存储写入积压，记录已停止"));
    return this.chain;
  }
  async stop(reason = "stopped", {exportFile = true, sourceDropped = 0} = {}) {
    if (this.active) {
      this.record("session_end", {reason, duration_s: (performance.now()-this.t0)/1000, ended_at: new Date().toISOString(),
        dropped_events: (this.dropped||0)+sourceDropped, event_counts:{...this.counts}, recording_error: this.error?.message});
      this.active = false; clearInterval(this.timer); this.deferredExport=true;
      this.sealing=(async()=>{
        await this.flush();
        this.info.ended = !this.error;
        this.info.error = this.error?.message;
        try {
          const tx = this.db.transaction("sessions", "readwrite"); tx.objectStore("sessions").put(this.info); await complete(tx);
        } catch (error) { this.fail(error); }
      })();
    } else if (!this.deferredExport) return null;
    await this.sealing;
    // Sealing touches only pending chunks and metadata. Never load the entire
    // recording or build a large Blob while the control loop is running.
    if (!exportFile) return null;
    const file=await this.export(this.id);this.deferredExport=false;return file;
  }
  async export(id, partIndex = 0) {
    await this.open();
    if(id===this.id)await this.flush();
    let info = await request(this.db.transaction("sessions").objectStore("sessions").get(id));
    if(id===this.id)info=structuredClone(this.info);
    const parts=info?.parts||[],part=parts[partIndex];
    if(!Number.isInteger(partIndex)||partIndex<0||(parts.length? !part : partIndex!==0))throw new Error("记录分段不存在");
    const first=part?.first??0,last=part?.last??Number.MAX_SAFE_INTEGER;
    const chunks = await request(this.db.transaction("chunks").objectStore("chunks").getAll(IDBKeyRange.bound([id, first], [id, last])));
    if (id === this.id) chunks.push(...this.failedChunks.filter(c=>c.index>=first&&c.index<=last));
    chunks.sort((a,b) => a.index-b.index);
    if (!info || !chunks.length) throw new Error("未找到本机记录");
    const missingChunks=part?Math.max(0,last-first+1-chunks.length):0;
    const texts=chunks.map(c=>c.text),count=Math.max(1,parts.length);
    if(info.header){
      if(partIndex===0&&chunks[0].index===0)texts[0]=texts[0].slice(texts[0].indexOf('\n')+1);
      texts.unshift(JSON.stringify({...JSON.parse(info.header),
        recording_status:{ended:info.ended,error:id===this.id?this.error?.message:info.error,missing_chunks:missingChunks},
        ...(count>1?{recording_part:{index:partIndex+1,count,session_id:id}}:{})})+'\n');
    }
    const name=count>1?info.name.replace(/\.jsonl$/,`_part${String(partIndex+1).padStart(3,'0')}.jsonl`):info.name;
    let savedPaths,saveError;
    if(this.directory&&!(id===this.id&&this.active)){
      try{savedPaths=await this.saveToDirectory(info);}catch(error){saveError=error.message;}
    }
    return {id,partIndex,partCount:count,name,url:URL.createObjectURL(new Blob(texts, {type: "application/x-ndjson"})), savedPaths,saveError,
      incomplete: !info.ended || !!missingChunks || id === this.id && !!this.error};
  }
  async saveToDirectory(info) {
    if(await this.directory.queryPermission({mode:'readwrite'})!=='granted')throw Error('请点击“选择日志根目录”重新授权；日志仍保存在浏览器');
    const header=JSON.parse(info.header),folder=logModelFolder(header.metadata?.model_id);
    const directory=await this.directory.getDirectoryHandle(folder,{create:true});
    const parts=info.parts||[],paths=[];
    this.savedFiles??=new Map();
    for(let i=0;i<parts.length;i++){
      const part=parts[i],name=parts.length>1?info.name.replace(/\.jsonl$/,`_part${String(i+1).padStart(3,'0')}.jsonl`):info.name;
      const signature=JSON.stringify([info.id,i,part.last,info.ended,info.error]);
      if(this.savedFiles.get(name)!==signature){
        const handle=await directory.getFileHandle(name,{create:true}),writer=await handle.createWritable();
        try{
          const partHeader={...header,recording_status:{ended:info.ended,error:info.error},
            ...(parts.length>1?{recording_part:{index:i+1,count:parts.length,session_id:info.id}}:{})};
          await writer.write(JSON.stringify(partHeader)+'\n');
          for(let index=part.first;index<=part.last;index++){
            const chunk=await request(this.db.transaction('chunks').objectStore('chunks').get([info.id,index]))
              ||(info.id===this.id?this.failedChunks.find(c=>c.index===index):null);
            if(!chunk)throw Error(`日志分段缺少数据块 ${index}，请保留浏览器缓存`);
            await writer.write(index===0?chunk.text.slice(chunk.text.indexOf('\n')+1):chunk.text);
          }
          await writer.close();this.savedFiles.set(name,signature);
        }catch(error){await writer.abort().catch(()=>{});throw error;}
      }
      paths.push(`${this.directory.name}/${folder}/${name}`);
    }
    return paths;
  }
  async list() {
    await this.open();
    return (await request(this.db.transaction("sessions").objectStore("sessions").getAll())).sort((a,b) => b.started-a.started);
  }
  async remove(id) {
    if (id === this.id && this.active) throw new Error("请先结束当前记录");
    await this.open(); const tx = this.db.transaction(["sessions", "chunks"], "readwrite");
    tx.objectStore("sessions").delete(id);
    tx.objectStore("chunks").delete(IDBKeyRange.bound([id,0], [id,Number.MAX_SAFE_INTEGER])); await complete(tx);
    if(id===this.id){this.failedChunks=[];this.deferredExport=false;}
  }
}
