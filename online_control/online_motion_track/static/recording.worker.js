// Full-fidelity angle logging stays off the camera/USB/UI thread. The existing
// LocalRecorder owns IndexedDB and the JSONL schema inside this worker.
import {LocalRecorder,LOG_PART_BYTES} from './local-recorder.js';
const storage=new LocalRecorder(error=>{
  self.postMessage({event:'error',message:error.message});
},{maxBytes:Infinity,partBytes:LOG_PART_BYTES});
let accepted=0;
self.onmessage=async({data:m})=>{
  try{
    if(m.method==='record'){
      storage.record(m.type,m.data,m.t);
      if(++accepted%50===0)self.postMessage({event:'progress',accepted,bytes:storage.bytes});
      return;
    }
    let result;
    if(m.method==='start'){
      await storage.start(m.args[0]);
      storage.t0=m.origin-performance.timeOrigin;
      accepted=0;
      result={id:storage.id,info:storage.info,seq:storage.seq};
    }else if(['stop','export','list','remove','flush','setDirectory','getDirectory'].includes(m.method)){
      if(m.method==='stop'&&m.recordingError&&!storage.error)storage.error=Error(m.recordingError);
      result=await storage[m.method](...m.args);
    }else throw Error('Unknown recording operation');
    self.postMessage({request:m.request,result,accepted,bytes:storage.bytes});
  }catch(error){
    if(m.method==='record')storage.fail(error);
    else self.postMessage({request:m.request,error:error.message});
  }
};
