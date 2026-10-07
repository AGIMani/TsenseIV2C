// Test-only: memory H7. This module never calls navigator.serial or opens USB.
import {BrowserSerial} from './browser-serial.js';
import {frame} from './serial-protocol.js';
export async function runBenchmark({seconds=8,mode='worker',clientId,onProgress=()=>{},apiBase=''}) {
  const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
  const headers={'X-Motion-Client':clientId};
  async function api(action,body){
    if(action==='serial_stream')return fetch(apiBase+'/api/'+action,{method:'POST',headers:{...headers,'Content-Type':'application/json'},body:JSON.stringify({token:body.token}),signal:body.signal});
    const raw=action==='serial_rx'||action==='serial_binary';
    const r=await fetch(apiBase+'/api/'+action,{method:body===undefined?'GET':'POST',headers:{...headers,...(body===undefined?{}:{'Content-Type':raw?'application/octet-stream':'application/json'})},
      body:body===undefined?undefined:raw?body:JSON.stringify(body)});
    if(!r.ok)throw Error(await r.text());return raw?new Uint8Array(await r.arrayBuffer()):r.json();
  }
  const times=new Float64Array(60000);let count=0,seq=0,feeding,collect=false,moving,changing=false,targetError='',reversals=0;
  const port={async open(){
    this.readable=new ReadableStream({start:c=>this.input=c});
    this.writable=new WritableStream({write:bytes=>{
      for(let i=0;i<bytes.length;){const length=8+bytes[i+2]+(bytes[i+3]<<8);
        if(bytes[i+5]===0x20&&collect&&count<times.length)times[count++]=performance.now();i+=length;}
    }});
  },async setSignals(){},async close(){}};
  const link=new BrowserSerial(api);
  function feed(){const p=new Uint8Array(484),v=new DataView(p.buffer);v.setUint16(0,++seq&65535,true);v.setUint32(4,seq,true);port.input.enqueue(frame(2,p));}
  const capability={serial:!!navigator.serial,getPorts:typeof navigator.serial?.getPorts==='function',secure:isSecureContext,
    context:typeof document==='undefined'?'DedicatedWorker':'Window',userAgent:navigator.userAgent};
  let result;
  try{
    await api('control',{});await api('model',{model_id:'TsenseI_v3_0'});
    await link.connect(port,'TsenseI_v3_0');feed();feeding=setInterval(feed,4);await sleep(60);
    let state=await api('state');await api('widget',{id:state.controls.realtimeJointControl.id,value:true});
    await sleep(150);
    // Safe simulated movement: API targets exercise the real Ruckig + mapper.
    state=await api('state');const joints=state.joints;
    for(const j of joints)await api('widget',{id:j.id,value:j.min+(j.max-j.min)*.45,commit:true});
    await sleep(60);collect=true;const started=performance.now();onProgress({phase:'sampling',mode});
    moving=setInterval(async()=>{
      if(changing||!link.active)return;changing=true;
      try{const fraction=++reversals%2?.15:.45;
        for(const j of joints){if(!collect)break;await api('widget',{id:j.id,value:j.min+(j.max-j.min)*fraction,commit:true});}
      }catch(error){targetError=error.message;}finally{changing=false;}
    },1000);
    await sleep(seconds*1000);collect=false;
    const final=await api('state');const intervals=[];
    for(let i=1;i<count;i++)intervals.push(times[i]-times[i-1]);intervals.sort((a,b)=>a-b);
    const pct=p=>intervals.length?intervals[Math.min(intervals.length-1,Math.floor(p*intervals.length))]:null;
    result={mode,capability,duration_s:(performance.now()-started)/1000,positions:count,position_hz:count/seconds,
      observed_span_s:count>1?(times[count-1]-times[0])/1000:0,
      position_hz_while_sending:count>1?(count-1)*1000/(times[count-1]-times[0]):0,
      interval_ms:{min:pct(0),p50:pct(.5),p95:pct(.95),p99:pct(.99),max:pct(1)},
      in_1_to_4_percent:intervals.length?100*intervals.filter(x=>x>=1&&x<=4).length/intervals.length:0,
      below_1:intervals.filter(x=>x<1).length,above_4:intervals.filter(x=>x>4).length,
      active:link.active,armed:final.device.armed,error:link.error||final.device.error,targetError,reversals,
      scope:'Real browser + local Python Ruckig/mapper + in-memory USB; NOT physical H7'};
  }finally{clearInterval(feeding);clearInterval(moving);await link.disconnect();await api('release',{});}
  return result;
}
