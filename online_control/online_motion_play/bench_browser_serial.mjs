// Used by bench_serial.py against its isolated localhost server. No real USB.
import {BrowserSerial} from './browser-serial.js';
import {frame,crc16} from './serial-protocol.js';

const [base,secondsText='3',mode='stream',delayText='0']=process.argv.slice(2),seconds=Number(secondsText),delay=Number(delayText);
if (!/^http:\/\/127\.0\.0\.1:\d+$/.test(base) || !(seconds>=1&&seconds<=30)) throw Error('Private benchmark URL and 1..30 seconds required');
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const headers={'X-Motion-Client':'node-serial-benchmark-client'};
async function api(action,body) {
  const raw=action==='serial_binary'||action==='serial_rx';
  if(action==='serial_stream')return fetch(base+'/api/'+action,{method:'POST',headers:{...headers,'Content-Type':'application/json'},body:JSON.stringify({token:body.token}),signal:body.signal});
  const response=await fetch(base+'/api/'+action,{method:body===undefined?'GET':'POST',
    headers:body===undefined?headers:{...headers,'Content-Type':raw?'application/octet-stream':'application/json'},
    body:body===undefined?undefined:raw?body:JSON.stringify(body),signal:AbortSignal.timeout(2000)});
  if(!response.ok)throw Error(await response.text());
  if(raw&&delay)await wait(delay);
  const result=raw?new Uint8Array(await response.arrayBuffer()):await response.json();
  if(action==='connect'&&mode==='binary')result.result.stream=false;
  return result;
}
let telemetrySeq=0,usbFrames=0,usbWrites=0;
const p={async open() {
  this.readable=new ReadableStream({start:c=>this.input=c});
  this.writable=new WritableStream({write:bytes=>{
    usbWrites++;
    for(let offset=0;offset<bytes.length;) {
      const size=8+bytes[offset+2]+(bytes[offset+3]<<8),wire=bytes.subarray(offset,offset+size);
      if(crc16(wire.subarray(0,-2))!==(wire.at(-2)|(wire.at(-1)<<8)))throw Error('Invalid USB CRC');
      if(wire[5]===0x20)usbFrames++;
      offset+=size;
    }
  }});
},async setSignals(){},async close(){}};
function feed() {
  const payload=new Uint8Array(484),view=new DataView(payload.buffer);
  view.setUint16(0,++telemetrySeq&65535,true);view.setUint32(4,telemetrySeq,true);
  p.input.enqueue(frame(2,payload));
}
const link=new BrowserSerial(api);
let feeding;
try {
  await api('control',{});await api('model',{model_id:'TsenseI_v3_0'});
  await link.connect(p,'TsenseI_v3_0');feed();feeding=setInterval(feed,2);
  await wait(40);
  const state=await api('state');
  await api('widget',{id:state.controls.realtimeJointControl.id,value:true});
  await wait(100);
  const start=performance.now(),before={frames:usbFrames,writes:usbWrites,rx:link.rxCount,exchanges:link.exchangeCount};
  await wait(seconds*1000);
  const dt=(performance.now()-start)/1000;
  if(!link.active)throw Error(link.error);
  const final=await api('state');if(!final.device.armed)throw Error(final.device.error);
  console.log(JSON.stringify({scope:'BrowserSerial JS + fetch + real server + mock Web Serial streams (Node, no physical USB)',
    mode,simulated_roundtrip_overhead_ms:delay,duration_s:dt,position_hz:(usbFrames-before.frames)/dt,usb_writes_hz:(usbWrites-before.writes)/dt,
    exchange_hz:(link.exchangeCount-before.exchanges)/dt,rx_kib_s:(link.rxCount-before.rx)/dt/1024}));
} finally {
  clearInterval(feeding);await link.disconnect();await api('release',{});
}
