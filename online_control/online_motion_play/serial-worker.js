import {BrowserSerial} from './browser-serial.js';

let link, clientId, heartbeat=performance.now(), activeJob=false, generation=0;
const sendState = () => {
  if (!link) return;
  postMessage({type:'state',state:{active:link.active,error:link.error,txCount:link.txCount,rxCount:link.rxCount,
    rttMs:link.rttMs,deviceLabel:link.deviceLabel,metrics:link.sampleMetrics(),connected:!!link.port}});
};
async function api(action,body) {
  const headers={'X-Motion-Client':clientId};
  const raw=action==='serial_binary'||action==='serial_rx';
  if(action==='serial_stream') return fetch('/api/'+action,{method:'POST',headers:{...headers,'Content-Type':'application/json'},
    body:JSON.stringify({token:body.token}),signal:body.signal,cache:'no-store'});
  const response=await fetch('/api/'+action,{method:body===undefined?'GET':'POST',
    headers:body===undefined?headers:{...headers,'Content-Type':raw?'application/octet-stream':'application/json'},
    body:body===undefined?undefined:raw?body:JSON.stringify(body),signal:AbortSignal.timeout(raw?180:2500),cache:'no-store'});
  if(!response.ok)throw Error((await response.json()).error||response.statusText);
  return raw?new Uint8Array(await response.arrayBuffer()):response.json();
}
export async function matchingPort(serial,info) {
  // SerialPort is not transferable; never guess between two identical VID/PIDs.
  const ports=await serial.getPorts();
  const matches=ports.filter(port=>{
    const candidate=port.getInfo();
    return ['usbVendorId','usbProductId','bluetoothServiceClassId'].every(k=>candidate[k]===info[k]);
  });
  if(matches.length!==1) throw Error('无法唯一识别已选串口：请在网站串口权限中只保留目标设备，再重新授权');
  return matches[0];
}
self.onmessage = async ({data}) => {
  if(data.type==='heartbeat'){heartbeat=performance.now();return;}
  if(data.type==='probe'){
    postMessage({id:data.id,result:{worker:true,serial:!!navigator.serial,getPorts:typeof navigator.serial?.getPorts==='function',
      secure:isSecureContext,userAgent:navigator.userAgent}});return;
  }
  let ownsJob=false;
  try {
    if(data.type==='disconnect') { generation++;await link?.disconnect(data.reason);sendState();postMessage({id:data.id,result:true});return; }
    if(data.type!=='connect') throw Error('未知 Worker 操作');
    if(activeJob || link?.port)throw Error('串口已连接或正在连接');
    activeJob=true;ownsJob=true;const current=++generation;clientId=data.clientId;heartbeat=performance.now();
    if(!navigator.serial?.getPorts)throw Error('此浏览器的 Worker 不支持 Web Serial');
    const port=await matchingPort(navigator.serial,data.info);
    if(current!==generation)throw Error('连接已取消');
    link=new BrowserSerial(api,sendState);
    await link.connect(port,data.model,data.baudRate);
    if(current!==generation){await link.disconnect('连接已取消');throw Error('连接已取消');}
    sendState();postMessage({id:data.id,result:true});
  } catch(error){postMessage({id:data.id,error:error.message});}
  finally{if(ownsJob)activeJob=false;}
};
setInterval(()=>{
  if(link?.active && performance.now()-heartbeat>2000) void link.disconnect('页面失去响应，Worker 已停止下发');
  sendState();
},200);
