import {WebXRReceiver} from './webxr-receiver.js';

export class WujiReceiver extends WebXRReceiver {
  constructor(options) {
    let sequence=0;
    const combined=options.combined===true;
    super({...options, endpoint:combined?'/api/wuji/frame':'/api/wuji/latest', source:'wuji_glove',
      pollMs:combined&&!options.background?8:16, fixedCadence:true,
      requestData:began=>combined&&!options.background?{sequence:sequence++,timestamp_ms:began,capture_unix_ms:Date.now()}: {},
      onFrame:snapshot=>options.onFrame({...snapshot,combined_frame:combined})});
  }
}

export function wujiStatus(snapshot) {
  if(snapshot.hand)return `手套已连接 · 数据龄期 ${Math.round(snapshot.age_ms)} ms`;
  const meta=snapshot.source_metadata||{};
  if(snapshot.connection_error||meta.event==='transport_error')return '手套采集服务连接中断，正在重试';
  if(meta.error)return `等待手套 · ${meta.error}`;
  if(snapshot.age_ms>=350)return '手套骨架已过期，等待新数据';
  return '等待所选手套，请检查电源、网络与左右手选择';
}
