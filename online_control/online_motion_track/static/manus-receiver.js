import {WebXRReceiver} from './webxr-receiver.js';

// Linux Integrated SDK capture and solving stay on the server. The browser
// only reads trusted observations/results and never uploads a second skeleton.
export class ManusReceiver extends WebXRReceiver {
  constructor(options) {
    super({staleAfterMs:200,...options, endpoint:options.combined?'/api/manus/frame':'/api/manus/latest',
      source:'manus_glove', pollMs:16, fixedCadence:true});
  }
}

export function manusStatus(snapshot) {
  if(snapshot.connection_error)return 'Manus 接收中断，正在重试';
  if(snapshot.hand)return `Manus 已连接 · 数据龄期 ${Math.round(snapshot.age_ms)} ms`;
  const meta=snapshot.source_metadata||{};
  if(meta.error)return `等待 Manus · ${meta.error}`;
  if(meta.event==='transport_error')return 'Manus 采集服务中断，正在重试';
  const staleAfter=meta.stale_after_ms??200;
  if(Number.isFinite(snapshot.age_ms)&&snapshot.age_ms>=staleAfter)
    return 'Manus 骨架已过期，等待新数据';
  if(['connecting','discovering','initializing'].includes(meta.event))
    return '正在连接本机 Manus 手套';
  return '等待所选 Manus 手套，请检查连接、左右手选择与本机 SDK 授权';
}
