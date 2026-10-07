import {WebXRReceiver} from './webxr-receiver.js';

// LeapC capture and background solving run on the Linux server. The browser
// reads the latest observation/result without uploading a second skeleton.
export class LeapMotionReceiver extends WebXRReceiver {
  constructor(options) {
    super({...options, endpoint:options.combined?'/api/leapmotion/frame':'/api/leapmotion/latest',
      source:'leapmotion', pollMs:16, fixedCadence:true});
  }
}

export function leapMotionStatus(snapshot) {
  if(snapshot.connection_error)return 'Leap Motion 2 接收中断，正在重试';
  if(snapshot.hand)return `Leap Motion 2 已连接 · 数据龄期 ${Math.round(snapshot.age_ms)} ms`;
  const meta=snapshot.source_metadata||{};
  if(meta.error)return `等待 Leap Motion 2 · ${meta.error}`;
  if(meta.event==='transport_error')return 'Leap Motion 2 采集服务中断，正在重试';
  // The shared receiver uses Infinity when the server has no hand timestamp.
  // That means no observation exists, not that a real skeleton has expired.
  if(Number.isFinite(snapshot.age_ms)&&snapshot.age_ms>=350)return 'Leap Motion 2 骨架已过期，等待新数据';
  if(meta.event==='tracking_lost')return 'Leap Motion 2 正在采集，未检测到所选手，请检查左右手选择并将手放入设备视野';
  if(meta.event==='connected')return 'Leap Motion 2 服务已连接，等待设备输出追踪帧';
  return '等待所选手，请检查设备、Ultraleap Tracking Service 与左右手选择';
}
