// Validate trajectory metadata and remember the last successful physical write.
// Ruckig owns motion limits on the server, as in Form2. Network coalescing can
// skip intermediate samples, so a fixed per-packet delta must not close USB.
export class PositionGuard {
  constructor(velocities) {
    if (!Array.isArray(velocities) || velocities.length !== 20 || velocities.some(v => !Number.isFinite(v) || v <= 0)) throw Error('缺少关节限速配置');
    this.velocities = velocities; this.last = null; this.at = 0;
  }
  check(q) {
    if (q.length !== 20 || q.some(v => !Number.isFinite(v))) throw Error('轨迹关节数据无效');
  }
  commit(q, at) { if(!this.last)this.last=new Float32Array(20);this.last.set(q);this.at = at; }
}

export {makePacer} from '../shared/serial-timing.js';
