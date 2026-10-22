// Infrared pixels only. Landmark detection uses the same MediaPipe worker as webcams.
export function decodeLeapImage(buffer) {
  if (buffer.byteLength < 4) throw Error('Leap 图像响应不完整');
  const length = new DataView(buffer).getUint32(0, true);
  if (length > 65536 || length + 4 > buffer.byteLength) throw Error('Leap 图像头无效');
  const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, length)));
  const pixels = new Uint8Array(buffer, length + 4);
  if (meta.has_frame && (meta.format !== 'gray8' || !Number.isInteger(meta.width) ||
      !Number.isInteger(meta.height) || meta.width < 1 || meta.height < 1 ||
      meta.width > 2048 || meta.height > 2048 || pixels.length !== meta.width * meta.height)) {
    throw Error('Leap 红外图像尺寸或格式无效');
  }
  return {meta, pixels};
}

export function leapCameraStatus(meta = {}, inferenceFps = null) {
  if (meta.error) return meta.error;
  if (!meta.connected) return '正在连接 Leap Motion 2 相机…';
  if (meta.age_ms === null || meta.age_ms >= meta.stale_after_ms) return '等待新的红外画面，请检查 USB 连接';
  const capture = meta.capture_fps == null ? '测量中' : meta.capture_fps.toFixed(1);
  const inference = inferenceFps == null ? '测量中' : inferenceFps.toFixed(1);
  return `相机采集 ${capture} / 120 FPS · MediaPipe 识别 ${inference} FPS` +
    (meta.fps_status === 'below_target' ? ' · 采集未达到 120 FPS，请检查 USB 3 连接和系统负载' : '');
}

export class LeapCameraClient {
  constructor(canvas, {fetchImpl = (...args) => fetch(...args), clock = () => performance.now(),
    createBitmap = (...args) => createImageBitmap(...args)} = {}) {
    this.canvas = canvas;
    this.fetch = fetchImpl;
    this.clock = clock;
    this.createBitmap = createBitmap;
    this.controller = new AbortController();
    this.revision = -1;
    this.meta = {};
    this.inferences = [];
    this.latest = null;
    this.waiter = null;
    this.pumping = false;
  }
  async read(session) {
    if (this.controller.signal.aborted) return null;
    if (!this.pumping) { this.pumping = true; void this.pump(session); }
    if (this.error) throw this.error;
    const item = this.latest || await new Promise((resolve, reject) => { this.waiter = {resolve, reject}; });
    this.latest = null;
    if (!item) return null;
    this.meta = {...item.meta};
    if (this.meta.age_ms != null) this.meta.age_ms += this.clock() - item.receivedAt;
    if (this.controller.signal.aborted || this.meta.age_ms >= this.meta.stale_after_ms) {
      item.bitmap?.close(); return null;
    }
    return item.bitmap;
  }
  async pump(session) {
    try {
      while (!this.controller.signal.aborted) {
        const item = await this.fetchImage(session);
        if (this.controller.signal.aborted) { item.bitmap?.close(); return; }
        if (this.waiter) {
          const waiter = this.waiter; this.waiter = null; waiter.resolve(item);
        } else {
          // Capture/transport overlap inference. Retain only the newest bitmap.
          this.latest?.bitmap?.close(); this.latest = item;
        }
      }
    } catch (error) {
      if (!this.controller.signal.aborted) {
        this.error = error; this.waiter?.reject(error); this.waiter = null;
      }
    }
  }
  async fetchImage(session) {
    const started = this.clock();
    const response = await this.fetch('/api/leap-camera/frame', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, cache: 'no-store',
      body: JSON.stringify({session, after_revision: this.revision}), signal: this.controller.signal,
    });
    if (!response.ok) {
      const error = await response.json();
      throw Error(error.error || `Leap 图像读取失败：HTTP ${response.status}`);
    }
    const {meta, pixels} = decodeLeapImage(await response.arrayBuffer());
    if (this.controller.signal.aborted) return {meta, bitmap:null, receivedAt:this.clock()};
    this.revision = meta.revision;
    // Conservatively include transport time so delayed images cannot drive control.
    const receivedAt = this.clock();
    if (meta.age_ms != null) meta.age_ms += receivedAt - started;
    if (!meta.has_frame || meta.age_ms == null || meta.age_ms >= meta.stale_after_ms) {
      return {meta, bitmap:null, receivedAt};
    }
    const canvas = this.canvas;
    if (canvas.width !== meta.width || canvas.height !== meta.height) {
      canvas.width = meta.width; canvas.height = meta.height;
      this.imageData = null;
    }
    const ctx = canvas.getContext('2d');
    this.imageData ??= ctx.createImageData(meta.width, meta.height);
    const rgba = this.imageData.data;
    for (let i = 0, j = 0; i < pixels.length; i++, j += 4) {
      rgba[j] = rgba[j + 1] = rgba[j + 2] = pixels[i]; rgba[j + 3] = 255;
    }
    ctx.putImageData(this.imageData, 0, 0);
    const bitmap = await this.createBitmap(canvas);
    return {meta, bitmap, receivedAt};
  }
  recordInference() {
    const now = this.clock();
    this.inferences.push(now);
    while (this.inferences.length > 240 || this.inferences[0] < now - 2000) this.inferences.shift();
  }
  get inferenceFps() {
    const times = this.inferences, duration = times.at(-1) - times[0];
    return duration >= 1000 ? (times.length - 1) * 1000 / duration : null;
  }
  close() {
    this.controller.abort();
    this.latest?.bitmap?.close(); this.latest = null;
    this.waiter?.resolve(null); this.waiter = null;
  }
}
