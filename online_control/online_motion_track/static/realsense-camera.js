import {LeapCameraClient} from './leap-camera.js';

// The host retains the depth frame. Only its paired RGB pixels and immutable
// frame identity cross the browser boundary; inference returns that identity.
export function decodeRealSenseImage(buffer) {
  if (buffer.byteLength < 4) throw Error('RealSense 图像响应不完整');
  const length = new DataView(buffer).getUint32(0, true);
  if (length > 65536 || length + 4 > buffer.byteLength) throw Error('RealSense 图像头无效');
  const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, length)));
  const pixels = new Uint8Array(buffer, 4 + length);
  if (meta.has_frame && (meta.format !== 'rgb8' || !Number.isInteger(meta.width) ||
      !Number.isInteger(meta.height) || meta.width < 1 || meta.height < 1 ||
      meta.width > 4096 || meta.height > 4096 || pixels.length !== meta.width * meta.height * 3 ||
      !Number.isSafeInteger(meta.frame_id) || meta.frame_id < 0 ||
      typeof meta.stream_id !== 'string' || !meta.stream_id ||
      !Number.isFinite(meta.age_ms) || !Number.isFinite(meta.stale_after_ms) || meta.stale_after_ms <= 0)) {
    throw Error('RealSense RGB 图像尺寸、帧标识或格式无效');
  }
  return {meta, pixels};
}

export function realSenseFrameFields(meta) {
  return {source: 'realsense_rgbd', realsense_frame_id: meta?.frame_id ?? null,
    realsense_stream_id: meta?.stream_id ?? null, source_metadata: {camera: {...meta}}};
}

export function realSenseFusionStatus(fusion, tracking = false) {
  if (!fusion) return '等待同帧深度约束';
  if (!tracking) return fusion.message || '深度约束不可用 · 保持姿态';
  const count = fusion.measured_count ?? fusion.valid_depth_points ?? fusion.constrained_points;
  return `深度约束已融合 · 21 点${Number.isFinite(count) ? ` · ${count} 点有效深度` : ''}`;
}

export function realSenseCameraStatus(meta = {}, inferenceFps = null) {
  if (meta.error) return meta.error;
  if (!meta.connected) return '正在连接 D455f RGB + 深度…';
  if (!meta.has_frame || meta.age_ms == null || meta.age_ms >= meta.stale_after_ms)
    return '等待新的 RGB + 深度帧';
  const capture = Number.isFinite(meta.capture_fps) ? meta.capture_fps.toFixed(1) : '测量中';
  const inference = Number.isFinite(inferenceFps) ? inferenceFps.toFixed(1) : '测量中';
  return `RGB + 深度采集 ${capture} FPS · MediaPipe 识别 ${inference} FPS`;
}

// Reuse the native-image transport lifecycle: one long-poll overlaps one
// inference, superseded bitmaps are released, and close aborts both consumers.
export class RealSenseCameraClient extends LeapCameraClient {
  async fetchImage(session) {
    const started = this.clock();
    const response = await this.fetch('/api/realsense/frame', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, cache: 'no-store',
      body: JSON.stringify({session, after_revision: this.revision}), signal: this.controller.signal,
    });
    if (!response.ok) {
      const error = await response.json();
      throw Error(error.error || `RealSense 图像读取失败：HTTP ${response.status}`);
    }
    const {meta, pixels} = decodeRealSenseImage(await response.arrayBuffer());
    if (this.controller.signal.aborted) return {meta, bitmap: null, receivedAt: this.clock()};
    this.revision = meta.revision;
    const receivedAt = this.clock();
    if (meta.age_ms != null) meta.age_ms += receivedAt - started;
    if (!meta.has_frame || meta.age_ms == null || meta.age_ms >= meta.stale_after_ms)
      return {meta, bitmap: null, receivedAt};
    const canvas = this.canvas;
    if (canvas.width !== meta.width || canvas.height !== meta.height) {
      canvas.width = meta.width; canvas.height = meta.height;
      this.imageData = null;
    }
    const ctx = canvas.getContext('2d');
    this.imageData ??= ctx.createImageData(meta.width, meta.height);
    const rgba = this.imageData.data;
    for (let i = 0, j = 0; i < pixels.length; i += 3, j += 4) {
      rgba[j] = pixels[i]; rgba[j + 1] = pixels[i + 1]; rgba[j + 2] = pixels[i + 2]; rgba[j + 3] = 255;
    }
    ctx.putImageData(this.imageData, 0, 0);
    const bitmap = await this.createBitmap(canvas);
    return {meta, bitmap, receivedAt};
  }
}
