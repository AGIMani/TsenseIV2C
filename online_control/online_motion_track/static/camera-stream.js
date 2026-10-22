const isRealSense = label => /real[\s_-]*sense/i.test(label || '');
const stopStream = stream => stream?.getTracks().forEach(track => track.stop());
const nearMaximum = (value, maximum) => Number.isFinite(value)
  && value >= maximum - Math.max(0.01, maximum * 0.001);

export function cameraCaptureLabel(camera) {
  const {width, height, frameRate} = camera.settings;
  const fps = Number.isFinite(frameRate) ? `${Number(frameRate.toFixed(2))} FPS` : '帧率未知';
  return `${camera.realSense ? 'RealSense' : 'CAMERA'} / ${width || '—'} × ${height || '—'} · 采集设置 ${fps}`;
}

// Resolve only after the selected camera is configured. Callers must await this
// before creating the detector; a failed maximum-FPS request must not become a
// silently capped 30 FPS RealSense session.
export async function openMediaPipeCamera({deviceId = '', label = '', signal,
  mediaDevices = navigator.mediaDevices, onStatus = () => {}} = {}) {
  let stream;
  const abort = () => stopStream(stream);
  const check = () => {
    signal?.throwIfAborted();
    if (stream && stream.getVideoTracks()[0]?.readyState === 'ended')
      throw Error('摄像头在配置期间已断开，请重新连接后开始。');
  };
  signal?.addEventListener('abort', abort, {once: true});
  try {
    check();
    stream = await mediaDevices.getUserMedia({audio: false, video: {
      width: {ideal: 640}, height: {ideal: 480},
      ...(!isRealSense(label) ? {frameRate: {ideal: 30, max: 30}} : {}),
      ...(deviceId ? {deviceId: {exact: deviceId}} : {facingMode: 'user'}),
    }});
    check();
    let track = stream.getVideoTracks()[0];
    if (!track) throw Error('所选设备没有可读取的视频轨道。');
    const realSense = isRealSense(track.label || label);
    let maximum = null;
    if (realSense) {
      onStatus('RealSense：正在查询最高可用采集帧率…');
      const capabilities = track.getCapabilities?.();
      maximum = capabilities?.frameRate?.max;
      if (!Number.isFinite(maximum) || maximum <= 0)
        throw Error('浏览器未提供 RealSense 的帧率范围，无法确认最高帧率。请使用支持相机能力查询的浏览器。');
      // Capabilities describe separate ranges, not resolution/FPS combinations.
      // Only FPS is mandatory. Resolution is allowed to fall for faster modes;
      // resizeMode:none avoids counting a cropped/downscaled mode as native.
      const constraints = {frameRate: {exact: maximum}, width: {ideal: 640}, height: {ideal: 480},
        ...(capabilities.resizeMode?.includes('none') ? {resizeMode: {exact: 'none'}} : {})};
      onStatus(`RealSense：正在配置最高 ${Number(maximum.toFixed(2))} FPS…`);
      const pinnedDevice = track.getSettings?.().deviceId || deviceId;
      try {
        await track.applyConstraints(constraints);
      } catch (error) {
        check();
        if (error.name !== 'OverconstrainedError') throw error;
        // Some UVC/browser combinations cannot raise capture FPS on a live
        // stream. Reopen this exact camera with FPS as a required constraint.
      }
      check();
      if (!nearMaximum(track.getSettings?.().frameRate, maximum)) {
        if (!pinnedDevice) throw Error('无法确认 RealSense 设备标识，未能重新配置最高帧率。');
        stopStream(stream);
        stream = null;
        stream = await mediaDevices.getUserMedia({audio: false,
          video: {...constraints, deviceId: {exact: pinnedDevice}}});
        check();
        track = stream.getVideoTracks()[0];
      }
      if (!nearMaximum(track?.getSettings?.().frameRate, maximum))
        throw Error(`RealSense 未达到浏览器报告的最高 ${maximum} FPS，已停止启动。请关闭其他占用相机的程序后重试。`);
    }
    check();
    const camera = {stream, realSense, maxFrameRate: maximum, settings: track.getSettings?.() || {}, label: track.label || label};
    onStatus(cameraCaptureLabel(camera));
    return camera;
  } catch (error) {
    stopStream(stream);
    throw error;
  } finally {
    signal?.removeEventListener('abort', abort);
  }
}
