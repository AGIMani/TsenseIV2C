import {SCHEMA, JOINT_SET, ADB_SCHEMA, ADB_JOINT_SET, JOINT_NAMES, FINGER_CHAINS, collectHands, toAdbHand,
  tokenFromFragment, QuestPublisher} from './quest-capture-core.js';

const adbMode = /^\/q\/adb\/?$/.test(location.pathname);
const publishEndpoint = adbMode ? '/api/quest-adb/publish' : '/api/webxr/publish';
const skeletonFormat = adbMode ? {joint_set: ADB_JOINT_SET, units: 'm'}
  : {joint_set: JOINT_SET, joint_names: JOINT_NAMES};
const startButton = document.getElementById('start');
const stopButton = document.getElementById('stop');
const tokenInput = document.getElementById('pair-token');
const pairButton = document.getElementById('pair-connect');
const modeInput = document.getElementById('session-mode');
const statusEl = document.getElementById('status');
const canvas = document.getElementById('xr-canvas');
if (adbMode) {
  document.title = 'Quest 3 USB ADB · 21 点采集 · TsenseI';
  document.getElementById('capture-title').textContent = 'Quest 3 · USB ADB 21 点采集';
  document.getElementById('capture-description').textContent = '通过 USB ADB 将每只手的 21 个关节点发送到在线控制页（单位：米）。进入 XR 后放下控制器，让双手保持在头显视野内；桌面页负责手型映射、预览和机械手控制。';
  document.getElementById('capture-connection-hint').textContent = '请先用电脑的 Quest ADB 启动器建立 USB 连接，再从头显 Quest Browser 打开 localhost 采集地址，并启用系统手部追踪。USB 拔出、退出 XR 或追踪丢失时，桌面页会停止使用旧姿态。';
}
let pairedToken = tokenFromFragment(location.hash), pairing = false, pairingAttempt = 0;
const initialCode = '123456';
// Resolve the current PC session on every page load instead of reusing a
// cached capability from a stopped session. The human code never changes.
try { localStorage.removeItem('quest-control-pairing'); } catch {}
tokenInput.value = initialCode;
function rememberToken(token) {
  pairedToken = token;
}
// Keep the bookmarked entry stable; legacy token links still work in memory.
if (location.hash) history.replaceState(null, '', location.pathname + location.search);

let current = null;
let note = pairedToken ? '电脑会话已连接，点击进入 XR 即可。' : '固定配对码 123456，永久有效，正在连接电脑。';
const errorText = error => error instanceof Error ? error.message : String(error);

function updateStatus(text) {
  if (text) note = text;
  const c = current;
  statusEl.textContent = [
    note,
    `页面：${location.origin} · ${isSecureContext ? '安全上下文' : '需要 HTTPS'}`,
    `WebXR：${navigator.xr ? '可用' : '不可用'} · 会话：${c?.session ? '已进入' : c ? '正在启动 / 结束' : '未进入'}`,
    `传输：${adbMode ? 'USB ADB · ' : ''}${c?.network || '待连接'} · ${adbMode ? '发送：21 点 / 手' : '原生关节：25 / 手'}`,
    `左手：${c?.tracked.left ? '已追踪' : '未追踪'} · 右手：${c?.tracked.right ? '已追踪' : '未追踪'}`,
    `XR 帧：${c?.frames || 0} · 已发送：${c?.publisher.stats.sent || 0} · 已替换：${c?.publisher.stats.replaced || 0}`,
    `上传超时：${c?.publisher.stats.timedOut || 0} · 最近请求：${Math.round(c?.publisher.stats.requestMs || 0)} ms`,
  ].join('\n');
}

function controls(busy) {
  startButton.disabled = busy || pairing || !pairedToken || !isSecureContext || !navigator.xr;
  stopButton.disabled = !busy || Boolean(current?.ending);
  tokenInput.disabled = busy;
  pairButton.disabled = busy || pairing;
  modeInput.disabled = busy;
}

async function pairComputer() {
  if (current || pairing) return;
  const code = tokenInput.value.trim();
  if (!/^\d{6}$/.test(code)) {updateStatus('请输入完整的 6 位数字码。');return;}
  const attempt = ++pairingAttempt;
  pairing = true; rememberToken(''); controls(false);
  updateStatus('正在连接电脑…');
  try {
    const response = await fetch(adbMode ? '/api/quest-adb/pair' : '/api/webxr/pair', {method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify({code}),signal:AbortSignal.timeout(5000)});
    const result = await response.json();
    if (attempt !== pairingAttempt) return;
    if (!response.ok || !result.token) throw Error(result.error || '连接失败，请确认电脑已开始 Quest 识别后重试');
    rememberToken(result.token);
    updateStatus('电脑已连接，点击「进入 XR 并开始采集」。本次控制会话内无需再次输入。');
  } catch(error) {if (attempt === pairingAttempt) updateStatus(errorText(error));}
  finally {if (attempt === pairingAttempt) {pairing=false;controls(false);}}
}

function message(c, type, fields = {}) {
  return {
    schema: adbMode ? ADB_SCHEMA : SCHEMA, stream_id: c.streamId, type, sequence: c.sequence++,
    timestamp_ms: Date.now(), client_perf_ms: performance.now(),
    ...(!adbMode || ['webxr_local', 'webxr_local_floor'].includes(c.referenceSpaceType)
      ? {coordinate_space: c.referenceSpaceType} : {}),
    connection_state: c.network === '已连接' ? 'open' : 'connecting',
    ...(adbMode ? {...skeletonFormat, transport: 'adb'} : {}),
    ...fields,
  };
}

function createRenderer(gl) {
  function shader(type, source) {
    const value = gl.createShader(type);
    gl.shaderSource(value, source);
    gl.compileShader(value);
    if (!gl.getShaderParameter(value, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(value));
    return value;
  }
  const vertex = shader(gl.VERTEX_SHADER, `attribute vec3 position;
    uniform mat4 projection; uniform mat4 view;
    void main(){ gl_Position=projection*view*vec4(position,1.0); gl_PointSize=7.0; }`);
  const fragment = shader(gl.FRAGMENT_SHADER, `precision mediump float;
    uniform vec4 color; void main(){gl_FragColor=color;}`);
  const program = gl.createProgram();
  gl.attachShader(program, vertex); gl.attachShader(program, fragment); gl.linkProgram(program);
  gl.deleteShader(vertex); gl.deleteShader(fragment);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  const buffer = gl.createBuffer();
  const position = gl.getAttribLocation(program, 'position');
  const projection = gl.getUniformLocation(program, 'projection');
  const view = gl.getUniformLocation(program, 'view');
  const color = gl.getUniformLocation(program, 'color');
  return {
    draw(c, pose, hands) {
      const layer = c.session.renderState.baseLayer;
      gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
      gl.disable(gl.SCISSOR_TEST);
      if (c.mode === 'immersive-ar') gl.clearColor(0, 0, 0, 0);
      else gl.clearColor(0.025, 0.055, 0.04, 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      if (!pose) return;
      gl.useProgram(program); gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
      gl.enableVertexAttribArray(position); gl.vertexAttribPointer(position, 3, gl.FLOAT, false, 0, 0);
      gl.enable(gl.DEPTH_TEST);
      for (const eye of pose.views) {
        const viewport = layer.getViewport(eye);
        gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
        gl.uniformMatrix4fv(projection, false, eye.projectionMatrix);
        gl.uniformMatrix4fv(view, false, eye.transform.inverse.matrix);
        for (const hand of hands) {
          gl.uniform4fv(color, hand.handedness === 'left' ? [0.3, 1, 0.7, 1] : [0.3, 0.7, 1, 1]);
          const points = hand.joints.filter(joint => joint.valid).flatMap(joint => joint.position);
          gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(points), gl.DYNAMIC_DRAW);
          gl.drawArrays(gl.POINTS, 0, points.length / 3);
          const lines = [];
          for (const chain of FINGER_CHAINS) {
            for (let i = 1; i < chain.length; i++) {
              const a = hand.joints[chain[i - 1]], b = hand.joints[chain[i]];
              if (a.valid && b.valid) lines.push(...a.position, ...b.position);
            }
          }
          gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(lines), gl.DYNAMIC_DRAW);
          gl.drawArrays(gl.LINES, 0, lines.length / 3);
        }
      }
    },
    dispose() { gl.deleteBuffer(buffer); gl.deleteProgram(program); },
  };
}

async function finish(c, reason, text) {
  if (current !== c || c.ending) return;
  c.ending = true;
  c.tracked = {left: false, right: false};
  controls(true);
  updateStatus(text || '正在停止采集…');
  const sending = c.publisher.finish(message(c, 'session_ended', {reason}));
  if (c.session) await c.session.end().catch(() => {});
  await sending;
  c.renderer?.dispose();
  if (current === c) {
    current = null;
    controls(false);
    updateStatus(text || '采集已停止；再次进入 XR 会建立新的采集会话。');
  }
}

function syncCaptureVisibility(c) {
  if (current !== c || c.ending || !c.session) return false;
  const paused = document.visibilityState === 'hidden' || c.session.visibilityState !== 'visible';
  if (paused !== Boolean(c.paused)) {
    c.paused = paused;
    if (paused) {
      c.tracked = {left: false, right: false};
      c.publisher.offer(message(c, 'tracking_lost', {
        reason: 'xr_not_visible', session_state: c.session.visibilityState,
      }));
      updateStatus('XR 暂时失去前台，已暂停手势输入；返回后自动恢复。');
    } else {
      updateStatus('已返回 XR，等待新的手势帧。');
    }
  }
  return paused;
}

function frameLoop(c, predictedTime, frame) {
  if (current !== c || c.ending || !c.session) return;
  try {
    if (syncCaptureVisibility(c)) {
      c.renderer.draw(c, null, []);
      c.session.requestAnimationFrame((time, next) => frameLoop(c, time, next));
      return;
    }
    const pose = frame.getViewerPose(c.referenceSpace);
    // Viewer tracking loss makes joint poses unsafe to use, even if input sources remain.
    const hands = collectHands(frame, pose ? c.session.inputSources : [], c.referenceSpace);
    const wireHands = adbMode ? hands.map(toAdbHand) : hands;
    for (const hand of wireHands) {
      const side = hand.handedness;
      if (c.tracked[side] !== hand.tracked) {
        c.tracked[side] = hand.tracked;
        c.publisher.offer(message(c, hand.tracked ? 'tracking_restored' : 'tracking_lost', {
          handedness: side, reason: hand.tracked ? (adbMode ? 'all_21_joints_valid' : 'all_25_joints_valid') : 'hand_or_joint_unavailable',
        }));
      }
    }
    c.frames++;
    c.publisher.offer(message(c, 'hand_skeleton', {
      ...skeletonFormat,
      publisher_stats: {...c.publisher.stats},
      xr_predicted_display_time_ms: predictedTime, session_state: c.session.visibilityState, hands: wireHands,
    }));
    // Queue the validated observation before local stereo rendering. A slow
    // draw must not delay publication or hide its cost from client_perf_ms.
    c.renderer.draw(c, pose, hands);
    if (c.frames % 30 === 0) updateStatus();
    c.session.requestAnimationFrame((time, next) => frameLoop(c, time, next));
  } catch (error) {
    void finish(c, 'xr_frame_failed', `采集异常，已停止：${errorText(error)}`);
  }
}

async function startXR() {
  if (current) return;
  if (!isSecureContext || !navigator.xr) { updateStatus(adbMode ? '请用 Quest ADB 启动器建立 USB 连接，并在 Quest Browser 打开 localhost 采集页。' : '请在 Quest Browser 中使用 HTTPS 打开本页。'); return; }
  const token = pairedToken;
  if (!token) { updateStatus('请打开桌面页生成的 Quest 链接，或填写配对码。'); return; }
  const c = {
    streamId: crypto.randomUUID(), sequence: 0, mode: modeInput.value, session: null,
    ending: false, tracked: {left: false, right: false}, frames: 0,
    referenceSpaceType: 'webxr_page', network: '正在连接', renderer: null,
  };
  c.publisher = new QuestPublisher({token, endpoint: publishEndpoint,
    onStatus(state, stats, error) {
      c.network = state === 'connected' ? '已连接' : `连接中断，自动重试：${errorText(error)}`;
      if (current === c && state === 'error') updateStatus();
    },
    onFatal(error) { rememberToken(''); void finish(c, 'relay_rejected', `电脑会话已停止，请在电脑开始识别后点击连接电脑。配对码仍为 123456：${errorText(error)}`); },
  });
  current = c;
  controls(true);
  updateStatus('正在请求 XR 手部追踪权限…');
  try {
    // requestSession must run directly in the user click; do not await feature probes
    // or automatically retry another immersive mode after the permission gesture.
    const sessionPromise = navigator.xr.requestSession(c.mode, {
      requiredFeatures: ['hand-tracking'], optionalFeatures: ['local-floor', 'dom-overlay'],
      domOverlay: {root: document.getElementById('overlay-root')},
    });
    c.publisher.offer(message(c, 'client_hello', skeletonFormat));
    const session = await sessionPromise;
    if (current !== c || c.ending) { await session.end().catch(() => {}); return; }
    c.session = session;
    session.addEventListener('end', () => {
      c.session = null;
      void finish(c, 'xr_session_end', 'XR 会话已结束，采集已停止。');
    }, {once: true});
    session.addEventListener('visibilitychange', () => {
      syncCaptureVisibility(c);
    });
    const gl = canvas.getContext('webgl', {xrCompatible: true, alpha: true, antialias: false});
    if (!gl) throw new Error('WebGL 不可用');
    await gl.makeXRCompatible();
    if (current !== c || c.ending) return;
    session.updateRenderState({baseLayer: new XRWebGLLayer(session, gl, {alpha: true})});
    try {
      c.referenceSpace = await session.requestReferenceSpace('local-floor');
      c.referenceSpaceType = 'webxr_local_floor';
    } catch {
      c.referenceSpace = await session.requestReferenceSpace('local');
      c.referenceSpaceType = 'webxr_local';
    }
    if (current !== c || c.ending) return;
    c.renderer = createRenderer(gl);
    c.publisher.offer(message(c, 'session_started', {mode: c.mode, ...skeletonFormat}));
    updateStatus('正在采集，请展示双手。');
    session.requestAnimationFrame((time, frame) => frameLoop(c, time, frame));
  } catch (error) {
    await finish(c, 'session_start_failed', `启动失败：${errorText(error)}。可切换 AR / VR 后再次点击进入。`);
  }
}

startButton.addEventListener('click', () => { void startXR(); });
pairButton.addEventListener('click', () => { void pairComputer(); });
tokenInput.addEventListener('input', () => {
  ++pairingAttempt; pairing=false;rememberToken('');controls(false);
  if (/^\d{6}$/.test(tokenInput.value.trim())) void pairComputer();
});
stopButton.addEventListener('click', () => { if (current) void finish(current, 'user_stop_requested'); });
canvas.addEventListener('webglcontextlost', event => {
  event.preventDefault();
  if (current) void finish(current, 'webgl_context_lost', '图形上下文丢失，采集已停止。');
});
window.addEventListener('pagehide', () => {
  if (!current) return;
  const c = current;
  c.ending = true;
  c.publisher.close();
  // A final higher sequence invalidates any HTTP request already travelling to the server.
  navigator.sendBeacon?.(publishEndpoint, new Blob([JSON.stringify({
    token: c.publisher.token, message: message(c, 'session_ended', {reason: 'page_hidden'}),
  })], {type: 'application/json'}));
  void c.session?.end().catch(() => {});
});
document.addEventListener('visibilitychange', () => {
  if (current?.session) syncCaptureVisibility(current);
  else if (document.visibilityState === 'hidden' && current) void finish(current, 'page_hidden', '页面进入后台，采集已停止。');
});
controls(false);
updateStatus(!isSecureContext ? (adbMode ? 'USB 模式需要 localhost 安全上下文。请用电脑的 Quest ADB 启动器建立转发，再在头显打开 localhost 采集入口。' : '需要 HTTPS 安全上下文。请使用与桌面页相同的 HTTPS 入口。')
  : !navigator.xr ? '当前浏览器未提供 WebXR，请使用 Quest Browser。' : undefined);
if (!pairedToken) void pairComputer();
