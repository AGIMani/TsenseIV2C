const SLOT_N = 18;
const MIT_N = 2;
const ANGLE = new Set([6, 7, 14, 15]);
const ROT_MIN = -20;
const ROT_MAX = 20;
const DIP = new Set([0, 3, 8, 11, 16]);
const PIP = new Set([1, 4, 9, 12]);
const PART_NAME = ["指尖", "指腹", "指根"];
const TOUCH_SRC = [
  "手指1 · CAN1", "手指2 · CAN1", "手指3 · CAN2",
  "手指4 · CAN2", "拇指 · CAN3", "手掌 · UART", "全部手指",
];

const state = {
  self: Array(SLOT_N).fill(0),
  mitPos: [0, 0],
  mitVel: [0, 0],
  mitIq: [0, 0],
  streaming: false,
  sine: null,
  connected: false,
  scanning: false,
};

function $(id) { return document.getElementById(id); }

function linearMax(i) {
  if (i === 1) return 11.76149;
  if (DIP.has(i)) return 12.437477;
  if (i === 17 || PIP.has(i)) return 16.939022;
  return 19.627779;
}

function slotLabel(i) {
  const bus = i < 8 ? "CAN1" : i < 16 ? "CAN2" : "CAN3";
  const id = i < 8 ? i + 1 : i < 16 ? i - 7 : i - 15;
  const kind = ANGLE.has(i) ? "旋转" : DIP.has(i) ? "DIP" : PIP.has(i) ? "PIP" : "MCP";
  const unit = ANGLE.has(i) ? "deg" : "mm";
  return { bus, id, kind, unit, text: `#${String(i).padStart(2, "0")}  ${bus} M${id}  ${kind}` };
}

function buildMotors() {
  const host = $("motorCards");
  host.innerHTML = "";
  const groups = [
    { title: "CAN1  ·  自研", slots: [...Array(8).keys()] },
    { title: "CAN2  ·  自研", slots: [...Array(8).keys()].map((i) => i + 8) },
    { title: "CAN3  ·  自研 + MIT", slots: [16, 17] },
  ];
  groups.forEach((g) => {
    const card = document.createElement("div");
    card.className = "card";
    card.innerHTML = `<h3>${g.title}</h3>`;
    g.slots.forEach((i) => {
      const L = slotLabel(i);
      const row = document.createElement("div");
      row.className = "motor";
      row.innerHTML = `<span title="${L.text}">${L.text}</span>
        <input type="range" min="${ANGLE.has(i) ? ROT_MIN : 0}" max="${ANGLE.has(i) ? ROT_MAX : linearMax(i)}" step="0.1" value="0" data-self="${i}" />
        <input class="narrow" value="0" data-selfn="${i}" />
        <span class="unit">${L.unit}</span>`;
      card.appendChild(row);
    });
    if (g.slots[0] === 16) {
      for (let m = 0; m < MIT_N; m++) {
        const row = document.createElement("div");
        row.className = "motor";
        row.innerHTML = `<span title="MIT${m + 1} 输出轴">MIT${m + 1} 输出轴</span>
          <input type="range" min="-180" max="180" step="0.1" value="0" data-mit="${m}" />
          <input class="narrow" value="0" data-mitn="${m}" />
          <span class="unit">deg</span>`;
        card.appendChild(row);
        const extra = document.createElement("div");
        extra.className = "motor mit-extra";
        extra.innerHTML = `<span>MIT${m + 1} 转速 / 电流</span>
          <input class="narrow" value="0" data-mitvel="${m}" />
          <span class="unit">rpm</span>
          <input class="narrow" value="0" data-mitiq="${m}" />
          <span class="unit">A</span>`;
        card.appendChild(extra);
      }
    }
    host.appendChild(card);
  });
  host.querySelectorAll("[data-self]").forEach((el) => {
    el.addEventListener("input", () => {
      const i = +el.dataset.self;
      state.self[i] = +el.value;
      host.querySelector(`[data-selfn="${i}"]`).value = el.value;
      pushTargets();
    });
  });
  host.querySelectorAll("[data-selfn]").forEach((el) => {
    el.addEventListener("change", () => {
      const i = +el.dataset.selfn;
      let v = +el.value || 0;
      const lo = ANGLE.has(i) ? ROT_MIN : 0;
      const hi = ANGLE.has(i) ? ROT_MAX : linearMax(i);
      if (v < lo) v = lo;
      if (v > hi) v = hi;
      state.self[i] = v;
      el.value = v;
      const r = host.querySelector(`[data-self="${i}"]`);
      if (r) r.value = v;
      pushTargets();
    });
  });
  host.querySelectorAll("[data-mit]").forEach((el) => {
    el.addEventListener("input", () => {
      const i = +el.dataset.mit;
      state.mitPos[i] = +el.value;
      host.querySelector(`[data-mitn="${i}"]`).value = el.value;
      pushTargets();
    });
  });
  host.querySelectorAll("[data-mitn]").forEach((el) => {
    el.addEventListener("change", () => {
      const i = +el.dataset.mitn;
      state.mitPos[i] = +el.value || 0;
      const r = host.querySelector(`[data-mit="${i}"]`);
      if (r) r.value = state.mitPos[i];
      pushTargets();
    });
  });
  host.querySelectorAll("[data-mitvel]").forEach((el) => {
    el.addEventListener("change", () => { state.mitVel[+el.dataset.mitvel] = +el.value || 0; pushTargets(); });
  });
  host.querySelectorAll("[data-mitiq]").forEach((el) => {
    el.addEventListener("change", () => { state.mitIq[+el.dataset.mitiq] = +el.value || 0; pushTargets(); });
  });
}

function buildSp() {
  const sel = $("spSlot");
  for (let i = 0; i < SLOT_N; i++) {
    const o = document.createElement("option");
    o.value = i;
    o.textContent = slotLabel(i).text;
    sel.appendChild(o);
  }
}

function buildTouch() {
  const src = $("touchSrc");
  TOUCH_SRC.forEach((n, i) => {
    const o = document.createElement("option");
    o.value = i === 6 ? 255 : i;
    o.textContent = n;
    src.appendChild(o);
  });
  const grid = $("touchGrid");
  for (let i = 0; i < 6; i++) {
    const card = document.createElement("div");
    card.className = "card";
    card.id = `touch-${i}`;
    card.innerHTML = `<h3>${TOUCH_SRC[i]}</h3>
      <div class="muted" id="touch-meta-${i}">等待</div>
      <div class="muted" id="touch-ack-${i}">应答 —</div>
      <div class="heat" id="touch-heat-${i}"></div>`;
    grid.appendChild(card);
  }
}

function buildTelem() {
  const tb = $("telemTable").querySelector("tbody");
  tb.innerHTML = "";
  for (let i = 0; i < SLOT_N; i++) {
    const L = slotLabel(i);
    const tr = document.createElement("tr");
    tr.id = `row-${i}`;
    tr.innerHTML = `<td>${L.bus}</td><td>#${String(i).padStart(2, "0")}</td><td>${L.kind}</td>
      <td>自研 M${L.id}</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>等待</td>`;
    tb.appendChild(tr);
  }
  for (let m = 0; m < MIT_N; m++) {
    const tr = document.createElement("tr");
    tr.id = `row-mit-${m}`;
    tr.innerHTML = `<td>CAN3</td><td>MIT</td><td>旋转</td><td>外置 16/20mm M${m + 1}</td>
      <td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>等待</td>`;
    tb.appendChild(tr);
  }
}

async function api(path, body) {
  const opt = body === undefined ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  };
  const r = await fetch(path, opt);
  if (!r.ok) throw new Error(`${path} HTTP ${r.status}`);
  return r.json();
}

async function scanEth() {
  const btn = $("btnEthScan");
  if (btn) btn.disabled = true;
  const prev = btn ? btn.textContent : "";
  if (btn) btn.textContent = "扫描中";
  state.scanning = true;
  try {
    const port = +$("ethPort").value || 5001;
    const j = await api("/api/eth-hosts?port=" + port);
    const hosts = j.hosts || [];
    const list = $("ethHostList");
    if (list) {
      list.innerHTML = "";
      hosts.forEach((h) => {
        const o = document.createElement("option");
        o.value = h;
        list.appendChild(o);
      });
    }
    if (!hosts.length) {
      alert("本机网段没有找到 TCP " + port + "。H7 地址由 DHCP 分配，确认已拿到地址，或看串口日志 eth dhcp ip= 后手填。");
      return;
    }
    $("ethHost").value = hosts[0];
    if (hosts.length > 1) alert("找到 " + hosts.length + " 台，已填第一台：\n" + hosts.join("\n"));
  } catch (error) {
    const detail = error && error.message ? error.message : "";
    if (location.protocol === "file:") {
      alert("请先运行 a3_web_bridge.exe，再用浏览器打开 http://127.0.0.1:8080/ 。不要直接双击 index.html。");
    } else {
      alert("扫描请求没有完成" + (detail ? "：" + detail : "") + "。请确认 a3_web_bridge 正在运行，并打开 http://127.0.0.1:8080/ 。");
    }
  } finally {
    state.scanning = false;
    if (btn) {
      btn.disabled = false;
      btn.textContent = prev || "扫描";
    }
  }
}

async function refreshPorts() {
  const sel = $("usbPort");
  if (!sel) return;
  try {
    const j = await api("/api/ports");
    const ports = j.ports || [];
    sel.innerHTML = "";
    if (!ports.length) {
      const o = document.createElement("option");
      o.value = "";
      o.textContent = "未找到串口，点刷新";
      sel.appendChild(o);
      return;
    }
    ports.forEach((p) => {
      const o = document.createElement("option");
      o.value = p.name;
      o.textContent = (p.h7_cdc || p.st_cdc)
        ? `${p.name}  ${p.desc || "H7 USB CDC"}  [H7]`
        : `${p.name}  ${p.desc || ""}`;
      sel.appendChild(o);
    });
    const h7 = ports.find((p) => p.h7_cdc || p.st_cdc);
    sel.value = h7 ? h7.name : ports[0].name;
  } catch (_) {
    sel.innerHTML = "";
    const o = document.createElement("option");
    o.value = "";
    o.textContent = "连不上服务，请运行 a3_web_bridge";
    sel.appendChild(o);
    $("linkState").textContent = "服务未连接";
    $("linkState").className = "chip bad";
  }
}

function liveSelf() {
  const s = state.sine;
  if (!s) return state.self.slice();
  const period = Math.max(0.05, s.period);
  const t = (performance.now() / 1000) - s.t0;
  let k;
  if (s.cyclic) k = 0.5 * (1 - Math.cos((2 * Math.PI * t) / period));
  else if (t >= period) { k = 1; state.sine = null; }
  else k = 0.5 * (1 - Math.cos((Math.PI * t) / period));
  return s.selfAmp.map((a) => a * k);
}

function liveMitPos() {
  const s = state.sine;
  if (!s) return state.mitPos.slice();
  const period = Math.max(0.05, s.period);
  const t = (performance.now() / 1000) - s.t0;
  let k;
  if (s.cyclic) k = 0.5 * (1 - Math.cos((2 * Math.PI * t) / period));
  else if (t >= period) k = 1;
  else k = 0.5 * (1 - Math.cos((Math.PI * t) / period));
  return s.mitAmp.map((a) => a * k);
}

function pushTargets() {
  if (!state.connected) return;
  return api("/api/cmd", {
    type: "set_positions",
    self: liveSelf(),
    mit_pos: liveMitPos(),
    mit_vel: state.mitVel,
    mit_iq: state.mitIq,
  });
}

function heatColor(v, mx) {
  if (v == null || Number.isNaN(v)) return "#475569";
  if (mx <= 0) return "#64748b";
  const t = Math.min(1, v / mx);
  const g = Math.round(230 * (1 - t));
  return `rgb(232,${g},64)`;
}

function drawHeat(canvas, rows, cols, vals) {
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!rows || !cols || !vals || !vals.length) {
    ctx.fillStyle = "#1e293b";
    ctx.fillRect(0, 0, w, h);
    return;
  }
  let mx = 0;
  vals.forEach((v) => { if (v != null && v > mx) mx = v; });
  const cw = w / cols;
  const ch = h / rows;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const v = vals[r * cols + c];
      ctx.fillStyle = heatColor(v, mx);
      ctx.fillRect(c * cw, r * ch, cw + 0.5, ch + 0.5);
    }
  }
}

function renderTouch(touch) {
  (touch || []).forEach((t) => {
    const meta = $(`touch-meta-${t.src}`);
    const ack = $(`touch-ack-${t.src}`);
    const heat = $(`touch-heat-${t.src}`);
    if (!meta) return;
    meta.textContent = `${t.online ? "在线" : "等待"}  seq=${t.seq}  cmd=0x${Number(t.cmd).toString(16)}`;
    ack.textContent = "应答  " + (t.ack || "—");
    heat.innerHTML = "";
    (t.parts || []).forEach((p) => {
      const box = document.createElement("div");
      const name = PART_NAME[p.idx] || "阵列";
      box.innerHTML = `<div class="partlab">${name}  ${p.valid ? "有效" : "无效"}  ${p.rows}x${p.cols}</div>`;
      const cv = document.createElement("canvas");
      cv.className = "heatcv";
      cv.width = t.src === 5 ? 140 : 72;
      cv.height = t.src === 5 ? 140 : 72;
      box.appendChild(cv);
      heat.appendChild(box);
      drawHeat(cv, p.rows, p.cols, p.vals);
    });
  });
}

function renderState(j) {
  state.connected = !!j.connected;
  state.streaming = !!j.streaming;
  $("btnConnect").disabled = state.connected;
  $("btnDisconnect").disabled = !state.connected;
  $("linkState").textContent = j.connected ? `在线  ${j.link || ""}` : "未连接";
  $("linkState").className = "chip " + (j.connected ? "ok" : "wait");
  $("hzState").textContent = `TX ${Number(j.tx_hz || 0).toFixed(0)} Hz · RX ${Number(j.rx_hz || 0).toFixed(0)} Hz`;
  $("btnStream").textContent = state.streaming ? "停止连续" : "开始连续";
  if (j.board) {
    $("boardChips").innerHTML =
      `<span class="chip">NTC_C ${j.board.ntc_c}</span>` +
      `<span class="chip">NTC_B ${j.board.ntc_b}</span>` +
      `<span class="chip">VIN24 ${j.board.vin24}</span>`;
  }
  (j.self || []).forEach((s) => {
    const tr = $(`row-${s.slot}`);
    if (!tr) return;
    const td = tr.querySelectorAll("td");
    td[4].textContent = s.home;
    td[5].textContent = `${Number(s.pos).toFixed(3)} ${s.unit}`;
    td[6].textContent = `${Number(s.pref).toFixed(3)} ${s.unit}`;
    td[7].textContent = Number(s.iq).toFixed(3) + " A";
    td[8].textContent = Number(s.iqref).toFixed(3) + " A";
    td[9].textContent = s.grav;
    td[10].textContent = s.temp;
    td[11].textContent = s.prot || "—";
    td[12].textContent = s.online ? "在线" : "超时";
    tr.className = s.online ? "online" : "stale";
  });
  (j.mit || []).forEach((m) => {
    const tr = $(`row-mit-${m.idx}`);
    if (!tr) return;
    const td = tr.querySelectorAll("td");
    td[4].textContent = m.valid ? `${m.state} / ${m.calib}` : "—";
    td[5].textContent = m.valid ? `${Number(m.pos).toFixed(3)} deg` : "—";
    td[6].textContent = m.valid ? `${Number(m.vel).toFixed(1)} rpm` : "—";
    td[7].textContent = m.valid ? `${Number(m.iq).toFixed(3)} A` : "—";
    td[8].textContent = m.valid ? `${Number(m.vbus).toFixed(1)} V` : "—";
    td[9].textContent = m.mode || "—";
    td[10].textContent = m.type || "—";
    td[11].textContent = "—";
    td[12].textContent = m.faults || (m.online ? "在线" : "等待");
    tr.className = m.online ? "online" : "stale";
  });
  if (j.logs) $("log").textContent = j.logs.join("\n");
  renderTouch(j.touch);
}

function spSlot() { return +$("spSlot").value; }
function spScope() { return $("spScope").value; }

async function poll() {
  if (state.scanning) return;
  try {
    const j = await api("/api/state");
    renderState(j);
  } catch (_) {
    $("linkState").textContent = "服务未连接";
    $("linkState").className = "chip bad";
  }
  if (state.sine && state.streaming) pushTargets();
}

function showTab(name) {
  document.querySelectorAll(".tab").forEach((x) => x.classList.toggle("on", x.dataset.tab === name));
  document.querySelectorAll(".panel").forEach((x) => x.classList.toggle("on", x.id === `tab-${name}`));
}

function bind() {
  document.querySelectorAll(".tab").forEach((b) => {
    b.onclick = () => {
      showTab(b.dataset.tab);
      if (history.replaceState) history.replaceState(null, "", `#${b.dataset.tab}`);
    };
  });
  const initial = (location.hash || "#down").replace("#", "");
  if (document.querySelector(`.tab[data-tab="${initial}"]`)) showTab(initial);
  $("linkKind").onchange = () => {
    const eth = $("linkKind").value === "eth";
    $("usbFields").hidden = eth;
    $("ethFields").hidden = !eth;
  };
  $("linkKind").onchange();
  $("btnPorts").onclick = refreshPorts;
  const scanBtn = $("btnEthScan");
  if (scanBtn) scanBtn.onclick = scanEth;
  $("btnConnect").onclick = async () => {
    const kind = $("linkKind").value;
    const port = ($("usbPort").value || "").trim();
    const ethHost = ($("ethHost").value || "").trim();
    if (kind === "eth" && !ethHost) {
      alert("请先扫描或填写 IP");
      return;
    }
    if (kind !== "eth" && !port) {
      alert("请选择或填写串口（如 COM7）");
      return;
    }
    const body = kind === "eth"
      ? { type: "connect", link: "eth", host: ethHost, port: +$("ethPort").value }
      : { type: "connect", link: "usb", port };
    try {
      const j = await api("/api/cmd", body);
      if (!j.ok) alert(j.error || "连接失败");
    } catch (_) {
      alert("连不上 a3_web_bridge。请运行 build\\Release\\a3_web_bridge.exe 后访问 http://127.0.0.1:8080/");
    }
  };
  $("btnDisconnect").onclick = () => api("/api/cmd", { type: "disconnect" });
  $("btnOnce").onclick = async () => {
    try {
      const targets = await pushTargets();
      if (targets && !targets.ok) { alert(targets.error); return; }
      const result = await api("/api/cmd", { type: "send_once" });
      if (!result.ok) alert(result.error);
    } catch (error) { alert(error.message); }
  };
  $("btnStream").onclick = async () => {
    if ($("wave").value === "正弦" && !state.streaming) {
      state.sine = {
        t0: performance.now() / 1000,
        period: +$("sinePeriod").value || 2,
        cyclic: $("sineMode").value !== "单次",
        selfAmp: state.self.slice(),
        mitAmp: state.mitPos.slice(),
      };
    } else if (state.streaming) {
      state.sine = null;
    }
    try {
      if (!state.streaming) {
        const targets = await pushTargets();
        if (targets && !targets.ok) { state.sine = null; alert(targets.error); return; }
      }
      const result = await api("/api/cmd", { type: "stream", on: state.streaming ? 0 : 1, hz: +$("rate").value });
      if (!result.ok) { state.sine = null; alert(result.error); }
    } catch (error) { state.sine = null; alert(error.message); }
  };
  $("btnZero").onclick = () => {
    state.self = Array(SLOT_N).fill(0);
    state.mitPos = [0, 0];
    state.mitVel = [0, 0];
    state.mitIq = [0, 0];
    state.sine = null;
    $("motorCards").querySelectorAll("input").forEach((el) => { el.value = 0; });
    api("/api/cmd", { type: "zero" });
  };
  document.querySelectorAll("[data-en]").forEach((b) => {
    b.onclick = () => api("/api/cmd", {
      type: "enable", slot: spSlot(), scope: spScope(), on: +b.dataset.en, drv: +$("spDrv").value,
    });
  });
  $("btnHome").onclick = () => api("/api/cmd", {
    type: "home", slot: spSlot(), scope: spScope(), iq_seek: +$("iqSeek").value,
  });
  $("btnPosPid").onclick = () => api("/api/cmd", {
    type: "pid_pos", slot: spSlot(), kp: +$("pkp").value, ki: +$("pki").value, kd: +$("pkd").value,
  });
  $("btnCurPid").onclick = () => api("/api/cmd", {
    type: "pid_cur", slot: spSlot(),
    id_kp: +$("idkp").value, id_ki: +$("idki").value, id_kd: +$("idkd").value,
    iq_kp: +$("iqkp").value, iq_ki: +$("iqki").value, iq_kd: +$("iqkd").value,
  });
  $("btnSpinOn").onclick = () => api("/api/cmd", {
    type: "open_spin", slot: spSlot(), start: 1, freq_hz: +$("spinHz").value, half_s: +$("spinHalf").value,
  });
  $("btnSpinOff").onclick = () => api("/api/cmd", { type: "open_spin", slot: spSlot(), start: 0 });
  $("touchInfo").onclick = () => api("/api/cmd", { type: "touch", src: +$("touchSrc").value, cmd: 0x02, data: [] });
  $("touchSetMode").onclick = () => api("/api/cmd", {
    type: "touch", src: +$("touchSrc").value, cmd: 0x11, data: [+$("touchMode").value],
  });
  $("touchSetFreq").onclick = () => {
    const hz = +$("touchFreq").value;
    api("/api/cmd", { type: "touch", src: +$("touchSrc").value, cmd: 0x14, data: [hz & 255, (hz >> 8) & 255] });
  };
}

buildMotors();
buildSp();
buildTelem();
buildTouch();
bind();
refreshPorts();
setInterval(poll, 200);
poll();
