"""会话级网页报告：目标下发 vs 遥测实际 对比（角度/误差/电流/温度）。

使用方式：
  1) 程序启动时创建 SessionWebReportLogger(enabled=True/False)；
  2) 每次下发调用 record_command(hw_cfg, hand_side, source=...)；
  3) 周期读取遥测后调用 record_telemetry(frame, hand_side=...)；
  4) 退出时调用 close() 自动写出 HTML。
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from protocol import convert_hardware_to_ui
from usb.offline import (
    LINEAR_JOINT_SLOTS,
    ROTARY_JOINT_SLOTS,
    joint_cfg_from_telemetry_frame,
)


def _to_bool(v: object) -> bool:
    s = str(v).strip().lower()
    return s in ("1", "true", "yes", "y", "on")


def _finger_from_joint(joint: str) -> Optional[str]:
    s = str(joint)
    if "_hand_" not in s:
        return None
    tail = s.split("_hand_", 1)[1]
    return tail.split("_", 1)[0]


@dataclass
class SessionWebReportConfig:
    enabled: bool = False
    out_dir: str = "logs"
    filename: str = ""
    max_points: int = 30000


class SessionWebReportLogger:
    def __init__(self, cfg: SessionWebReportConfig) -> None:
        self.cfg = cfg
        self._t0 = time.perf_counter()
        self._closed = False
        self._commands: List[dict] = []
        self._telemetry: List[dict] = []
        self._finger_stats: List[dict] = []
        self._last_command_ui_by_joint: Dict[str, float] = {}
        self._all_joints: set[str] = set()
        self._all_fingers = ("thumb", "index", "mid", "ring", "pinky")
        self.output_path: Optional[str] = None

    @property
    def enabled(self) -> bool:
        return bool(self.cfg.enabled)

    def _now_s(self) -> float:
        return time.perf_counter() - self._t0

    def record_command(
        self,
        hw_cfg: dict,
        *,
        hand_side: str = "left",
        source: str = "unknown",
    ) -> None:
        if not self.enabled or not hw_cfg:
            return
        ui_cfg = convert_hardware_to_ui(hw_cfg, hand_side=hand_side)
        t = self._now_s()
        for k, v in ui_cfg.items():
            self._last_command_ui_by_joint[str(k)] = float(v)
            self._all_joints.add(str(k))
        self._commands.append(
            {
                "t": round(t, 6),
                "source": str(source),
                "hand_side": str(hand_side),
                "ui_cfg": {str(k): float(v) for k, v in ui_cfg.items()},
            }
        )
        if len(self._commands) > self.cfg.max_points:
            del self._commands[: len(self._commands) - self.cfg.max_points]

    def record_telemetry(self, frame: dict, *, hand_side: str = "left") -> None:
        if not self.enabled or not frame:
            return
        t = self._now_s()
        actual_ui = joint_cfg_from_telemetry_frame(frame, use_ref=False, hand_side=hand_side)
        for k in actual_ui.keys():
            self._all_joints.add(str(k))
        point = {"t": round(t, 6), "hand_side": str(hand_side), "actual_ui": {}}
        for joint, val in actual_ui.items():
            joint = str(joint)
            actual = float(val)
            target = float(self._last_command_ui_by_joint.get(joint, actual))
            point["actual_ui"][joint] = actual
            point.setdefault("target_ui", {})[joint] = target
            point.setdefault("err_ui", {})[joint] = actual - target
        self._telemetry.append(point)
        if len(self._telemetry) > self.cfg.max_points:
            del self._telemetry[: len(self._telemetry) - self.cfg.max_points]

        linear = frame.get("linear") if isinstance(frame, dict) else None
        rotary = frame.get("rotary") if isinstance(frame, dict) else None
        if isinstance(linear, list) and isinstance(rotary, list):
            stat = {"t": round(t, 6), "hand_side": str(hand_side), "finger": {}}
            for finger in self._all_fingers:
                cur_vals: List[float] = []
                temp_vals: List[float] = []

                for j, slot in LINEAR_JOINT_SLOTS.items():
                    if _finger_from_joint(j) != finger or slot >= len(linear):
                        continue
                    obj = linear[slot]
                    iq = getattr(obj, "iq_meas", None)
                    temp = getattr(obj, "mcu_temp_c", None)
                    if isinstance(iq, (int, float)):
                        cur_vals.append(float(iq))
                    if isinstance(temp, (int, float)):
                        temp_vals.append(float(temp))

                for j, slot in ROTARY_JOINT_SLOTS.items():
                    if _finger_from_joint(j) != finger or slot >= len(rotary):
                        continue
                    obj = rotary[slot]
                    iq = getattr(obj, "iq", None)
                    if isinstance(iq, (int, float)):
                        cur_vals.append(float(iq))

                if cur_vals or temp_vals:
                    stat["finger"][finger] = {
                        "current": float(sum(cur_vals) / len(cur_vals)) if cur_vals else 0.0,
                        "temp_c": float(sum(temp_vals) / len(temp_vals)) if temp_vals else 0.0,
                    }
            self._finger_stats.append(stat)
            if len(self._finger_stats) > self.cfg.max_points:
                del self._finger_stats[: len(self._finger_stats) - self.cfg.max_points]

    def _build_html(self) -> str:
        joints = sorted(self._all_joints)
        payload = {
            "joints": joints,
            "telemetry": self._telemetry,
            "finger_stats": self._finger_stats,
            "commands_count": len(self._commands),
            "telemetry_count": len(self._telemetry),
        }
        payload_js = json.dumps(payload, ensure_ascii=False)
        return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Hand Downlink vs Telemetry Report</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 16px; }}
    .row {{ display:flex; gap:16px; align-items:center; flex-wrap: wrap; margin-bottom: 12px; }}
    .card {{ border:1px solid #ddd; border-radius:8px; padding:12px; margin-bottom: 14px; }}
    canvas {{ border:1px solid #ccc; width: 100%; height: 280px; }}
    table {{ border-collapse: collapse; width:100%; font-size:12px; }}
    th, td {{ border: 1px solid #ddd; padding: 4px 6px; text-align: right; }}
    th:first-child, td:first-child {{ text-align: center; }}
    .muted {{ color:#666; font-size: 12px; }}
  </style>
</head>
<body>
  <h2>下发目标 vs 遥测实际 对比报告</h2>
  <div class="muted">记录点：downlink=<span id="cntCmd"></span>，telemetry=<span id="cntTel"></span></div>

  <div class="card">
    <div class="row">
      <label>关节选择：
        <select id="jointSel"></select>
      </label>
      <span id="jointStats" class="muted"></span>
    </div>
    <canvas id="jointCanvas" width="1200" height="320"></canvas>
    <div style="max-height:260px; overflow:auto; margin-top:10px;">
      <table id="jointTable">
        <thead><tr><th>#</th><th>t(s)</th><th>target(rad)</th><th>actual(rad)</th><th>error(rad)</th></tr></thead>
        <tbody></tbody>
      </table>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <label>手指选择：
        <select id="fingerSel">
          <option value="thumb">thumb</option>
          <option value="index">index</option>
          <option value="mid">mid</option>
          <option value="ring">ring</option>
          <option value="pinky">pinky</option>
        </select>
      </label>
      <span class="muted">current: A（平均iq） / temp: °C（linear槽温度均值）</span>
    </div>
    <canvas id="fingerCanvas" width="1200" height="280"></canvas>
  </div>

  <script>
    const DATA = {payload_js};
    const jointSel = document.getElementById('jointSel');
    const fingerSel = document.getElementById('fingerSel');
    const jointTableBody = document.querySelector('#jointTable tbody');
    const jointStats = document.getElementById('jointStats');
    document.getElementById('cntCmd').textContent = DATA.commands_count;
    document.getElementById('cntTel').textContent = DATA.telemetry_count;

    for (const j of DATA.joints) {{
      const op = document.createElement('option');
      op.value = j; op.textContent = j;
      jointSel.appendChild(op);
    }}

    function drawLines(canvas, series, yPad=0.1) {{
      const ctx = canvas.getContext('2d');
      const W = canvas.width, H = canvas.height;
      ctx.clearRect(0,0,W,H);
      if (!series.length || !series[0].points.length) return;
      let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
      for (const s of series) for (const p of s.points) {{
        xmin = Math.min(xmin, p.x); xmax = Math.max(xmax, p.x);
        ymin = Math.min(ymin, p.y); ymax = Math.max(ymax, p.y);
      }}
      if (!isFinite(xmin) || !isFinite(ymin)) return;
      if (xmax <= xmin) xmax = xmin + 1e-6;
      if (ymax <= ymin) ymax = ymin + 1e-6;
      const dy = (ymax - ymin) * yPad; ymin -= dy; ymax += dy;

      const L=60,R=20,T=20,B=35;
      const sx = x => L + (x-xmin)/(xmax-xmin)*(W-L-R);
      const sy = y => H-B - (y-ymin)/(ymax-ymin)*(H-T-B);

      ctx.strokeStyle = '#bbb'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(L,H-B); ctx.lineTo(W-R,H-B); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(L,T); ctx.lineTo(L,H-B); ctx.stroke();
      ctx.fillStyle='#333'; ctx.font='12px Arial';
      ctx.fillText(`t: ${{xmin.toFixed(2)}} ~ ${{xmax.toFixed(2)}} s`, L, H-8);
      ctx.fillText(`y: ${{ymin.toFixed(3)}} ~ ${{ymax.toFixed(3)}}`, L+180, H-8);

      for (const s of series) {{
        if (!s.points.length) continue;
        ctx.strokeStyle = s.color; ctx.lineWidth = 1.6;
        ctx.beginPath();
        s.points.forEach((p,idx)=>{{ idx?ctx.lineTo(sx(p.x),sy(p.y)):ctx.moveTo(sx(p.x),sy(p.y)); }});
        ctx.stroke();
      }}
      let lx=W-250, ly=24;
      for (const s of series) {{
        ctx.fillStyle = s.color; ctx.fillRect(lx, ly-9, 10, 10);
        ctx.fillStyle = '#222'; ctx.fillText(s.name, lx+14, ly);
        ly += 16;
      }}
    }}

    function refreshJoint() {{
      const j = jointSel.value || DATA.joints[0];
      const rows = [];
      for (const p of DATA.telemetry) {{
        const t = p.t;
        const target = p.target_ui?.[j];
        const actual = p.actual_ui?.[j];
        const err = p.err_ui?.[j];
        if ([target, actual, err].every(v => typeof v === 'number')) {{
          rows.push({{t, target, actual, err}});
        }}
      }}
      const errAbsAvg = rows.length ? rows.reduce((a,r)=>a+Math.abs(r.err),0)/rows.length : 0;
      const errAbsMax = rows.length ? Math.max(...rows.map(r=>Math.abs(r.err))) : 0;
      jointStats.textContent = `samples=${{rows.length}}, mean|err|=${{errAbsAvg.toFixed(4)}} rad, max|err|=${{errAbsMax.toFixed(4)}} rad`;

      drawLines(document.getElementById('jointCanvas'), [
        {{name:'target', color:'#1f77b4', points: rows.map(r=>({{x:r.t,y:r.target}}))}},
        {{name:'actual', color:'#2ca02c', points: rows.map(r=>({{x:r.t,y:r.actual}}))}},
        {{name:'error',  color:'#d62728', points: rows.map(r=>({{x:r.t,y:r.err}}))}},
      ], 0.15);

      jointTableBody.innerHTML = '';
      const tail = rows.slice(-500);
      tail.forEach((r,idx) => {{
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${{idx+1}}</td><td>${{r.t.toFixed(3)}}</td><td>${{r.target.toFixed(4)}}</td><td>${{r.actual.toFixed(4)}}</td><td>${{r.err.toFixed(4)}}</td>`;
        jointTableBody.appendChild(tr);
      }});
    }}

    function refreshFinger() {{
      const f = fingerSel.value;
      const rows = [];
      for (const p of DATA.finger_stats) {{
        const v = p.finger?.[f];
        if (v && typeof v.current === 'number' && typeof v.temp_c === 'number') {{
          rows.push({{t:p.t, current:v.current, temp:v.temp_c}});
        }}
      }}
      drawLines(document.getElementById('fingerCanvas'), [
        {{name:'current(A)', color:'#9467bd', points: rows.map(r=>({{x:r.t,y:r.current}}))}},
        {{name:'temp(C)', color:'#ff7f0e', points: rows.map(r=>({{x:r.t,y:r.temp}}))}},
      ], 0.2);
    }}

    jointSel.addEventListener('change', refreshJoint);
    fingerSel.addEventListener('change', refreshFinger);
    if (DATA.joints.length) jointSel.value = DATA.joints[0];
    refreshJoint();
    refreshFinger();
  </script>
</body>
</html>
"""

    def close(self) -> Optional[str]:
        if self._closed or not self.enabled:
            return self.output_path
        self._closed = True
        if not self.cfg.filename:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.cfg.filename = f"downlink_telemetry_report_{ts}.html"
        out_dir = self.cfg.out_dir or "logs"
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, self.cfg.filename)
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            f.write(self._build_html())
        self.output_path = out
        return out


def build_report_logger_from_argv(argv: list[str]) -> SessionWebReportLogger:
    enabled = False
    out_dir = "logs"
    filename = ""
    i = 0
    while i < len(argv):
        tok = str(argv[i]).strip()
        if tok == "--log_file_write" and i + 1 < len(argv):
            enabled = _to_bool(argv[i + 1])
            i += 2
            continue
        if tok.startswith("--log_file_write="):
            enabled = _to_bool(tok.split("=", 1)[1])
            i += 1
            continue
        if tok == "--log_report_dir" and i + 1 < len(argv):
            out_dir = str(argv[i + 1])
            i += 2
            continue
        if tok.startswith("--log_report_dir="):
            out_dir = tok.split("=", 1)[1]
            i += 1
            continue
        if tok == "--log_report_name" and i + 1 < len(argv):
            filename = str(argv[i + 1])
            i += 2
            continue
        if tok.startswith("--log_report_name="):
            filename = tok.split("=", 1)[1]
            i += 1
            continue
        i += 1
    return SessionWebReportLogger(
        SessionWebReportConfig(enabled=enabled, out_dir=out_dir, filename=filename)
    )

