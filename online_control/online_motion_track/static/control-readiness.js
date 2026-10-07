import {isA30Model, isA3Model} from "../../shared/hand-models.js";

// Collision correction is optional on A3; V2 retains its existing arm policy.
export function collisionProtectionRequired(model, mode) {
  return !isA3Model(model) || !["off", "monitor"].includes(mode);
}

export function collisionProtectionLabel(mode) {
  return mode === "off" ? "碰撞保护关闭" : mode === "monitor" ? "仅检测 · 不修正" : "碰撞保护开启";
}
// Keep every disabled-button condition visible, including server errors.
export function armBlockers({allowed, connected, homing, busy, active, hardware, telemetryIssue, handAgeMs, side, mode, requireHandConfirmation=true, directMode=false}) {
  const reasons=[];
  if(hardware.mode_confirmed===false)reasons.push("控制模式尚未确认，请重新切换模式");
  if(typeof hardware.direct_mode==='boolean'&&hardware.direct_mode!==directMode)reasons.push("等待控制模式同步");
  if(!allowed)reasons.push("需要 Chrome / Edge 和可信 HTTPS 或 localhost");
  if(!connected)reasons.push("请连接本机串口");
  if(homing)reasons.push("正在找零");
  if(busy||hardware.power_busy)reasons.push("等待当前控制操作完成");
  if(!active)reasons.push("请先开始手势识别");
  if(connected&&telemetryIssue)reasons.push(telemetryIssue);
  if(active&&requireHandConfirmation&&handAgeMs>400)reasons.push(`未检测到最近 400 ms 内的有效${side==="right"?"右":"左"}手手势`);
  // A latched A3 fault can only be cleared by the arm action's validated
  // feedback rebase. Do not disable that action waiting for a camera target.
  const canRebase=isA30Model(hardware.model_id)&&hardware.rebase_required===true;
  if(active&&!hardware.target_ready&&!canRebase)reasons.push(`映射目标未就绪${hardware.last_error?`：${hardware.last_error}`:requireHandConfirmation?"（等待连续有效手势）":"（等待保持姿态就绪）"}`);
  if(!isA3Model(hardware.model_id)&&["off","monitor"].includes(mode))reasons.push("请选择 Mink、避碰规划或碰撞停止模式");
  if(active&&hardware.ruckig_enabled===false&&!directMode&&!hardware.direct_mode)reasons.push("Ruckig 尚未就绪");
  if(active&&!['off','monitor'].includes(mode)&&hardware.sample_collision_ready===false)reasons.push("采样碰撞检查尚未就绪");
  return reasons;
}
