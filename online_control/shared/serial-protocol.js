// H7 wire format, matching src/usb/offline.py and usb/telemetry.py.
const LINEAR_JOINTS = ["ring_MCP", "ring_PIP", "ring_DIP", "pinky_MCP", "pinky_PIP", "pinky_DIP", "index_MCP", "index_PIP", "index_DIP", "mid_MCP", "mid_PIP", "mid_DIP", "thumb_MCP", "thumb_PIP"];
const ROTARY_JOINTS = ["index_swing", "mid_swing", "ring_swing", "pinky_swing", "thumb_rota", "thumb_swing", "pinky_rota"];
export const slotMaps = side => Object.fromEntries([["linear", LINEAR_JOINTS], ["rotary", ROTARY_JOINTS]].map(([kind, names]) =>
  [kind, Object.fromEntries(names.map((name, i) => [`${side}_hand_${name}_joint`, i]))]));
export const positionSlots = side => Object.fromEntries(Object.entries(slotMaps(side)).flatMap(([kind, slots]) =>
  Object.entries(slots).map(([name, i]) => [name, (kind === "linear" ? 7 : 63) + i*4])));
export const powerChannels = side => Object.fromEntries([
  ...LINEAR_JOINTS.map((n, i) => [`${side}_hand_${n}_joint`, {kind:"linear", bus:Math.floor(i/6), motor:i%6+1}]),
  ...ROTARY_JOINTS.map((n, i) => [`${side}_hand_${n}_joint`, {kind:"rotary", bus:[0,0,1,1,3,3,2][i], motor:i%2}]),
]);
export const linearPower = (bus, motor, enabled) => frame(0x11, new Uint8Array([bus,motor,enabled ? 1 : 2,0]));
export function holdTelemetry(seq, t) {
  return positions(seq, t.linear.map(s => s.pos_act_mm), t.rotary.map(s => s.pos_actual_mm));
}
export const hex = bytes => Array.from(bytes, b => b.toString(16).padStart(2, "0")).join("");
export function crc16(bytes) {
  let crc = 0xffff;
  for (const b of bytes) {
    crc ^= b;
    for (let i = 0; i < 8; i++) crc = crc & 1 ? (crc >>> 1) ^ 0xa001 : crc >>> 1;
  }
  return crc;
}
export function frame(fun, payload) {
  const out = new Uint8Array(payload.length + 8), v = new DataView(out.buffer);
  out.set([0xaa, 0x55]); v.setUint16(2, payload.length, true);
  out[4] = 3; out[5] = fun; out.set(payload, 6);
  v.setUint16(out.length - 2, crc16(out.subarray(0, -2)), true);
  return out;
}
export const handType = side => frame(0x2b, new Uint8Array([1, side === "right" ? 1 : 0, 0, 0]));
export const rotaryDrive = (bus, a, b) => frame(0x22, new Uint8Array([bus, a, b, 0]));
export const rotaryZero = (bus, motor) => frame(0x25, new Uint8Array([bus, motor === 0 ? 1 : 0, motor === 1 ? 1 : 0, 0]));
export function rotaryPosition(bus, motor, degrees) {
  if (!Number.isFinite(degrees)) throw new Error("旋转保持位置无效");
  const payload = new Uint8Array(12), view = new DataView(payload.buffer);
  payload[0] = bus; payload[1] = 1 << motor;
  view.setFloat32(4 + motor * 4, degrees, true);
  return frame(0x21, payload);
}
export function linearHome(bus, motor, cmd, iq) {
  const p = new Uint8Array([bus, motor, cmd, 0, 0, 0, 0, 0]);
  new DataView(p.buffer).setFloat32(4, iq, true);
  return frame(0x12, p);
}
export function positions(sequence, linear, rotary) {
  if (linear.length !== 14 || rotary.length !== 7 || ![...linear, ...rotary].every(Number.isFinite))
    throw new Error("下发数据必须为 14 路直线和 7 路旋转有限数值");
  const p = new Uint8Array(88), v = new DataView(p.buffer); p[0] = sequence & 255;
  [...linear, ...rotary].forEach((value, i) => v.setFloat32(1 + i * 4, value, true));
  // UI -> HW signs have already been applied by the server. Never negate twice.
  return frame(0x20, p);
}
export function decodePositions(bytes) {
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return {wire_seq: bytes[6], linear_mm: Array.from({length: 14}, (_, i) => v.getFloat32(7 + 4*i, true)),
    rotary_deg: Array.from({length: 7}, (_, i) => v.getFloat32(63 + 4*i, true))};
}
export class UplinkParser {
  constructor() { this.buffer = new Uint8Array(); }
  push(chunk) {
    const next = new Uint8Array(this.buffer.length + chunk.length);
    next.set(this.buffer); next.set(chunk, this.buffer.length); this.buffer = next;
    const packets = []; let i = 0;
    while (i + 8 <= next.length) {
      if (next[i] !== 0x55 || next[i+1] !== 0xaa) { i++; continue; }
      const length = next[i+2] | next[i+3] << 8, end = i + length + 8;
      if (![16, 780, 932].includes(length)) { i++; continue; }
      if (end > next.length) break;
      const bytes = next.subarray(i, end);
      if (crc16(bytes.subarray(0, -2)) !== (bytes.at(-2) | bytes.at(-1) << 8)) { i++; continue; }
      if (bytes[4] === 3 && bytes[5] === 2) packets.push(bytes.slice(6, -2));
      i = end;
    }
    this.buffer = next.slice(i);
    return packets;
  }
}
export function telemetry(payload) {
  if (![780, 932].includes(payload.length)) return null;
  const v = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const f = o => v.getFloat32(o, true), u = o => v.getUint32(o, true);
  const mm = o => f(o) === 0 || Math.abs(f(o)) >= 1e-6 ? f(o) : v.getInt32(o, true)*.001;
  const rotaryNames = ["iq", "id_", "iq_ref", "id_ref", "pos_ref_mm", "pos_actual_mm", "pos_err_mm", "vq_cmd", "force_n", "elec_angle_rad"];
  const rot = o => Object.assign({home_label: payload[o]}, Object.fromEntries(rotaryNames.map((name, i) => [name, f(o+4+i*4)])));
  const linear = [], rotary = [], modern = payload.length === 780;
  let off = 8;
  for (let i = 0; i < 14; i++, off += modern ? 32 : 44) {
    let row;
    if (modern) {
      const b = payload.subarray(off, off+4);
      let home = 255, drv = b[1], elec = f(off);
      if (b[0] === 0 && b[1] === 0 && b[2] === 0 && b[3] <= 3) { home = b[3]; drv = 0; elec = 0; }
      else if (b[0] <= 3 && b[2] === 0 && b[3] === 0) { home = b[0]; elec = 0; }
      const lo = v.getInt16(off+28, true), hi = v.getInt16(off+30, true);
      const plausible = x => Math.abs(x) >= 500 && Math.abs(x) <= 12000;
      const high = plausible(hi) && !plausible(lo) || !(plausible(lo) && !plausible(hi)) && lo === 0 && hi !== 0;
      const temp = high ? hi : lo;
      row = {mech_home_state: home, drv_auto_recover: drv, elec_angle_rad: elec,
        pos_act_mm: mm(off+4), iq_meas: f(off+8), iq_ref: f(off+12), pos_ref_mm: mm(off+16),
        pos_err_mm: mm(off+16)-mm(off+4), grav_iq: f(off+20), status_b_u32: u(off+24),
        can_rx_error_cnt: payload[off+(high ? 28 : 30)], can_tx_error_cnt: payload[off+(high ? 29 : 31)],
        mcu_temp_centi: temp, mcu_temp_c: Math.abs(temp) >= 500 && Math.abs(temp) <= 8500 ? temp/100 : null};
    } else {
      const r = rot(off);
      row = {mech_home_state: r.home_label, drv_auto_recover: 0, pos_act_mm: r.pos_actual_mm,
        iq_meas: r.iq, iq_ref: r.iq_ref, pos_ref_mm: r.pos_ref_mm, pos_err_mm: r.pos_err_mm,
        grav_iq: 0, elec_angle_rad: r.elec_angle_rad, id_meas: r.id_, id_ref: r.id_ref, vq_out: r.vq_cmd};
    }
    row.active = [row.iq_meas, row.iq_ref, row.pos_act_mm, row.pos_ref_mm, row.mech_home_state].some(x => Math.abs(x) > 1e-6);
    linear.push(row);
  }
  const hand_type = modern ? payload[off+1] : null;
  for (let i = 0; i < 7; i++, off += 44) {
    const row = rot(off); row.active = Object.values(row).some(x => Math.abs(x) > 1e-6); rotary.push(row);
  }
  const board = {seq: v.getUint16(0, true), tick_ms: u(4), hand_type, fmt: modern ? "v2" : "legacy"};
  if (modern) Object.assign(board, {imu_ok_cnt: payload[2], imu_hdr_cnt: payload[3],
    imu_valid: v.getUint16(off, true), imu_flag: v.getUint16(off+2, true),
    imu_roll_deg: f(off+4), imu_pitch_deg: f(off+8), imu_yaw_deg: f(off+12)});
  return {telemetry_seq: board.seq, tick_ms: board.tick_ms, hand_side: hand_type === null ? null : hand_type === 1 ? "right" : "left", linear, rotary, board};
}
