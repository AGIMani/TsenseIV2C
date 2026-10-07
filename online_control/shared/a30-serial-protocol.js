// A3_USB_Downlink.pyw: AA55, 444/484-byte uplink, 108-byte command. Never V2 opcodes.
import {frame, crc16} from "./serial-protocol.js";
import {SELF, EXTERNAL, MIT_GEAR_RATIOS, ROTARY, selfFeedbackValid, externalFeedbackValid} from "./a30-telemetry.js";
export {SELF, EXTERNAL, MIT_GEAR_RATIOS, ROTARY, selfFeedbackValid, externalFeedbackValid, telemetry, boardTelemetry} from "./a30-telemetry.js";
export {TELEMETRY_FORMAT, telemetryMetadata} from "./a30-telemetry.js";
export const positionSlots = side => Object.fromEntries([
  ...SELF.map((name, i) => [`${side}_hand_${name}_joint`, 7 + i*4]),
  ...EXTERNAL.map((name, i) => [`${side}_hand_${name}`, 79 + i*4]),
]);
// Geometry <-> wire: all nine linear actuators have the opposite direction.
export const selfPositionToGeometry = (slot, value) => ROTARY.has(slot) ? value : -value;
export const LINEAR_NAMES=SELF.filter((_,i)=>!ROTARY.has(i)).map(n=>n+"_joint");
export const ROTARY_NAMES=[...SELF.filter((_,i)=>ROTARY.has(i)).map(n=>n+"_joint"),...EXTERNAL];
export const slotMaps=side=>Object.fromEntries([["linear",LINEAR_NAMES],["rotary",ROTARY_NAMES]].map(([kind,names])=>
  [kind,Object.fromEntries(names.map((n,i)=>[`${side}_hand_${n}`,i]))]));
export const address = slot => slot < 8 ? [0,slot+1] : slot < 16 ? [1,slot-7] : [2,slot-15];
export const powerChannels = side => Object.fromEntries(SELF.map((name, slot) => {
  const [bus, motor] = address(slot);
  return [`${side}_hand_${name}_joint`, {kind:"self", bus, motor}];
}));
function finite(value) {
  if (!Number.isFinite(value) || !Number.isFinite(Math.fround(value))) throw new Error("A3 数值无效或超出 float32");
  return value;
}
function target(bus,node) {
  if (!Number.isInteger(bus) || !Number.isInteger(node) || bus < 0 || bus > 2 || node < 1 || node > (bus === 2 ? 2 : 8)) throw new Error("A3 CAN 通道无效");
}
export function power(bus,node,cmd) {target(bus,node); if (![1,2].includes(cmd)) throw new Error("A3 使能命令无效"); return frame(0x21,new Uint8Array([bus,node,cmd,3]));}
export function home(bus,node,cmd,iq) {
  target(bus,node); if (![1,2].includes(cmd) || !Number.isFinite(iq) || iq < 0 || iq > 5000) throw new Error("A3 找零参数无效");
  const p=new Uint8Array(12); p.set([bus,node,cmd,0]); new DataView(p.buffer).setFloat32(4,iq,true); return frame(0x22,p);
}
export function positions(seq,linear,rotary,side) {
  if (!["left","right"].includes(side)) throw new Error("A3 手别无效");
  const names=SELF.map(n=>`${side}_hand_${n}_joint`), ext=EXTERNAL.map(n=>`${side}_hand_${n}`);
  const expectedLinear=names.filter((_,i)=>!ROTARY.has(i)), expectedRotary=[...names.filter((_,i)=>ROTARY.has(i)),...ext];
  if (!linear || !rotary || Object.keys(linear).length!==9 || Object.keys(rotary).length!==11 ||
      !expectedLinear.every(n=>Object.hasOwn(linear,n)) || !expectedRotary.every(n=>Object.hasOwn(rotary,n))) throw new Error("A3 需要完整 9+11 关节目标");
  const p=new Uint8Array(100),v=new DataView(p.buffer); p[0]=seq&255;
  names.forEach((n,i)=>{
    const value=finite((ROTARY.has(i)?rotary:linear)[n]);
    // Match online_motion_play: linear wire targets are always -abs(stroke).
    v.setFloat32(1+i*4,ROTARY.has(i)?value:value===0?0:-Math.abs(value),true);
  });
  ext.forEach((n,i)=>v.setFloat32(73+i*4,finite(rotary[n]*MIT_GEAR_RATIOS[i]),true));
  // External speed/current FF remain zero, as in Python encode defaults.
  return frame(0x20,p);
}
export function decodePositions(bytes) {
  const v=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  const self=Array.from({length:18},(_,i)=>v.getFloat32(7+i*4,true));
  return {wire_seq:bytes[6], self_pos:self,
    linear_mm:self.filter((_,i)=>!ROTARY.has(i)).map(value=>-value),rotary_deg:[...self.filter((_,i)=>ROTARY.has(i)),v.getFloat32(79,true)/MIT_GEAR_RATIOS[0],v.getFloat32(83,true)/MIT_GEAR_RATIOS[1]],
    mit_pos_deg:[v.getFloat32(79,true),v.getFloat32(83,true)], mit_vel_rpm:[v.getFloat32(87,true),v.getFloat32(91,true)],
    mit_iq_ff_a:[v.getFloat32(95,true),v.getFloat32(99,true)]};
}
export function missingFeedback(t,side){
  return [...SELF.filter((_,i)=>!selfFeedbackValid(t?.self?.[i])).map(n=>`${side}_hand_${n}_joint`),
    ...EXTERNAL.filter((_,i)=>!externalFeedbackValid(t?.external?.[i])).map(n=>`${side}_hand_${n}`)];
}
export function holdTelemetry(seq,t,side,fallback,limits){
  if(!t?.self || !t?.external)throw new Error("请等待 A3 遥测数据包");
  const linear={},rotary={};
  SELF.forEach((n,i)=>{
    const name=`${side}_hand_${n}_joint`,kind=ROTARY.has(i)?"rotary":"linear";
    (kind==="rotary"?rotary:linear)[name]=selfFeedbackValid(t.self[i])?selfPositionToGeometry(i,t.self[i].position):fallback?.[kind]?.[name];
  });
  EXTERNAL.forEach((n,i)=>{
    const name=`${side}_hand_${n}`,s=t.external[i];
    if(s?.online&&(s.fault||s.state===5))throw new Error("外接电机报告故障");
    rotary[name]=externalFeedbackValid(s)?s.position:fallback?.rotary?.[name];
  });
  // Bounds are actuator values converted from the model's joint endpoints.
  // This also covers homing's initial hold, which does not call browser_rebase.
  if(limits)for(const [kind,values] of Object.entries({linear,rotary}))for(const name of Object.keys(values)){
    const range=limits[kind]?.[name];
    if(!range||range.length!==2||!range.every(Number.isFinite)||range[0]>range[1])throw new Error(`缺少 ${name} 的反馈范围`);
    values[name]=Math.min(range[1],Math.max(range[0],finite(values[name])));
  }
  return positions(seq,linear,rotary,side);
}
export class UplinkParser {
  constructor(){this.buffer=new Uint8Array();this.boardPackets=[];}
  drainBoard(){return this.boardPackets.splice(0);}
  push(chunk){
    const b=new Uint8Array(this.buffer.length+chunk.length); b.set(this.buffer); b.set(chunk,this.buffer.length);
    let i=0; const out=[];
    while(i+8<=b.length){
      if(b[i]!==0xaa||b[i+1]!==0x55){i++;continue;}
      const len=b[i+2]|b[i+3]<<8,end=i+len+8;
      if(len===0||len+8>2048){i++;continue;}
      if(end>b.length)break;
      const wire=b.subarray(i,end);
      if(crc16(wire.subarray(0,-2))!==(wire.at(-2)|wire.at(-1)<<8)){i++;continue;}
      if(wire[4]===3){
        if(wire[5]===2&&len>=444){
          const payload=wire.slice(6,6+(len>=484?484:444));
          // Keep framing evidence without duplicating every 484B payload in logs.
          Object.defineProperty(payload,"wire_info",{value:{header_hex:Array.from(wire.subarray(0,6),v=>v.toString(16).padStart(2,"0")).join(""),
            crc16:wire.at(-2)|wire.at(-1)<<8,crc_verified:true,payload_bytes:len,
            extension_hex:Array.from(wire.subarray(6+payload.length,6+len),v=>v.toString(16).padStart(2,"0")).join("")}});
          out.push(payload);
        }
        if(wire[5]===3&&len>=8)this.boardPackets.push(wire.slice(6,14));
      }
      i=end;
    }
    this.buffer=b.slice(i); return out;
  }
}
