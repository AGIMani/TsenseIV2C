// Shared telemetry decoder. Layout follows A3_USB_Downlink.pyw (<HHII>, 18 x <5f2h>, 2 x 20B).
export const SELF = ["pinky_DIP", "pinky_swing", "ring_DIP", "ring_swing", "pinky_PIP", "pinky_MCP", "ring_PIP", "ring_MCP",
  "mid_DIP", "mid_swing", "index_DIP", "index_swing", "mid_PIP", "mid_MCP", "index_PIP", "index_MCP", "thumb_PIP", "thumb_MCP"];
export const EXTERNAL = ["thumb_rotaback_joint2", "thumb_rota_block_joint"];
export const MIT_GEAR_RATIOS = Object.freeze([50, 30]); // ROTABACK / ROTA_BLOCK
export const ROTARY = new Set([0,1,2,3,8,9,10,11,16]);
export const TELEMETRY_FORMAT = "a3-usb-5f2h/v1";
export function telemetryMetadata(side="left") {
  const linear=SELF.filter((_,i)=>!ROTARY.has(i)).map(n=>n+"_joint");
  const rotary=[...SELF.filter((_,i)=>ROTARY.has(i)).map(n=>n+"_joint"),...EXTERNAL];
  return {model_id:"TsenseI_v3_0",telemetry_format:TELEMETRY_FORMAT,
    slot_maps:Object.fromEntries(Object.entries({linear,rotary}).map(([kind,names])=>
      [kind,Object.fromEntries(names.map((n,i)=>[`${side}_hand_${n}`,i]))])),
    telemetry_units:{self_position:"mm or deg by slot",self_current:"firmware_native",
      gravity_current:"A",external_current:"A",external_velocity:"output rpm"},
    linear_feedback_sign:-1,external_gear_ratios:[...MIT_GEAR_RATIOS]};
}
export const selfFeedbackValid = s => !!(s?.online && Number.isFinite(s.position));
// The source accepts an online-mask bit even if the cached status identifier differs.
// Identifier-only packets may be displayed, but remain stale for motion startup.
export const externalFeedbackValid = s => !!(s?.online && !s.fault && s.state!==5 && Number.isFinite(s.position));
export function boardTelemetry(p){
  if(p.length<8)return null;
  const v=new DataView(p.buffer,p.byteOffset,p.byteLength),temp=o=>v.getInt16(o,true)===0x7fff?null:v.getInt16(o,true)/100;
  return {telemetry_seq:v.getUint16(0,true),ntc_c:temp(2),ntc_b:temp(4),vin_v:v.getUint16(6,true)===0xffff?null:v.getUint16(6,true)/1000};
}
export function telemetry(p){
  if(![444,484].includes(p.length))return null;
  if(p.length===444){
    const full=new Uint8Array(484);full.set(p);full[2]=full[3]=0;
    return {...telemetry(full),payload_bytes:444};
  }
  const v=new DataView(p.buffer,p.byteOffset,p.byteLength),mask=v.getUint32(8,true),mitMask=v.getUint16(2,true);
  const self=SELF.map((_,i)=>{
    const o=12+i*24,spin=p[o]>=1&&p[o]<=4,home=p[o]===0&&p[o+1]===0&&p[o+2]===0;
    return {slot:i,joint_suffix:SELF[i]+"_joint",position_unit:ROTARY.has(i)?"deg":"mm",
      online:!!(mask&(1<<i)),home_state:home?p[o+3]:null,
      theta_rad:!spin&&!home?v.getFloat32(o,true):null,
      open_spin_state:spin?p[o]:null,open_spin_remaining_ms:spin?v.getUint16(o+1,true):null,
      open_spin_frequency_hz:spin?v.getFloat32(o+20,true):null,
      position:v.getFloat32(o+4,true),iq:v.getFloat32(o+8,true),iq_ref:v.getFloat32(o+12,true),pos_ref:v.getFloat32(o+16,true),
      grav_iq:spin?null:v.getInt16(o+20,true)/1000,mcu_temp_c:spin||v.getInt16(o+22,true)===0x7fff?null:v.getInt16(o+22,true)/100};
  });
  const external=EXTERNAL.map((_,i)=>{
    const o=444+i*20,online=!!(mitMask&(1<<i)),ident_ok=p[o]===5+4*i&&p[o+1]===5,fault=v.getUint32(o+12,true);
    return {slot:i,joint_suffix:EXTERNAL[i],position_unit:"deg",gear_ratio:MIT_GEAR_RATIOS[i],
      online,ident_ok,valid:online||ident_ok,ident0:p[o],ident1:p[o+1],
      motor_position_deg:(v.getUint32(o+2,true)-0x80000000)/2097152*360,
      motor_velocity_rpm:(v.getUint16(o+6,true)-0x8000)/10,
      position:(v.getUint32(o+2,true)-0x80000000)/2097152*360/MIT_GEAR_RATIOS[i],
      velocity:(v.getUint16(o+6,true)-0x8000)/10/MIT_GEAR_RATIOS[i],iq:(v.getUint16(o+8,true)-0x8000)*11/4096,
      vbus_v:v.getUint16(o+10,true)/10,fault,faults:["驱动","过压","欠压","过流","编码器"].filter((_,bit)=>fault&(1<<bit)),
      state:p[o+16],calib:p[o+17],mode:p[o+18],mtype:p[o+19]};
  });
  // The reference UI shows no measurement for absent/unidentified MIT packets.
  for(const s of external)if(!s.valid)
    for(const key of ["position","velocity","iq","vbus_v"])s[key]=null;
  const linear=self.filter((_,i)=>!ROTARY.has(i)).map(s=>({...s,pos_act_mm:-s.position,pos_ref_mm:-s.pos_ref,iq_meas:s.iq,mech_home_state:s.home_state}));
  const rotary=[...self.filter((_,i)=>ROTARY.has(i)).map(s=>({...s,pos_actual_mm:s.position,pos_ref_mm:s.pos_ref,home_label:s.home_state})),
    ...external.map(s=>({...s,pos_actual_mm:s.position}))];
  const decode_issues=[];
  self.forEach((s,i)=>{
    if(!s.online)return;
    for(const field of ["position","pos_ref","iq","iq_ref"])
      if(!Number.isFinite(s[field]))decode_issues.push({slot:i,joint_suffix:s.joint_suffix,field,reason:"nonfinite_wire_value"});
    if(s.mcu_temp_c!==null&&(s.mcu_temp_c < -50 || s.mcu_temp_c > 150))
      decode_issues.push({slot:i,joint_suffix:s.joint_suffix,field:"mcu_temp_c",reason:"unusual_wire_value"});
  });
  return {telemetry_format:TELEMETRY_FORMAT,telemetry_seq:v.getUint16(0,true),tick_ms:v.getUint32(4,true),
    self_online_mask:mask,external_online_mask:mitMask,payload_bytes:484,self,external,linear,rotary,decode_issues,
    all_online:self.every(selfFeedbackValid)&&external.every(externalFeedbackValid)};
}
