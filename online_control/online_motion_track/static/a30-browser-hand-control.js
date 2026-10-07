import {BrowserHandControl} from "./browser-hand-control.js";
import * as protocol from "./a30-serial-protocol.js";
import {crc16} from "./serial-protocol.js";
import {consumeRecords} from "../../shared/stream.js";
import {StreamClock} from './stream-clock.js';
const LABEL={pinky:"小拇指",ring:"无名指",mid:"中指",index:"食指",thumb:"拇指"};
const RECOVERY_TIMEOUT_MS=2000;

export class A30BrowserHandControl extends BrowserHandControl {
  constructor(options={}){super({...options,protocol});this.modelId=options.modelId||"TsenseI_v3_0";this.requireHandConfirmation=false;
    this.recentWrites=new Float64Array(1024);this.recentWriteCount=0;}
  snapshot(){
    const missing=this.latest?protocol.missingFeedback(this.latest,this.side):[];
    const missingLabels=missing.map(n=>{
      const [finger,...part]=n.replace(`${this.side}_hand_`,'').replace(/_joint2?$/,'').split('_');
      return `${LABEL[finger]||finger} ${part.join('_')}`;
    });
    const base=super.snapshot(),cutoff=this.now()-1000;
    let recent=0;
    if(this.armed)for(let i=0;i<Math.min(this.recentWriteCount,this.recentWrites.length);i++)
      if(this.recentWrites[i]>cutoff)recent++;
    return {...base,local_average_send_hz:base.local_send_hz,local_send_hz:recent,
      collision_paused:!!this.collisionPaused,
      trajectory_recovering:this.recoveryStarted!=null,feedback_missing_joints:missing,
      telemetry_decode_issues:this.latest?.decode_issues||[],
      board_telemetry:this.boardTelemetry?{...this.boardTelemetry,stale:!this.connected||this.now()-this.boardTelemetryAt>4000}:null,
      feedback_note:missing.length?`反馈缺失 ${missing.length}/20 路（${missingLabels.join('、')}） · 使用上次下发目标，首次使用初始姿态`:""};
  }
  async write(bytes,options){
    await super.write(bytes,options);
    if(bytes[5]===0x20){
      const decoded=protocol.decodePositions(bytes),linear={},rotary={};
      protocol.SELF.forEach((n,i)=>(protocol.ROTARY.has(i)?rotary:linear)[`${this.side}_hand_${n}_joint`]=protocol.selfPositionToGeometry(i,decoded.self_pos[i]));
      protocol.EXTERNAL.forEach((n,i)=>rotary[`${this.side}_hand_${n}`]=decoded.mit_pos_deg[i]/protocol.MIT_GEAR_RATIOS[i]);
      this.fallbackTargets={linear,rotary};
    }
  }
  event(type,data={}){
    if(type==="conversion"){
      const named={linear:data.linear_mm,rotary:data.rotary_deg};
      data={...data,named_targets:named,
        linear_mm:protocol.LINEAR_NAMES.map(n=>named.linear[`${this.side}_hand_${n}`]),
        rotary_deg:protocol.ROTARY_NAMES.map(n=>named.rotary[`${this.side}_hand_${n}`])};
    }
    return super.event(type,{...data,model_id:this.modelId});
  }
  async hold(token,source){
    if(source==="arming/feedback_hold"){
      const issue=this.controlTelemetryIssue();if(issue)throw new Error(issue);
    }else if(!this.freshTelemetry())throw new Error("请等待新鲜的 A3 遥测数据包");
    if(!this.feedbackPositionLimits)throw new Error("请等待模型反馈范围加载");
    const seed=source==="arming/feedback_hold"?this.rebasedTargets:null;
    if(source==="arming/feedback_hold"&&!seed)throw new Error("缺少截断后的反馈起始姿态");
    const bytes=seed?protocol.positions(this.seq++,seed.linear,seed.rotary,this.side):
      protocol.holdTelemetry(this.seq++,this.latest,this.side,this.fallbackTargets,this.feedbackPositionLimits);let sent=false;
    try{await this.write(bytes,{token});sent=true;}
    finally{this.event("downlink",{...protocol.decodePositions(bytes),sent,source,joint_enabled:bytes.jointEnabled,wire_hex:Array.from(bytes,b=>b.toString(16).padStart(2,"0")).join("")});}
  }
  freshTelemetry(){return !!(super.freshTelemetry()&&!this.latest.external.some(s=>s.online&&(s.fault||s.state===5)));}
  telemetryIssue(){
    const issue=super.telemetryIssue();if(issue)return issue;
    const i=this.latest.external.findIndex(s=>s.online&&(s.fault||s.state===5));
    return i<0?"":`外接拇指电机 M${i+1} 报告故障`;
  }
  controlTelemetryIssue(){
    // Startup needs one decoded seed; receive gaps do not stop the command stream.
    if(!this.latest)return "首次开启请先接收一包有效的 A3 手部遥测";
    if(this.latest.hand_side&&this.latest.hand_side!==this.side)return "遥测左右手与所选模型不一致";
    const i=this.latest.external.findIndex(s=>s.online&&(s.fault||s.state===5));
    return i<0?"":`外接拇指电机 M${i+1} 报告故障`;
  }
  async arm(rebase,{session}={}){
    if(!this.modeConfirmed)throw new Error("控制模式尚未确认，请重新切换模式");
    this.lastWrittenSample=null;
    this.recentWriteCount=0;
    this.enabling=true;this.rebasedTargets=null;
    this.rebaseControl=rebase;
    this.lastPlannerAt=null;
    this.nextPositionDeadline=null;
    let seedState,seedRequestedAt;
    try{
      await super.arm(async(pose,telemetry)=>{
        seedRequestedAt=this.now();
        const state=await rebase(pose,telemetry);
        seedState=state;
        this.motionLimits=state.motion_limits;
        if(state.linear_strokes_mm&&state.rotary_positions_deg){
          this.rebasedTargets={linear:state.linear_strokes_mm,rotary:state.rotary_positions_deg};
          this.fallbackTargets=this.rebasedTargets;
        }
        this.feedbackPositionLimits=state.feedback_position_limits||this.feedbackPositionLimits;
        this.event("feedback_seed",{fallback_joints:state.feedback_fallback_joints||[],clipped_joints:state.feedback_clipped_joints||[],
          source:"clamped_feedback_or_last_sent_or_initial_pose"});
        return state;
      });
      // Use the validated rebase epoch immediately. Waiting for the next
      // 100 ms status poll can consume the entire 150 ms stream watchdog.
      if(session){
        if(!seedState.control_stream)throw Error('服务器未提供可用的实时下发流');
        this.startStream(session,seedState,seedRequestedAt);
      }
    }catch(e){await this.emergency(e.message);throw e;}
    finally{this.enabling=false;this.rebasedTargets=null;}
  }
  disarm(reason){this.streamRevision=(this.streamRevision||0)+1;this.recoveryStarted=null;this.recovering=false;this.collisionPaused=false;this.streamAbort?.abort();this.streamAbort=null;super.disarm(reason);}
  writeWatchdogExpired(now){return !this.collisionPaused&&!this.recovering&&super.writeWatchdogExpired(now);}
  watchdog(){
    if(!this.armed)return;
    const now=this.now(),issue=this.controlTelemetryIssue();
    if(issue){this.disarm(issue);return;}
    if(this.recoveryStarted!=null){
      this.serialTrace?.flush();
      // One deadline covers rebase AND the first checked sample in the new
      // stream. Status polling must not extend it indefinitely.
      if(now-this.recoveryStarted>RECOVERY_TIMEOUT_MS)this.disarm('轨迹恢复超过 2 秒，实时下发已关闭');
      return;
    }
    if(this.streamAbort&&this.rebaseControl&&!this.positionWritePending&&
       (now-this.lastPollAt>150||this.writeWatchdogExpired(now))){
      // A delayed browser task may run the watchdog before the stale packet
      // handler. Both orders should discard the old stream and replan once.
      void this.recoverTrajectory('stream_timeout',{
        receive_age_ms:now-this.lastPollAt,write_age_ms:now-this.lastSentAt},this.job);
      return;
    }
    super.watchdog();
  }
  finishRecovery(){
    if(this.recoveryStarted==null)return;
    this.event('trajectory_recovered',{duration_ms:this.now()-this.recoveryStarted,browser_epoch:this.epoch});
    this.recoveryStarted=null;
  }
  async recoverTrajectory(reason,detail,token){
    if(!this.armed||token!==this.job||this.recovering)return;
    this.event('trajectory_recovery',{reason,...detail});
    if(!this.rebaseControl)throw Error('缺少反馈重规划接口');
    this.recovering=true;
    this.recoveryStarted??=this.now();
    this.streamRevision=(this.streamRevision||0)+1;
    this.streamAbort?.abort();this.streamAbort=null;
    try{
      const requestedAt=this.now();
      const state=await this.rebaseControl(this.held,this.latestPayload,{resume_from_commanded_pose:true});
      if(!this.armed||token!==this.job)return;
      this.watchdog();
      if(!this.armed||token!==this.job)return;
      this.held={...state.ruckig_joint_angles_rad};this.epoch=state.browser_epoch;
      this.motionLimits=state.motion_limits;this.lastSample=-1;this.lastPlannerAt=null;
      this.nextPositionDeadline=null;
      this.lastPollAt=this.now();this.lastSentAt=this.now();this.accepted=false;
      this.setCollisionPaused(false);
      // Rebase already returned the new epoch and clock. Do not wait for an
      // unrelated status poll (or reuse the old epoch's delayed clock anchor).
      if(this.streamSession&&Number.isFinite(state.server_time)){
        this.recovering=false;
        this.startStream(this.streamSession,state,requestedAt);
      }
    }catch(error){if(this.armed&&token===this.job)this.disarm(error.message);}
    finally{if(token===this.job)this.recovering=false;}
  }
  setCollisionPaused(paused){
    if(!!this.collisionPaused===paused)return;
    this.collisionPaused=paused;
    this.lastError=paused?'碰撞保护暂停，手势安全后自动恢复':'';
    this.event(paused?'control_pause':'control_resume',{reason:'sample_collision',last_sample:this.lastSample});
    this.changed();
  }
  async sample(state,requestedAt){if(!this.enabling&&!state.control_stream)await super.sample(state,requestedAt);}
  startStream(session,state,requestedAt){
    if(!!state.direct_mode!==this.directMode){if(this.armed)this.disarm('控制模式已变化，请重新开启');return;}
    this.streamSession=session;
    this.clockAnchor=(this.streamClock??=new StreamClock()).update(session,state.server_time,requestedAt,this.now());
    if(!this.armed||this.recovering||this.streamAbort||!state.control_stream||state.browser_epoch!==this.epoch)return;
    this.motionLimits=state.motion_limits;
    const abort=this.streamAbort=new AbortController(), token=this.job;
    const names=state.control_joint_names;
    this.streamTask=(async()=>{
      const response=await fetch('/api/a30/stream',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({session,epoch:this.epoch}),signal:abort.signal,cache:'no-store'});
      await consumeRecords(response,bytes=>this.streamSample(bytes,names,token),{signal:abort.signal});
    })().catch(error=>{if(!abort.signal.aborted&&this.job===token)this.disarm(error.message);});
  }
  async streamSample(bytes,names,token){
    if(!this.armed||this.job!==token||this.recovering)return;
    const revision=this.streamRevision||0;
    if(bytes.length<108||String.fromCharCode(...bytes.subarray(0,4))!=='GTR1'||names?.length!==20)
      throw Error('手势下发数据格式错误');
    const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),now=this.now();
    const epoch=view.getUint32(4,true),tick=view.getUint32(8,true),at=view.getFloat64(16,true),flags=view.getUint32(24,true);
    if(epoch!==this.epoch)throw Error('反馈起始姿态已变化，请重新开启');
    if(![0,1,2,4].includes(flags))throw Error('手势下发状态标志无效');
    if(flags===2){this.disarm('跟踪保护已阻断下发，请重新开启');return;}
    const wire=bytes.subarray(108);
    if(flags===1&&(wire.length!==108||wire[0]!==0xaa||wire[1]!==0x55||wire[4]!==3||wire[5]!==0x20||
       (wire[2]|wire[3]<<8)!==100||crc16(wire.subarray(0,-2))!==(wire.at(-2)|wire.at(-1)<<8)))
      throw Error('手势下发帧校验失败');
    const age=now-this.clockAnchor.local-(at-this.clockAnchor.server)*1000;
    if(!Number.isFinite(age))throw Error('手势下发时间戳无效');
    if(age>100){
      await this.recoverTrajectory('stale_sample',{age_ms:age,tick},token);return;
    }
    this.lastPollAt=now;this.watchdog();
    if(!this.armed||revision!==(this.streamRevision||0))return;
    if(flags===4){
      if(bytes.length!==108)throw Error('碰撞暂停帧格式错误');
      this.setCollisionPaused(true);
      this.finishRecovery();
      return;
    }
    if(!(flags&1)){
      if(this.accepted||(flags&2))this.disarm('跟踪保护已阻断下发，请重新开启');
      return;
    }
    if(tick<=this.lastSample)return;
    const dt=this.lastPlannerAt==null?Math.min(.1,Math.max(.002,(now-this.lastSentAt)/1000)):at-this.lastPlannerAt;
    if(!Number.isFinite(dt)||dt<=0)throw Error('轨迹采样时间未递增');
    const pose={};let requiredMs=0;
    for(let i=0;i<20;i++){
      const n=names[i],q=view.getFloat32(28+4*i,true),limit=this.limits[n];
      const velocity=this.motionLimits?.[n]?.velocity;
      if(!Number.isFinite(q)||!limit)throw Error('轨迹包含无效关节或非有限角度');
      if(!this.directMode&&(!Number.isFinite(velocity)||velocity<=0))throw Error('服务器未提供有效运动限值，请更新并刷新页面');
      const delta=Math.abs(q-this.held[n]);
      if(q<limit[0]-1e-5||q>limit[1]+1e-5||(!this.directMode&&delta>velocity*dt*1.05+.002)){
        await this.recoverTrajectory(q<limit[0]-1e-5||q>limit[1]+1e-5?'position_limit':'sample_jump',
          {joint:n,q,previous:this.held[n],lower:limit[0],upper:limit[1],delta,planner_dt_s:dt,
           usb_dt_s:(now-this.lastSentAt)/1000,velocity_limit:velocity,tick},token);return;
      }
      if(!this.directMode)requiredMs=Math.max(requiredMs,delta/velocity*1000);
      pose[n]=q;
    }
    // Do not drain a network batch as an unpaced burst of position commands.
    // Planner timestamps validate the trajectory; local time paces USB writes.
    // Space command starts, not completions: otherwise every USB write's
    // duration is added to the planner period and slowly builds a backlog.
    // lastSentAt still means successful completion for the watchdog/rebase.
    // A stationary or direct-mode target still needs packet pacing. The first
    // sample is immediate; subsequent starts target at least 2 ms apart.
    // Keep a 2 ms schedule, not "actual wake-up + 2 ms". Browser timers often
    // overshoot by 0.4–0.8 ms; adding that to every period creates an unbounded
    // queue even when USB itself takes <1 ms. Recover small timing debt while
    // retaining velocity pacing and at least 1 ms between command starts.
    const deadline=this.nextPositionDeadline??this.now();
    const earliest=Math.max(deadline,(this.lastPositionWriteStartedAt??this.lastSentAt)+
      Math.max(requiredMs,this.lastPositionWriteStartedAt==null?0:1));
    const waitMs=earliest-this.now();
    if(waitMs>100){await this.recoverTrajectory('stream_gap',{tick,wait_ms:waitMs},token);return;}
    while(this.now()<earliest){
      await this.wait(earliest-this.now());
      if(!this.armed||token!==this.job||revision!==(this.streamRevision||0))return;
    }
    if(!this.armed||token!==this.job||revision!==(this.streamRevision||0))return;
    this.watchdog();
    if(!this.armed||revision!==(this.streamRevision||0))return;
    const sendAge=this.now()-this.clockAnchor.local-(at-this.clockAnchor.server)*1000;
    if(sendAge>100){await this.recoverTrajectory('stale_after_pacing',{tick,age_ms:sendAge},token);return;}
    this.nextPositionDeadline=Math.max(deadline,this.now()-20)+2;
    this.positionWritePending=true;
    try{await this.write(wire,{token,stream:true});}
    finally{this.positionWritePending=false;}
    if(!this.armed||this.job!==token)return;
    this.lastSentAt=this.now();this.sendStarted||=this.lastSentAt;this.lastSample=tick;
    this.held=pose;this.lastPlannerAt=at;this.accepted=true;this.sent++;
    this.recentWrites[this.recentWriteCount++%this.recentWrites.length]=this.lastSentAt;
    // Publish only after successful USB completion; the renderer consumes this
    // exact sample, never a newer pose from an unrelated HTTP status response.
    this.lastWrittenSample={pose,tick,epoch,frame_id:view.getInt32(12,true),
      planner_at:at,written_at:this.lastSentAt};
    this.setCollisionPaused(false);
    this.finishRecovery();
    if(this.recorder?.active){
      const frame_id=view.getInt32(12,true),joint_enabled={...this.jointEnabled};
      const sample={sample_ticks:tick,browser_epoch:epoch,planner_at:at,written_at:this.lastSentAt};
      if(this.directMode)this.event('direct_command',{joints:pose,frame_id,sample_ticks:tick,
        browser_epoch:epoch,planner_at:at,sent:true,stage:'checked',source:'successful_usb_write'});
      const command_id=this.event('conversion',{...sample,joints_hw:pose,linear_mm:this.fallbackTargets.linear,
        rotary_deg:this.fallbackTargets.rotary,frame_id,joint_enabled,source:'shared_binary_stream'});
      this.event('h7',{...sample,joints_ui:pose,joints_hw:pose,joints_ruckig_ui:this.directMode?{}:pose,frame_id,joint_enabled,
        sent:true,usb_connected:true,ruckig_faults:[],control_dt_s:dt,source:'shared_binary_stream'});
      const downlink_seq=this.event('downlink',{...sample,...protocol.decodePositions(wire),sent:true,command_id,
        joints_ui:pose,frame_id,joint_enabled,source:'shared_binary_stream'});
      this.lastCommand={downlink_seq,command_id,wire_seq:wire[6],frame_id};
    }
  }
  async home(iq,initPose,limits){
    if(!Number.isFinite(iq)||iq<0||iq>5000)throw new Error("找零 Iq 必须为 0..5000");
    if(!this.connected||this.homing||this.powerBusy||this.armPending||!this.freshTelemetry())throw new Error("请连接并等待 A3 遥测数据包及当前操作结束");
    this.disarm(""); const token=this.job,results=[];
    this.homing=true;this.homed=false;this.homeReport=null;this.homeResult=null;this.requiresTelemetrySeed=true;
    const check=()=>{if(token!==this.job||!this.connected)throw new Error("找零已取消");if(!this.freshTelemetry())throw new Error("A3 遥测超时或外接电机报告故障");};
    const send=async b=>{check();await this.write(b,{token});};
    try{
      this.event("homing_start",{iq_seek:iq,model_id:this.modelId});
      check();await this.hold(token,"homing/feedback_hold");
      for(let slot=0;slot<18;slot++){
        const [bus,node]=protocol.address(slot),suffix=protocol.SELF[slot],[finger,part]=suffix.split("_");
        const joint=`${this.side}_hand_${suffix}_joint`;
        if(this.jointEnabled[joint]===false||!protocol.selfFeedbackValid(this.latest.self[slot])){
          if(this.jointEnabled[joint]===false)await this.sendJointPower(joint,false,token);
          results.push({joint,label:`${LABEL[finger]} ${part}`,channel:`CAN${bus+1} M${node}`,success:false,skipped:true,
            reason:this.jointEnabled[joint]===false?"已选择断电，跳过找零":"该电机反馈缺失，跳过找零"});
          continue;
        }
        this.homeStatus=`找零 ${slot+1}/18 · ${LABEL[finger]} ${part}`;this.changed();
        // Reset old completion, then seek. No V2 rotary-zero/hand-select opcode.
        await send(protocol.home(bus,node,2,iq));
        const resetAt=this.now();let cleared=false,seq=this.lastTelemetrySeq;
        while(this.now()-resetAt<1500){
          await this.wait(25);check();if(seq===this.lastTelemetrySeq)continue;seq=this.lastTelemetrySeq;
          if(protocol.selfFeedbackValid(this.latest.self[slot])&&this.latest.self[slot].home_state===0){cleared=true;break;}
        }
        let done=false,reason="找零复位未得到确认";
        if(cleared){
          await this.sendJointPower(joint,true,token);
          await send(protocol.home(bus,node,1,iq));
          const started=this.now();seq=this.lastTelemetrySeq;
          while(this.now()-started<15000){
            await this.wait(25);check();if(seq===this.lastTelemetrySeq)continue;seq=this.lastTelemetrySeq;
            const state=this.latest.self[slot].home_state;
            if(!protocol.selfFeedbackValid(this.latest.self[slot])){reason="该电机反馈缺失";break;}
            if(state===2){done=true;break;}
            if(state===3){reason="下位机报告找零错误";break;}
            reason="15 秒内未确认完成";
          }
        }
        if(!done)await send(protocol.home(bus,node,2,iq));
        const result={joint:`${this.side}_hand_${suffix}_joint`,label:`${LABEL[finger]} ${part}`,channel:`CAN${bus+1} M${node}`,
          kind:protocol.ROTARY.has(slot)?"rotary":"linear",bus,motor:node,slot,success:done,reason:done?null:reason};
        results.push(result);this.event(done?"homing_axis_complete":"homing_axis_failed",result);
      }
      for(const [slot,suffix] of protocol.EXTERNAL.entries())results.push({joint:`${this.side}_hand_${suffix}`,label:`拇指 ${suffix}`,
        channel:`CAN3 外接 M${slot+1}`,kind:"rotary",success:false,reason:"源协议未提供独立找零命令，将使用实际位置反馈起步"});
      check();this.held={...initPose};this.limits=limits;this.homed=true;
      this.homeReport={hand_side:this.side,completed:true,total:20,results,failures:results.filter(r=>!r.success)};
      this.homeResult=this.homeReport.failures.length?"partial":"complete";
      this.homeStatus=`找零结束 · ${this.homeReport.failures.length} 路未确认，可根据反馈继续下发`;
      this.event("homing_complete",{...this.homeReport,seed_source:"telemetry_on_arm"});return this.homeReport;
    }catch(e){if(token===this.job){this.homeStatus=e.message;await this.emergency(e.message);}throw e;}
    finally{this.homing=false;this.changed();}
  }
  async emergency(reason="已停发，并请求自研电机退出找零及断电"){
    this.disarm(reason);this.homed=false;this.homing=false;this.homeResult="cancelled";
    if(!this.connected)return;
    const token=this.job;let ok=true;
    for(let slot=0;slot<18;slot++){
      const [bus,node]=protocol.address(slot);
      for(const b of [protocol.home(bus,node,2,0),protocol.power(bus,node,2)])
        ok=(await this.write(b,{token}).then(()=>true,()=>false))&&ok;
    }
    this.lastError=ok?"自研电机已请求断电；外接两路仅停发（源协议无断电命令）":"停止指令未全部写入，请检查设备";
    this.powerSent = {}; // A USB write is not a firmware power-state acknowledgement.
    this.event("emergency_stop",{sent:ok,reason,external_power_off_supported:false});this.changed();
  }
}
