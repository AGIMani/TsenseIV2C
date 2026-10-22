// Pure JS tests: no browser, network, or hardware is launched.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../tools/hand_control_log_viewer.html'), 'utf8');
const elements = new Map();
class Element {
  constructor(id='') { this.id=id;this.dataset={};this.children=[];this.options=[];this.value='';this.checked=false;this.style={};this.listeners={}; }
  set innerHTML(value) { this.html=value;for(const m of value.matchAll(/id="([^"]+)"/g))get(m[1]); }
  get innerHTML() { return this.html||''; }
  append(...items) { this.children.push(...items); }
  add(option) { this.options.push(option);if(this.options.length===1)this.value=option.value; }
  replaceChildren() { this.children=[];this.options=[];this.value=''; }
  addEventListener(name,callback) { this.listeners[name]=callback; }
  getBoundingClientRect() { return {width:1100,height:360,left:0,top:0}; }
  getContext() { return new Proxy({}, {get:(_,key)=>key==='measureText'?text=>({width:String(text).length*7}):()=>{}}); }
  click() { this.listeners.click?.(); }
}
function get(id){if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id);}
for(const m of html.matchAll(/id="([^"]+)"/g))get(m[1]);
for(const m of html.matchAll(/<select id="([^"]+)"[^>]*>([\s\S]*?)<\/select>/g)) {
  for(const o of m[2].matchAll(/<option value="([^"]+)"[^>]*>([^<]*)<\/option>/g))get(m[1]).add({value:o[1],text:o[2]});
}
for(const m of html.matchAll(/<input id="([^"]+)"[^>]*checked[^>]*>/g))get(m[1]).checked=true;
const coords=[0,1,2].map(i=>Object.assign(new Element(),{value:String(i),checked:true}));
const document={getElementById:get,createElement:()=>new Element(),querySelectorAll:selector=>{
  if(selector.startsWith('.coord'))return coords.filter(x=>!selector.endsWith(':checked')||x.checked);
  if(selector==='#jointChecks input:checked')return get('jointChecks').children.flatMap(x=>x.children||[]).filter(x=>x.checked);
  if(selector==='details[data-fold]')return [];
  throw new Error('Unexpected selector: '+selector);
}};
let exported;
const context=vm.createContext({document,window:{devicePixelRatio:1,addEventListener(){},localStorage:{getItem(){return null;},setItem(){}}},location:{protocol:'file:'},crypto:require('node:crypto').webcrypto,Option:class {constructor(text,value){this.text=text;this.value=value;}},console,TextDecoder,Uint8Array,Blob,URL:{createObjectURL:b=>(exported=b,'blob:test'),revokeObjectURL(){}},setTimeout:()=>{},requestAnimationFrame:callback=>callback()});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const run=source=>vm.runInContext(source,context);
const consume=row=>run(`consume(${JSON.stringify(row)})`);
consume({type:'session',schema:'hand-control-session/v2',metadata:{hand_side:'left',serial_port:'<img src=x>'}});
for(let i=0;i<30;i++) {
  const t=i/10;
  consume({type:'conversion',seq:1+i*3,t,frame_id:i,hand_side:'left',linear_mm:[i],rotary_deg:[i/2]});
  consume({type:'downlink',seq:2+i*3,t:t+.01,frame_id:i,command_id:1+i*3,hand_side:'left',sent:i!==5,linear_mm:[i],rotary_deg:[i/2]});
  consume({type:'telemetry',seq:3+i*3,t:t+.02,hand_side:'left',accepted:true,linear:[{pos_ref_mm:i,pos_act_mm:i-.2,iq_meas:i===10?100:1,iq_ref:1.2,grav_iq:.2,mcu_temp_c:i===0?null:35}],rotary:[{pos_ref_mm:i/2,pos_actual_mm:i/2-.1,iq:2,iq_ref:3}]});
}
run('drawAll(true)');
assert.equal(run('makeMotorSeries().position[0].points.length'),30);
assert.equal(run('makeMotorSeries().position[1].points.length'),29);
assert.equal(run('makeMotorSeries().position[4].points.length'),1);
assert.equal(run('makeMotorSeries().temperature[0].points.length'),29);
// Curve checkboxes affect drawing, tooltip series and automatic Y bounds only.
const positionChecks=get('positionLegend').children.map(label=>label.children[0]);
assert.equal(positionChecks.length,5);assert(positionChecks.every(input=>input.checked));
for(const index of [0,1,2,4]){positionChecks[index].checked=false;positionChecks[index].listeners.change();}
assert.equal(run('chartModels.get($("positionCanvas")).series.length'),1);
assert.equal(run('chartModels.get($("positionCanvas")).series[0].name'),'回传实际');
assert.equal(run('chartModels.get($("positionCanvas")).ymin'),-.2);
assert.equal(run('chartModels.get($("positionCanvas")).ymax'),28.8);
assert.equal(run('makeMotorSeries().position.length'),5); // Stored/exported data is retained.
positionChecks[3].checked=false;positionChecks[3].listeners.change();
assert.equal(run('chartModels.has($("positionCanvas"))'),false);
for(const input of positionChecks){input.checked=true;input.listeners.change();}
assert.equal(run('chartModels.get($("positionCanvas")).series.length'),5);
assert(get('sessionStats').innerHTML.includes('回传帧'));
assert(!get('sessionStats').innerHTML.includes('<img'));
assert.equal(get('motor').options.length,21);
run('inspectFrame(1.01)');
assert.equal(JSON.parse(get('frameDetail').textContent).conversion.frame_id,10);
// Verify peak-preserving drawing and exact tooltip data on long recordings.
assert.equal(run('Math.max(...downsample(Array.from({length:20000},(_,i)=>[i,i===12345?999:0]),100).map(p=>p[1]))'),999);
const before=run('chartModels.get($("currentCanvas")).xmax-chartModels.get($("currentCanvas")).xmin');
get('positionZoomIn').click();
const after=run('chartModels.get($("currentCanvas")).xmax-chartModels.get($("currentCanvas")).xmin');
assert(after<before);
// Local X zoom fits all visible curves, excluding peaks outside the window.
run('drawChart($("positionCanvas"),[{name:"local",color:"red",points:[[0,1000],[1,2],[1.5,7],[2,4],[3,-1000]]}],"","mm")');
run('applyChartView($("positionCanvas"),{xmin:1,xmax:2,ymin:-1000,ymax:1000})');
assert.equal(run('chartModels.get($("positionCanvas")).ymin'),2);
assert.equal(run('chartModels.get($("positionCanvas")).ymax'),7);
// Y wheel remains manual until the next X operation.
run('interactionControllers.get($("positionCanvas")).zoom(.5,null,null,"y")');
assert.equal(run('chartModels.get($("positionCanvas")).autoY'),false);
assert.equal(run('chartModels.get($("positionCanvas")).ymin'),3.25);
assert.equal(run('chartModels.get($("positionCanvas")).ymax'),5.75);
run('applyChartView($("positionCanvas"),{xmin:1.4,xmax:1.8,ymin:-1000,ymax:1000})');
assert.equal(run('chartModels.get($("positionCanvas")).autoY'),true);
assert.equal(run('chartModels.get($("currentCanvas")).ymin'),.2);
assert.equal(run('chartModels.get($("currentCanvas")).ymax'),1.2);
assert.equal(run('chartModels.get($("temperatureCanvas")).visibleMin'),35);
assert.equal(run('chartModels.get($("temperatureCanvas")).visibleMax'),35);
assert(run('chartModels.get($("temperatureCanvas")).ymax>chartModels.get($("temperatureCanvas")).ymin'));
assert(get('temperatureRange').textContent.includes('[35, 35]'));
// Boundary interpolation is part of the visible line, even with no sample inside.
assert.equal(run('JSON.stringify(visibleCurvePoints([[0,0],[1,10]],.25,.75))'),'[[0.25,2.5],[0.75,7.5]]');
assert.equal(run('visibleCurvePoints([[0,0],[1,10]],2,3).length'),0);
// Missing retarget frames cannot become a fabricated ramp or tooltip sample.
assert.equal(run('JSON.stringify(curveSegments([[0,0],[.1,1],[4,2]],.3))'),'[[[0,0],[0.1,1]],[[4,2]]]');
assert.equal(run('exactJointSampler([{t:0,joints:{left_hand_index_swing_joint:0}},{t:4,joints:{left_hand_index_swing_joint:1}}],"index_swing",.3)(2)'),null);
assert.equal(run('exactJointSampler([{t:0,joints:{left_hand_index_swing_joint:0}},{t:4,joints:{left_hand_index_swing_joint:1}}],"index_swing",.3)(4)[1]'),1);
run('drawChart($("jointCanvas"),[{name:"retarget",color:"red",maxGap:.3,points:[[0,0],[.1,1],[4,2]]}],"","rad")');
run('applyChartView($("jointCanvas"),{xmin:1,xmax:3,ymin:0,ymax:2},{linkTime:false})');
assert.equal(run('chartModels.get($("jointCanvas")).visibleMin'),null);
run('applyChartView($("temperatureCanvas"),{xmin:0,xmax:.05,ymin:30,ymax:40})');
assert.equal(run('chartModels.get($("temperatureCanvas")).visibleMin'),null);
assert(get('temperatureRange').textContent.includes('当前区间无曲线'));
get('autoFitY').checked=false;get('autoFitY').listeners.change();
run('applyChartView($("positionCanvas"),{xmin:1,xmax:2,ymin:0,ymax:10})');
assert.equal(run('chartModels.get($("positionCanvas")).ymin'),0);
assert.equal(run('chartModels.get($("positionCanvas")).ymax'),10);
get('autoFitY').checked=true;get('autoFitY').listeners.change();
get('motor').value='rotary:0';run('drawAll(true)');
assert.equal(run('makeMotorSeries().temperature[0].points.length'),0);
assert.equal(run('motorSelection().unit'),'deg');
get('handSide').value='right';run('drawAll(true)');
assert.equal(run('makeMotorSeries().position[0].points.length'),0);
get('handSide').value='left';get('motor').value='linear:0';
get('exportMotor').click();
assert(exported instanceof Blob);
// Missing / null values must not become zero when inspecting raw MediaPipe.
consume({type:'mediapipe_raw',t:0,frame_id:1,world_landmarks:[[[null,0.2,0.3]]],normalized_landmarks:[]});
get('mpSource').value='world';run('drawAll(true)');
assert.equal(run('makeMpSeries()[0].points.length'),0);
assert.equal(run('makeMpSeries()[1].points[0][1]'),0.2);
// Original v1 files still load with the existing charts and no fabricated telemetry.
run('resetData()');
consume({type:'retarget',t:0,frame_id:1,joints:{left_hand_thumb_rota_joint:.3}});
consume({type:'h7',t:.01,frame_id:1,joints_ui:{left_hand_thumb_rota_joint:.2},sent:true});
run('drawAll(true)');
assert.equal(run('makeJointSeries()[0].points[0][1]'),.3);
assert.equal(run('makeMotorSeries().current[0].points.length'),0);
// A recording may start on the right hand and then contain only left-hand events.
run('resetData()');
consume({type:'session',metadata:{hand_side:'right'}});
consume({type:'conversion',t:0,seq:2,hand_side:'left',joints_hw:{left_hand_thumb_rota_joint:.4},linear_mm:[1],rotary_deg:[2]});
consume({type:'telemetry',t:.01,seq:3,hand_side:'left',accepted:true,linear:[{pos_ref_mm:1,pos_act_mm:.8,iq_meas:3,mcu_temp_c:35}],rotary:[]});
consume({type:'mediapipe_raw',t:0,world_landmarks:[],normalized_landmarks:[]});
run('selectLoadedData()');
assert.equal(get('handSide').value,'left');
assert.equal(run('makeJointSeries().some(s=>s.points.length)'),false); // Conversion is auxiliary, not a pipeline angle stage.
assert(!html.includes('id="stageConversion"'));
assert.equal(run('makeMotorSeries().current[0].points.length'),1);
assert(get('dataNotice').textContent.includes('未检测到手'));
assert(get('dataNotice').textContent.includes('会话头'));
// Optional integration check against a local recording, without copying it into tests.
const zeroPayload=Buffer.alloc(780);
zeroPayload.writeFloatLE(-0,8+16);
zeroPayload.writeFloatLE(-0,8+32+4);
const oldZeroRow={type:'telemetry',t:0,seq:4,hand_side:'left',payload_hex:zeroPayload.toString('hex'),linear:[{pos_ref_mm:-2147483.648,pos_act_mm:-1.8,pos_err_mm:-2147481.848},{pos_ref_mm:2,pos_act_mm:-2147483.648,pos_err_mm:2147485.648}],rotary:[]};
context.oldZeroRow=oldZeroRow;
const fixed=run('repairLegacySignedZero(oldZeroRow)');
assert.equal(fixed.linear[0].pos_ref_mm,0);
assert.equal(fixed.linear[0].pos_act_mm,-1.8);
assert.equal(fixed.linear[0].pos_err_mm,1.8);
assert.equal(fixed.linear[1].pos_act_mm,0);
assert.equal(fixed.linear[1].pos_err_mm,2);
assert.equal(oldZeroRow.linear[0].pos_ref_mm,-2147483.648); // Original remains untouched.
assert.equal(fixed.payload_hex,oldZeroRow.payload_hex);
assert.equal(fixed.decode_corrections.filter(c=>c.field!=='pos_err_mm').length,2);
assert.equal(run('repairLegacySignedZero({...oldZeroRow,payload_hex:undefined}).linear[0].pos_ref_mm'),-2147483.648);
assert.equal(run('repairLegacySignedZero({...oldZeroRow,payload_hex:"00".repeat(780)}).linear[0].pos_ref_mm'),-2147483.648);
run('resetData()');consume(oldZeroRow);run('selectLoadedData()');
assert.equal(run('makeMotorSeries().position[2].points[0][1]'),0);
assert(run('chartModels.get($("positionCanvas")).ymin')>-10);
assert(get('dataNotice').textContent.includes('修正 2 处'));

// Raw A3 / directory diagnostic logs need neither cached decoded arrays nor a header.
run('resetData()');
const a30Payload=Buffer.alloc(484);a30Payload.writeUInt32LE(0x3ffff,8);
for(let i=0;i<18;i++){
  const o=12+i*24;a30Payload.writeFloatLE(-i-.25,o+4);
  a30Payload.writeFloatLE(i+.5,o+8);a30Payload.writeFloatLE(i+.75,o+12);
  a30Payload.writeFloatLE(-i-1,o+16);a30Payload.writeInt16LE(2500,o+22);
}
consume({event:'telemetry',elapsed_ms:1250,context:{hand_side:'left'},payload_hex:a30Payload.toString('hex')});
consume({type:'retarget',t:1,joints:{left_hand_thumb_rota_block_joint:.3,left_hand_thumb_rotaback_joint2:.4}});
run('selectLoadedData()');
assert.equal(get('motor').options.length,20);
get('motor').value='linear:1';
assert.equal(run('makeMotorSeries().position[3].points[0][1]'),5.25);
assert.equal(run('makeMotorSeries().current[0].points[0][1]'),5.5);
assert.equal(run('state.telemetry[0].t'),1.25);
assert.equal(run('state.telemetry[0].external[0].position'),null);
assert.equal(run('findJoint(state.retarget[0].joints,"thumb_rotaback_joint2")'),.4);
get('finger').value='thumb';run('setJointChecks()');
assert.equal(run('selectedJoints()[0]'),'thumb_rota_block');
assert.equal(run('makeJointSeries()[0].points[0][1]'),.3);
get('finger').value='pinky';run('setJointChecks()');
assert.equal(get('jointChecks').children.length,4);
a30Payload.writeFloatLE(NaN,12+5*24+16);
consume({type:'telemetry',t:2,hand_side:'left',payload_hex:a30Payload.toString('hex')});
run('updateDataNotice();inspectFrame(2)');
assert(get('dataNotice').textContent.includes('非有限值'));
assert(JSON.parse(get('frameDetail').textContent).decode_issues.some(x=>x.joint_suffix==='pinky_MCP_joint'));
get('finger').value='thumb';

// SDK A3.2 uses its own 14+6 map, not the V2 14+7 or A3.0 wire order.
for(const model of ['TsenseI_v3_2','TsenseI_v3_2_withid']) {
  run('resetData()');
  const channels=JSON.parse(fs.readFileSync(path.join(__dirname,'../src/a32_driver/a32_hardware.json'),'utf8')).channels;
  const slot_maps=Object.fromEntries(['linear','rotary'].map(kind=>[kind,Object.fromEntries(channels.filter(c=>(c.unit==='mm')===(kind==='linear')).map((c,i)=>[c.joint,i]))]));
  consume({type:'session',metadata:{model_id:model,hand_side:'left',slot_maps,telemetry_format:'a32-native-sdk-state/v1',read_only_capture:true}});
  consume({type:'sdk_status',t:12,hand_side:'left',sdk_state:{connected:false,self:[],mit:[],telemetry_age_ms:-1}});
  consume({type:'sdk_error',t:12,hand_side:'left',error:'synthetic timeout'});
  run('selectLoadedData();inspectFrame(12)');
  assert.equal(get('motor').options.length,20);
  assert(get('motor').options.some(option=>option.text.includes('thumb_rotaback_joint2')));
  assert.equal(run('selectedJoints()[0]'),'thumb_rota_block');
  assert.equal(run('state.telemetry.length'),0);
  assert.equal(JSON.parse(get('frameDetail').textContent).sdk_status.sdk_state.connected,false);
  assert.equal(JSON.parse(get('frameDetail').textContent).sdk_event.error,'synthetic timeout');
  assert(get('dataNotice').textContent.includes('没有下发动作'));
  assert(get('sessionStats').innerHTML.includes('12.000 s'));
  get('finger').value='pinky';run('setJointChecks()');
  assert.equal(get('jointChecks').children.length,4);
  get('finger').value='thumb';
}

if(process.argv[2]) {
  run('resetData()');
  for(const line of fs.readFileSync(process.argv[2],'utf8').split(/\r?\n/))if(line.trim())consume(JSON.parse(line));
  run('selectLoadedData()');
  const metrics=JSON.parse(run('JSON.stringify({side:$("handSide").value,conversion:makeMotorSeries().position[0].points.length,current:makeMotorSeries().current[0].points.length,temperature:makeMotorSeries().temperature[0].points.length,positionRange:[chartModels.get($("positionCanvas"))?.ymin,chartModels.get($("positionCanvas"))?.ymax],notice:$("dataNotice").textContent})'));
  if(run('!!state.meta?.metadata?.read_only_capture')) {
    assert.equal(metrics.conversion,0);
    assert(run('state.sdk.length>0'));
    run('inspectFrame(0)');
    assert(JSON.parse(get('frameDetail').textContent).sdk_status);
  } else {
    assert(metrics.current>0);assert(metrics.conversion>0);
  }
  console.log('Local recording:',JSON.stringify(metrics));
}
run('resetData()');
consume({type:'session',metadata:{model_id:'TsenseI_v3_0',hand_side:'left',offline_replay:true}});
assert.equal(run('JSON.stringify(heldPoints([[1,0],[2,1]]))'),'[[1,0],[2,0],[2,1]]');
consume({type:'control_target',t:1,hand_side:'left',joints:{left_hand_mid_PIP_joint:.7}});
consume({type:'direct_filter',t:1.002,hand_side:'left',sent:false,joints:{left_hand_mid_PIP_joint:.25}});
consume({type:'direct_filter_detail',t:1.002,filtered_joint_angles_rad:{left_hand_mid_PIP_joint:.27},checked_joint_angles_rad:{left_hand_mid_PIP_joint:.25}});
consume({type:'h7',t:1.002,direct_mode:true,joints_ui:{left_hand_mid_PIP_joint:.25},joints_ruckig_ui:{left_hand_mid_PIP_joint:.25}});
get('finger').value='mid';run('selectLoadedData()');
for(const wrapper of get('jointChecks').children)for(const input of wrapper.children)if(typeof input==='object')input.checked=input.value==='mid_PIP';
get('stageDirect').checked=true;get('stageRuckig').checked=true;
assert(run('makeJointSeries().some(s=>s.name.includes("数据直出")&&s.points.some(p=>p[1]===.25))'));
assert(!html.includes('id="stageFilterCandidate"'));
assert(run('makeJointSeries().find(s=>s.stage==="mink").points.length')===0);
assert(run('makeJointSeries().filter(s=>s.name.includes("Ruckig")).every(s=>s.points.length===0)'));
assert(get('dataNotice').textContent.includes('未写入真机'));
run('inspectFrame(1.002)');
assert.equal(JSON.parse(get('frameDetail').textContent).direct_filter.sent,false);
assert.equal(JSON.parse(get('frameDetail').textContent).direct_filter_detail.filtered_joint_angles_rad.left_hand_mid_PIP_joint,.27);
run('resetData()');assert.equal(run('state.filtered.length+state.filterDetail.length'),0);
// Sampled diagnostic logs contain raw keypoints but no filtered-input or H7
// events. Loading them must choose an existing, honestly labelled data source.
get('mpSource').value='input';get('mpHand').value='0';
consume({type:'session',schema:'hand-diagnostic/v1',metadata:{}});
consume({type:'mediapipe_raw',t:2,world_landmarks:[[],[[.1,.2,.3]]],normalized_landmarks:[]});
run('selectLoadedData()');
assert.equal(get('mpSource').value,'world');
assert.equal(get('mpHand').value,'1');
assert.equal(run('makeMpSeries()[0].points[0][1]'),.1);
assert.equal(run('state.mp.length'),0); // No fabricated retarget input.
assert(get('dataNotice').textContent.includes('没有 Retarget 输入关键点'));
assert(get('dataNotice').textContent.includes('H7 曲线为空'));
// A valid user's raw/normalized selection remains selected on file load.
run('resetData()');get('mpSource').value='normalized';
consume({type:'mediapipe',t:1,landmarks:[[0,0,0],[.1,.2,.3]]});
consume({type:'mediapipe_raw',t:1,normalized_landmarks:[[[.4,.5,.6]]],world_landmarks:[]});
run('selectLoadedData()');
assert.equal(get('mpSource').value,'normalized');
assert.equal(get('mpHand').value,'0');
get('mpSource').value='input';run('selectLoadedData()');
assert.equal(get('mpSource').value,'input');
assert(get('dataNotice').textContent.includes('腕坐标系原点'));
// Null-only raw records are missing observations, not zero-valued samples.
run('resetData()');get('mpSource').value='input';
consume({type:'mediapipe_raw',t:1,world_landmarks:[[[null,null,null]]],normalized_landmarks:[[[.2,.3,.4]]]});
run('selectLoadedData()');assert.equal(get('mpSource').value,'normalized');
// Quest native 25 joints have their own indices and meter units. They cannot
// be labelled as MediaPipe image coordinates or fabricated retarget inputs.
// Wuji's native 21 points remain separate from camera and WebXR coordinates.
run('resetData()');get('mpSource').value='input';
const wujiPoints=Array.from({length:21},(_,i)=>[i*.001,i*.002,i*.003]);
consume({type:'wuji_glove_raw',t:0,hand_side:'left',tracked:true,raw_wuji:{positions_xyz:wujiPoints},source_metadata:{stale:false}});
run('selectLoadedData()');assert.equal(get('mpSource').value,'wuji_world');
assert.equal(run('hasMpSource("normalized")'),false);
assert.equal(run('hasMpSource("world")'),false);
assert.deepEqual(JSON.parse(run('JSON.stringify(state.raw[0].world_landmarks[0])')),wujiPoints);
consume({type:'wuji_glove',t:0,landmarks:wujiPoints,depth_source:'wuji_glove_metric'});
assert.equal(run('state.mp.length'),1);
consume({type:'wuji_glove_raw',t:1,tracked:false,raw_wuji:{positions_xyz:wujiPoints}});
assert.equal(run('state.raw[1].world_landmarks.length'),0);

run('resetData()');get('mpSource').value='normalized';get('mpHand').value='0';
const webxrNames=JSON.parse(run('JSON.stringify(WEBXR_NAMES)'));
const nativeWebXR={type:'hand_skeleton',joint_set:'webxr-hand-25',coordinate_space:'webxr_local_floor',
  hands:[{handedness:'left',tracked:true,joints:webxrNames.map((name,index)=>({
    name,position:[index/100,1.2,-.5],orientation:[0,0,0,1],radius:.007,valid:index!==6}))}]};
const webxrMetadata={source:'webxr',output_coordinate_space:'webxr_wrist_relative',normalized_is_image:false,stale:false};
consume({type:'webxr_raw',t:1,frame_id:17,hand_side:'left',source_metadata:webxrMetadata,raw_webxr:nativeWebXR,
  world_landmarks:[Array.from({length:21},()=>[0,0,0])],normalized_landmarks:[[[.1,.2,.3]]]});
run('selectLoadedData()');
assert.equal(get('mpSource').value,'webxr_world');
assert.equal(get('landmark').options.length,25);
assert(get('mpTitle').textContent.includes('WebXR 原始 world / m'));
assert(get('dataNotice').textContent.includes('WebXR 没有归一化图像坐标'));
assert.equal(run('state.raw[0].world_landmarks[0].length'),25);
assert.equal(run('state.raw[0].compatibility_world_landmarks[0].length'),21);
assert.equal(run('state.raw[0].normalized_landmarks.length'),0);
assert.equal(run('hasMpSource("normalized")'),false);
assert.equal(run('hasMpSource("world")'),false);
assert.equal(run('state.mp.length'),0);
get('landmark').value='24';run('drawAll(true)');
assert.equal(run('makeMpSeries()[0].points[0][1]'),.24);
assert(run('makeMpSeries()[0].name.includes("pinky-finger-tip")'));
get('landmark').value='6';run('drawAll(true)');
assert.equal(run('makeMpSeries()[0].points.length'),0);
assert.equal(nativeWebXR.hands[0].joints[6].position[0],.06); // Validity gating leaves raw data intact.
run('inspectFrame(1)');
let webxrDetail=JSON.parse(get('frameDetail').textContent);
assert.deepEqual(webxrDetail.webxr_raw.raw_webxr,nativeWebXR);
assert.deepEqual(webxrDetail.webxr_raw.source_metadata,webxrMetadata);
assert.equal(webxrDetail.mediapipe_raw,null);
consume({type:'webxr',t:1,frame_id:17,hand_side:'left',depth_source:'webxr_metric',landmarks:Array.from({length:21},(_,i)=>[i/200,0,0])});
get('mpSource').value='input';run('drawAll(true)');
assert.equal(get('landmark').options.length,21);
assert.equal(run('state.mp[0].depth'),'webxr_metric');
get('landmark').value='20';run('drawAll(true)');
assert.equal(run('makeMpSeries()[0].points[0][1]'),.1);
// Stale snapshots retain the original payload for inspection, but cannot
// create fresh raw samples or connect a line across a tracking-loss interval.
consume({type:'webxr_raw',t:2,frame_id:18,source_metadata:{...webxrMetadata,stale:true},raw_webxr:nativeWebXR});
consume({type:'webxr_raw',t:3,frame_id:19,source_metadata:webxrMetadata,raw_webxr:nativeWebXR});
get('mpSource').value='webxr_world';get('landmark').value='4';run('drawAll(true)');
assert.equal(run('makeMpSeries()[0].points.length'),2);
assert.equal(run('makeMpSeries()[0].segments.length'),2);
assert.equal(run('makeMpSeries()[0].sampleAt(2)'),null);
assert.deepEqual(JSON.parse(run('JSON.stringify(state.raw[1].raw_webxr)')),nativeWebXR);
// A log containing both sources keeps each raw schema separate.
consume({type:'mediapipe_raw',t:4,frame_id:20,world_landmarks:[[[.6,.7,.8]]],normalized_landmarks:[[[.2,.3,.4]]]});
get('mpSource').value='world';get('landmark').value='0';run('drawAll(true)');
assert.equal(get('landmark').options.length,21);
assert.equal(run('makeMpSeries()[0].points.length'),1);
assert.equal(run('makeMpSeries()[0].points[0][1]'),.6);
// Already-extracted native world rows can also load without inventing a raw payload.
run('resetData()');get('mpSource').value='input';
consume({type:'webxr_raw',t:0,world_landmarks:[Array.from({length:25},(_,i)=>[i,0,0])]});
run('selectLoadedData()');assert.equal(get('mpSource').value,'webxr_world');
assert.equal(run('state.raw[0].raw_webxr'),undefined);
run('resetData()');
consume({type:'session',schema:'hand-control-session/v2',recording_part:{index:2,count:5},metadata:{}});
consume({type:'retarget_skipped',frame_id:2,t:1,reason:'queue_expired'});
consume({type:'retarget_skipped',frame_id:3,t:1.1,reason:'queue_replaced'});
run('selectLoadedData()');
assert(get('dataNotice').textContent.includes('第 2 / 5 段'));
assert(get('dataNotice').textContent.includes('送入求解前已过期 1 帧'));
assert(get('dataNotice').textContent.includes('被更新帧替换 1 帧'));
// Usage identity survives repeated imports, out-of-order parts and device renames.
run('resetData();usageRecords=[]');
const usageHeader={type:'session',schema:'hand-control-session/v2',session_id:'usage-1',started_at:'2026-09-24T01:00:00Z',
  recording_part:{index:2,count:2,session_id:'usage-1'},metadata:{model_id:'TsenseI_v3_0',hand_side:'left',hand_name:'sample <01>'}};
consume(usageHeader);consume({type:'retarget',t:60,hand_side:'left',joints:{}});consume({type:'session_end',t:120,duration_s:120});
run('registerUsage("part002.jsonl")');
assert.equal(run('usageDuration(usageRecords[0])'),60000);assert.equal(run('usageComplete(usageRecords[0])'),false);
run('registerUsage("part002-copy.jsonl")');assert.equal(run('usageRecords.length'),1);
run('usageRecords[0].name="renamed";resetData()');
consume({...usageHeader,recording_part:{index:1,count:2,session_id:'usage-1'}});
consume({type:'retarget',t:59,hand_side:'left',joints:{}});run('registerUsage("part001.jsonl");renderUsage()');
assert.equal(run('usageDuration(usageRecords[0])'),120000);assert.equal(run('usageComplete(usageRecords[0])'),true);
assert.equal(run('usageRecords[0].name'),'renamed');
assert.equal(run('JSON.stringify(unionRanges([[0,20],[10,30],[50,60]]))'),'[[0,30],[50,60]]');
assert.equal(run('validUsage({id:"bad"})'),false);
run('resetData()');consume({...usageHeader,session_id:'usage-2',recording_part:undefined,metadata:{model_id:'TsenseI_v3_2',hand_side:'right',hand_name:'<img src=x>'}});
consume({type:'retarget',t:30,hand_side:'right',joints:{}});get('usageModel').value='A32';run('registerUsage("interrupted.jsonl");renderUsage()');
assert.equal(run('usageRecords.length'),2);assert.equal(run('usageComplete(usageRecords[1])'),false);
assert.equal(run('usageDuration(usageRecords[1])'),30000);
assert(!get('usageRows').innerHTML.includes('<img'));assert(get('usageRows').innerHTML.includes('&lt;img'));
run('resetData()');consume({...usageHeader,schema:'hand-diagnostic/v1'});run('registerUsage("diagnostic.jsonl")');
assert.equal(run('usageRecords.length'),2);
// A3.2 host clock is kept separately from the monotonic plot/control clock.
// An adjusted wall clock must not change control periods or CSV ordering.
run('resetData()');
const sessionStart='2026-09-28T14:00:00.000+08:00',epoch=Date.parse(sessionStart);
consume({type:'session',schema:'hand-control-session/v2',started_at:sessionStart,
  metadata:{model_id:'TsenseI_v3_2_withid',hand_side:'left',telemetry_format:'a32-native-sdk-state/v1'}});
consume({type:'conversion',t:1,seq:2,hand_side:'left',linear_mm:[1],rotary_deg:[]});
consume({type:'downlink',t:1,seq:3,command_id:2,hand_side:'left',sent:true,linear_mm:[1],wall_time_unix_ms:epoch+1001.25});
consume({type:'h7',t:1.001,seq:4,command_id:2,hand_side:'left',joints_ui:{left_hand_mid_PIP_joint:.1},sent:true,control_dt_s:.01});
consume({type:'downlink',t:1.01,seq:5,hand_side:'left',sent:false,linear_mm:[2]});
consume({type:'downlink',t:1.02,seq:6,hand_side:'left',sent:true,linear_mm:[3],wall_time:'2026-09-28T13:59:01.020+08:00'});
consume({type:'h7',t:1.021,seq:7,hand_side:'left',joints_ui:{},control_period_ms:20});
consume({type:'h7',t:1.04,seq:8,hand_side:'left',joints_ui:{},control_dt_s:-.1});
consume({type:'sdk_error',t:2,seq:9,hand_side:'left',error:'synthetic stopped stream'});
get('motor').value='linear:0';run('selectLoadedData();inspectFrame(1)');
assert.equal(run('eventWallMillis(state.downlink[0])'),epoch+1001.25);
assert.equal(run('eventWallMillis(state.downlink[2])'),Date.parse('2026-09-28T05:59:01.020Z'));
assert.equal(run('eventWallSource(state.downlink[0])'),'recorded_host_time');
assert.equal(run('eventWallSource(state.conversion[0])'),'derived_from_started_at_and_t');
assert.equal(run('eventWallMillis(state.conversion[0])'),epoch+1000);
assert.equal(run('eventWallSource({t:1,wall_time_unix_ms:null,wall_time:"bad"})'),'derived_from_started_at_and_t');
assert.equal(run('wallIso(eventWallMillis({at:"2026-09-28T06:00:02Z"}))'),'2026-09-28T06:00:02.000Z');
assert.equal(JSON.parse(get('frameDetail').textContent).selected_wall_time,'2026-09-28T06:00:01.000Z');
assert.equal(JSON.parse(get('frameDetail').textContent).selected_wall_time_source,'derived_from_started_at_and_t');
assert.equal(JSON.parse(get('frameDetail').textContent).command_sample.control_dt_s,.01);
assert.equal(JSON.parse(get('frameDetail').textContent).conversion.seq,2);
assert.equal(run('makeControlPeriodSeries()[0].points.length'),2);
assert.equal(run('makeControlPeriodSeries()[0].points[0][1]'),10);
assert(Math.abs(run('makeControlPeriodSeries()[1].points[0][1]')-20)<1e-8);
assert.equal(run('makeMotorSeries().position[1].name'),'SDK 写入成功');
assert(get('sessionStats').innerHTML.includes('15.000 ms'));
assert(get('sessionStats').innerHTML.includes('SDK 错误'));
assert(get('sessionStats').innerHTML.includes('2.000 s'));
assert(get('sessionTimeRange').textContent.includes(sessionStart));
assert(get('dataNotice').textContent.includes('推算并标明来源'));
assert(get('writeSemantics').textContent.includes('SDK sendOnce'));
assert(get('writeSemantics').textContent.includes('不是固件逐条确认'));
assert(!get('sessionStats').innerHTML.includes('H7 串口'));
assert.equal(get('h7StageLabel').textContent,'SDK 下发（关节空间）');
get('viewStart').value='1.005';get('viewEnd').value='1.015';get('applyTimeRange').click();
assert.equal(run('chartModels.get($("controlPeriodCanvas")).xmin'),1.005);
assert.equal(run('chartModels.get($("positionCanvas")).xmax'),1.015);
get('viewEnd').value='1';get('applyTimeRange').click();
assert(get('status').textContent.includes('有效时间范围'));
get('exportMotor').click();
const a32Export=exported;
run('resetData()');
assert.equal(run('eventWallMillis({t:1})'),null);
assert.equal(run('eventWallSource({t:1})'),'unavailable');
assert.equal(run('makeControlPeriodSeries()[0].points.length'),0);
assert.equal(run('makeControlPeriodSeries()[1].points.length'),0);
const missingTelemetryExports=[];
for(const model of ['TsenseI_v3_2','TsenseI_v3_2_withid']){
  run('resetData()');
  consume({type:'session',schema:'hand-control-session/v2',started_at:sessionStart,
    metadata:{model_id:model,hand_side:'left',slot_maps:{linear:{left_hand_pinky_DIP_joint:0},rotary:{left_hand_pinky_swing_joint:0}}}});
  // Match native SDK/telemetry_event shape: target is raw, while normalized
  // position/target/current/temperature are null for offline or stale channels.
  const motor=(value,mode)=>{
    const usable=mode==='valid';
    return {online:mode!=='offline',measurement_valid:usable,position:value,target:value+.1,homing:254,temperature_centi:3500,
      pos_act_mm:usable?value:null,pos_actual_mm:usable?value:null,pos_ref_mm:usable?value+.1:null,
      iq_meas:usable?value+1:null,iq:usable?value+1:null,iq_ref:usable?3:null,grav_iq:usable?1:null,mcu_temp_c:usable?35:null};
  };
  for(const [t,mode] of [[0,'valid'],[1,'offline'],[2,'valid'],[3,'stale'],[4,'valid'],[6,'valid'],[7,'missing'],[8,'valid']]){
    const slot=motor(['offline','stale'].includes(mode)?99:t+1,mode);
    if(t===0){slot.target=0;slot.pos_ref_mm=0;}
    assert(!Object.hasOwn(slot,'pos_ref'));
    consume({type:'telemetry',t,seq:t+2,hand_side:'left',accepted:true,telemetry_age_ms:mode==='stale'?700:1,
      linear:mode==='missing'?[]:[slot],rotary:mode==='missing'?[]:[slot]});
  }
  // No new wire sequence during this outage: only status was sampled.
  consume({type:'sdk_status',t:5,seq:7,hand_side:'left',sdk_state:{connected:true,telemetry_age_ms:1000}});
  get('motor').value='linear:0';run('selectLoadedData()');
  for(const selection of ['linear:0','rotary:0']){
    get('motor').value=selection;
    assert.equal(run('JSON.stringify(seriesSegments(makeMotorSeries().position[3]).map(segment=>segment.length))'),'[1,1,1,1,1]');
    assert.equal(run('seriesSegments(makeMotorSeries().position[2]).length'),5);
    assert.equal(run('seriesSegments(makeMotorSeries().current[0]).length'),5);
    assert.equal(run('seriesSegments(makeMotorSeries().temperature[0]).length'),5);
    for(const gap of [1,3,5,7])assert.equal(run(`makeMotorSeries().position[3].sampleAt(${gap})`),null);
    assert.equal(run('makeMotorSeries().position[3].sampleAt(4)[1]'),selection==='linear:0'?-5:5);
    // Zoom entirely into a gap: interpolation cannot manufacture a ramp.
    run('drawChart($("positionCanvas"),[makeMotorSeries().position[3]],"","mm");applyChartView($("positionCanvas"),{xmin:.2,xmax:1.8,ymin:0,ymax:10},{linkTime:false})');
    assert.equal(run('chartModels.get($("positionCanvas")).visibleMin'),null);
  }
  get('motor').value='linear:0';get('exportMotor').click();missingTelemetryExports.push(exported);
}
// A3.2 old/new service rows can be mixed without double-negating normalized
// positions. All 14 linear axes change sign; raw evidence and rotary axes do not.
for(const model of ['TsenseI_v3_2','TsenseI_v3_2_withid']){
  run('resetData()');consume({type:'session',metadata:{model_id:model,linear_feedback_sign:-1}});
  const legacy={type:'telemetry',t:0,linear:Array.from({length:14},(_,i)=>({
    online:true,measurement_valid:true,position:-i-.25,target:-i-.5,
    pos_act_mm:-i-.25,pos_ref_mm:-i-.5,pos_err_mm:-.25,iq_meas:-3,iq_ref:4})),
    rotary:Array.from({length:6},(_,i)=>({position:i+.25,target:i+.5,pos_actual_mm:i+.25,pos_ref_mm:i+.5})),
    sdk_state:{self:[{position:-.25,target:-.5}]}};
  const original=JSON.stringify(legacy);
  consume(legacy);
  const normalized=JSON.parse(run('JSON.stringify(state.telemetry[0])'));
  assert.equal(normalized.linear_feedback_sign,-1);
  assert.equal(normalized.linear.length,14);
  for(const [i,slot] of normalized.linear.entries()){
    assert.equal(slot.pos_act_mm,i+.25);assert.equal(slot.pos_ref_mm,i+.5);assert.equal(slot.pos_err_mm,.25);
    assert.equal(slot.position,-i-.25);assert.equal(slot.target,-i-.5);
    assert.equal(slot.iq_meas,-3);assert.equal(slot.iq_ref,4);
  }
  assert.deepEqual(normalized.rotary,legacy.rotary);assert.deepEqual(normalized.sdk_state,legacy.sdk_state);
  consume({...normalized,t:1});
  assert.deepEqual(JSON.parse(run('JSON.stringify(state.telemetry[1].linear)')),normalized.linear);
  assert.equal(run('normalizeA32Telemetry(state.telemetry[0])===state.telemetry[0]'),true);
  assert.equal(JSON.stringify(legacy),original);
}
// Captured browser batching: transport rows used receipt time while canonical
// SDK angles retained per-write times. The header was created 13 ms after t=0.
run('resetData()');
const batchHeader={type:'session',t:.013,started_at:'2026-10-09T11:48:07.132Z',
  metadata:{model_id:'TsenseI_v3_2_withid',hand_side:'left'}};
consume(batchHeader);
const batchOrigin=1791546487119;
const batchedWrites=[1791546499114.0715,1791546499116.0933].map((sent_at_unix_ms,i)=>({
  type:'h7',t:12.032,seq:3423+i*3,tick:501+i,sample_ticks:501+i,source:'a32_sdk_successful_write',
  sent:true,sent_at_unix_ms,self:[10.105538496415798-i*.009973616883503],mit_pos:[0,0]}));
for(const row of batchedWrites){
  consume(row);
  consume({type:'joint_angles',stage:'sdk',t:(row.sent_at_unix_ms-batchOrigin)/1000,seq:row.seq+2,
    tick:row.tick,sent:true,source:row.source,sent_at_unix_ms:row.sent_at_unix_ms,joints:{left_hand_pinky_DIP_joint:.5}});
}
run('selectLoadedData()');
assert.equal(run('state.downlink.length'),2);
assert.equal(run('state.downlink[0].t'),(batchedWrites[0].sent_at_unix_ms-batchOrigin)/1000);
assert.equal(run('state.downlink[1].t'),run('angleStages.sdk[1].t'));
assert.equal(run('state.h7[0].t'),run('state.downlink[0].t'));
assert.equal(run('state.downlink[0].recorded_t'),12.032);
assert.equal(run('state.downlink[0].self[0]'),batchedWrites[0].self[0]);
assert.equal(batchedWrites[0].t,12.032);
assert(get('dataNotice').textContent.includes('还原 2 条 SDK 下发记录'));
const restoredDownlink=run('JSON.stringify(state.downlink)');
run('selectLoadedData()');assert.equal(run('JSON.stringify(state.downlink)'),restoredDownlink);
// Legacy files without canonical SDK angles use the session clock origin.
run('resetData()');consume(batchHeader);batchedWrites.forEach(consume);run('selectLoadedData()');
assert.equal(run('state.downlink[0].t'),(batchedWrites[0].sent_at_unix_ms-batchOrigin)/1000);
// Already fixed downloads need no correction marker.
run('resetData()');consume(batchHeader);
consume({...batchedWrites[0],t:(batchedWrites[0].sent_at_unix_ms-batchOrigin)/1000});run('selectLoadedData()');
assert.equal(run('state.downlink[0].recorded_t'),undefined);

// Browser A3.2 downloads use native SDK snapshots and joint-keyed conversions.
run('resetData()');
consume({type:'session',metadata:{model_id:'TsenseI_v3_2_withid',hand_side:'left'}});
const browserChannels=JSON.parse(fs.readFileSync(path.join(__dirname,'../src/a32_driver/a32_hardware.json'),'utf8')).channels
  .sort((a,b)=>(a.channel!=='self')-(b.channel!=='self')||a.index-b.index);
assert.deepEqual(JSON.parse(run('JSON.stringify(a32Channels())')),browserChannels.map(({joint,channel,index,unit})=>({joint,channel,index,unit})));
const browserHardware={connected:true,telemetry_age_ms:10,sequence:42,self:[],mit:[],hand_side:'left'};
for(const c of browserChannels)browserHardware[c.channel][c.index]={joint:c.joint,online:true,valid:true,
  position:c.index+.25,target:0,iq:3,iq_ref:4,temperature_centi:3500,velocity_rpm:2,voltage:12};
const parsedBrowser=JSON.parse(run(`JSON.stringify(a32Telemetry(${JSON.stringify({type:'a32_sdk_sample',hardware:browserHardware})}))`));
assert.equal(parsedBrowser.linear_feedback_sign,-1);
assert.equal(parsedBrowser.linear.length,14);
for(const slot of parsedBrowser.linear){
  assert.equal(slot.pos_act_mm,-slot.position);assert.equal(slot.pos_actual_mm,-slot.position);
  assert.equal(slot.pos_ref_mm,0);assert.equal(slot.iq_meas,3);
}
for(const slot of parsedBrowser.rotary)assert.equal(slot.pos_actual_mm,slot.position);
const browserConversion={type:'a32_conversion',seq:2,t:1,frame_id:7,valid:true,
  joint_angles_rad:{left_hand_pinky_DIP_joint:.5},linear_strokes_mm:{},rotary_positions_deg:{}};
for(const [i,c] of browserChannels.entries())browserConversion[c.unit==='mm'?'linear_strokes_mm':'rotary_positions_deg'][c.joint]=i+1;
consume(browserConversion);
consume({type:'a32_sdk_sample',seq:3,t:1.1,sent:true,hardware:browserHardware});
assert.equal(run('state.downlink.length'),0); // Polling sent=true is not a new host write.
assert.equal(run('state.h7.length'),0);
get('motor').value='linear:0';run('selectLoadedData();inspectFrame(1)');
assert.equal(get('motor').options.length,20);
assert(get('motor').options[0].text.includes('pinky_DIP'));
assert.equal(run('makeMotorSeries().position[0].points[0][1]'),1);
assert.equal(run('makeMotorSeries().position[2].points[0][1]'),0); // A measured zero stays zero.
assert.equal(run('makeMotorSeries().position[3].points[0][1]'),-.25);
assert.equal(run('makeMotorSeries().temperature[0].points[0][1]'),35);
assert.equal(JSON.parse(get('frameDetail').textContent).conversion.source_type,'a32_conversion');
assert.equal(JSON.parse(get('frameDetail').textContent).sdk_status.hardware.sequence,42);
get('motor').value='rotary:4';
assert.equal(run('makeMotorSeries().position[0].points[0][1]'),19); // MIT 0, thumb back, not self slot 4.
assert.equal(run('makeMotorSeries().position[3].points[0][1]'),.25);
assert.equal(run('makeMotorSeries().position[2].points[0][1]'),0);
const invalidHardware=JSON.parse(JSON.stringify(browserHardware));
invalidHardware.self[0].online=false;invalidHardware.mit[0].valid=false;
consume({type:'a32_sdk_sample',seq:4,t:2,hardware:invalidHardware});
consume({type:'a32_sdk_sample',seq:5,t:3,hardware:{...browserHardware,telemetry_age_ms:700}});
consume({type:'a32_sdk_sample',seq:6,t:4,hardware:{connected:false,self:[],mit:[],telemetry_age_ms:-1}});
consume({type:'a32_sdk_sample',seq:7,t:5,hardware:browserHardware});
for(const selection of ['linear:0','rotary:4']){
  get('motor').value=selection;
  assert.equal(run('makeMotorSeries().position[3].points.length'),2);
  assert.equal(run('seriesSegments(makeMotorSeries().position[3]).length'),2);
  for(const t of [2,3,4])assert.equal(run(`makeMotorSeries().position[3].sampleAt(${t})`),null);
}
const wireCommand={self:Array.from({length:18},(_,i)=>i+10),mit_pos:[81,82]};
for(const [i,t] of [5.1,5.2].entries())consume({type:'h7',seq:8+i,t,frame_id:7,source:'a32_sdk_successful_write',
  ...wireCommand,sent:true,joints_ui:{left_hand_pinky_DIP_joint:.5},sent_at_unix_ms:epoch+100+i*2,
  control_timing:{control_dt_ms:2,host_write_interval_ms:2}});
assert.equal(run('state.downlink.length'),2);
assert.equal(run('state.h7.length'),2);
assert.equal(run('makeMotorSeries().position[1].points[0][1]'),81);
assert.equal(run('makeControlPeriodSeries()[1].points[0][1]'),2); // Actual write interval, not browser poll interval.
assert.equal(run('makeControlPeriodSeries()[0].points.length'),2);
consume({type:'a32_sdk_action',t:6,action:'disconnect',hardware:{connected:false}});
consume({type:'a32_sdk_recording_gap',t:6,count:12});
run('inspectFrame(5.1);drawAll(true)');
assert.equal(JSON.parse(get('frameDetail').textContent).conversion.frame_id,7);
assert.equal(JSON.parse(get('frameDetail').textContent).sdk_event.action,'disconnect');
assert(get('dataNotice').textContent.includes('遗漏 12 条'));
get('motor').value='linear:0';get('exportMotor').click();
const browserExport=exported;
browserExport.text().then(csv=>{
  const rows=csv.replace(/^\uFEFF/,'').split(/\r?\n/).map(line=>line.split(',').map(value=>value.replace(/^"|"$/g,'')));
  const [header,...events]=rows,index=Object.fromEntries(header.map((name,i)=>[name,i]));
  const offline=events.find(r=>r[index.t_s]==='2');
  assert.equal(offline[index.actual_position],'');assert.equal(offline[index.raw_position],'0.25');
  assert.equal(events.filter(r=>r[index.type]==='downlink').length,2);
}).catch(error=>{console.error(error);process.exitCode=1;});
consume({...browserConversion,t:7,valid:false});
assert.equal(run('state.conversion.at(-1).linear_mm[0]'),null);
run('resetData()');consume({type:'session',metadata:{model_id:'TsenseI_v3_2'}});
consume({type:'session_end',t:1,reason:'启动失败，请重试'});run('selectLoadedData()');
assert(get('dataNotice').textContent.includes('文件已读取，但没有动作或电机回传数据'));
assert(get('dataNotice').textContent.includes('启动失败，请重试'));

// A3.0 preview conversions and missing/offline motors, both asset variants and hands.
for(const model of ['TsenseI_v3_0','TsenseI_v3_0_withid'])for(const side of ['left','right']){
  run('resetData()');consume({type:'session',metadata:{model_id:model,hand_side:side}});
  const maps=JSON.parse(run(`JSON.stringify(A3Telemetry.telemetryMetadata('${side}').slot_maps)`));
  const joints=Object.fromEntries(Object.values(maps).flatMap(m=>Object.keys(m).map(n=>[n,.2])));
  consume({type:'a30_conversion',t:0,seq:2,hand_side:side,valid:true,joint_angles_rad:joints,
    linear_strokes_mm:Object.fromEntries(Object.keys(maps.linear).map(n=>[n,1])),rotary_positions_deg:Object.fromEntries(Object.keys(maps.rotary).map(n=>[n,2]))});
  for(const [t,online] of [[0,true],[1,false],[2,true]])consume({type:'telemetry',t,hand_side:side,accepted:true,
    linear:[{online,pos_act_mm:0,pos_ref_mm:1,iq_meas:2,mcu_temp_c:35}],rotary:[{online,pos_actual_mm:0,iq:2}]});
  run('selectLoadedData()');get('motor').value='linear:0';
  assert.equal(get('handSide').value,side);assert.equal(get('motor').options.length,20);
  assert.equal(run('makeMotorSeries().position[0].points[0][1]'),1);
  assert.equal(run('makeMotorSeries().position[3].points.length'),2);
  assert.equal(run('makeMotorSeries().position[3].sampleAt(1)'),null);
  assert.equal(run('seriesSegments(makeMotorSeries().position[3]).length'),2);
  assert.equal(run('makeMotorSeries().current[0].sampleAt(1)'),null);
  get('motor').value='rotary:0';assert.equal(run('makeMotorSeries().position[0].points[0][1]'),2);
  consume({type:'serial_disconnect',t:3,hand_side:side,first_error:'synthetic lost link'});
  run('inspectFrame(3)');assert.equal(JSON.parse(get('frameDetail').textContent).other_event.first_error,'synthetic lost link');
}
// Five joint-angle stages coexist with the original auxiliary data panels.
assert.deepEqual([...html.matchAll(/id="(stage[^\"]+)"/g)].map(m=>m[1]),
  ['stageRetarget','stageDirect','stageRuckig','stageMink','stageH7']);
assert.equal(run('eventWallMillis({t:1,sent_at_unix_ms:1780000000123.5})'),1780000000123.5);
assert.equal(run('eventWallSource({t:1,sent_at_unix_ms:1780000000123.5})'),'recorded_host_time');
for(const stage of ['stageControl','stageFilterCandidate','stageConversion','stageMediaPipeAngle','stageDirectFilter'])assert(!html.includes(`id="${stage}"`));
for(const model of ['TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid','TsenseI_v2'])for(const side of ['left','right']){
  run('resetData()');get('finger').value='index';
  consume({type:'session',schema:'hand-control-session/v2',started_at:sessionStart,metadata:{model_id:model,hand_side:side}});
  for(const [i,stage] of ['retarget','direct','ruckig','mink','sdk'].entries()){
    consume({type:'joint_angles',stage,t:2+i*.01,frame_id:99,model_id:model,hand_side:side,
      joints:{[`${side}_hand_index_swing_joint`]:i*.1},...(stage==='sdk'?{sent:true}:{}),landmarks:['not an angle payload']});
  }
  for(const sent of [false,undefined,1])consume({type:'joint_angles',stage:'sdk',t:3,hand_side:side,joints:{[`${side}_hand_index_swing_joint`]:9},sent});
  consume({type:'a32_sdk_sample',t:3,hand_side:side,sent:true,hardware:{connected:false}});
  consume({type:'control_target',t:3,hand_side:side,joints:{[`${side}_hand_index_swing_joint`]:9}});
  consume({type:'conversion',t:3,hand_side:side,joints_hw:{[`${side}_hand_index_swing_joint`]:9},linear_mm:[2],rotary_deg:[3]});
  consume({type:'telemetry',t:3,hand_side:side,accepted:true,linear:[{pos_act_mm:1,pos_ref_mm:2,iq_meas:4,mcu_temp_c:35}],rotary:[]});
  run('selectLoadedData();inspectFrame(2)');
  assert.equal(get('handSide').value,side);
  assert.equal(run('makeJointSeries().length'),5);
  assert.equal(run('makeJointSeries().filter(s=>s.points.length===1).length'),5);
  assert.equal(run('angleStages.sdk.length'),1);
  assert.equal(run('angleStages.mink.length'),1);
  assert.equal(run('angleStages.retarget[0].landmarks'),undefined);
  assert.equal(run('state.conversion.length'),1);assert.equal(run('state.telemetry.length'),1);
  assert.equal(run('chartModels.get($("jointCanvas")).fullXmax'),3);
  const detail=JSON.parse(get('frameDetail').textContent);
  assert.equal(detail.joint_angle_stages.sdk.sent,true);assert.equal(detail.joint_angle_stages.retarget.frame_id,99);
  assert(!get('dataNotice').textContent.includes('文件已读取，但没有动作'));
  get('exportJoints').click();
}
const jointExport=exported;
run('resetData()');get('finger').value='index';
consume({type:'session',metadata:{model_id:'TsenseI_v3_2',hand_side:'right'}});
consume({type:'joint_angles',stage:'ruckig',t:4,model_id:'TsenseI_v3_2',joints:{left_hand_index_swing_joint:0}});
consume({type:'joint_angles',stage:'ruckig',t:4.1,joints:{left_hand_index_swing_joint:null}});
consume({type:'joint_angles',stage:'ruckig',t:4.2,joints:{left_hand_index_swing_joint:.2}});
consume({type:'joint_angles',stage:'conversion',t:4.3,joints:{left_hand_index_swing_joint:4}});
run('selectLoadedData()');
assert.equal(get('handSide').value,'left');
assert.equal(run('angleStages.ruckig.length'),3);
assert.equal(run('JSON.stringify(seriesSegments(makeJointSeries().find(s=>s.stage==="ruckig")).map(s=>s.length))'),'[1,1]');
assert.equal(run('makeJointSeries().find(s=>s.stage==="ruckig").sampleAt(4.1)'),null);
assert(get('dataNotice').textContent.includes('Mink 输出'));
assert(get('dataNotice').textContent.includes('会话头'));
assert.equal(run('angleStages.mink.length'),0);
assert.equal(run('angleStages.sdk.length'),0);
consume({type:'direct_command',t:5,hand_side:'left',joints:{left_hand_index_swing_joint:.3}});
consume({type:'h7',t:5.01,hand_side:'left',direct_mode:false,sent:false,joints_ui:{left_hand_index_swing_joint:8},joints_ruckig_ui:{left_hand_index_swing_joint:.4}});
consume({type:'h7',t:5.02,hand_side:'left',direct_mode:true,sent:true,joints_ui:{left_hand_index_swing_joint:.5},joints_ruckig_ui:{left_hand_index_swing_joint:8}});
assert.equal(run('angleStages.direct.length'),1);
assert.equal(run('angleStages.ruckig.at(-1).joints.left_hand_index_swing_joint'),.4);
assert.equal(run('angleStages.sdk.length'),1);
assert.equal(run('angleStages.sdk[0].joints.left_hand_index_swing_joint'),.5);
run('resetData()');assert.equal(run('Object.values(angleStages).flat().length'),0);
jointExport.text().then(csv=>{
  assert(csv.includes('"stage","t_s","frame_id","hand_side","model_id","joint","angle_rad","sent"'));
  const rows=csv.replace(/^\uFEFF/,'').split(/\r?\n/);
  assert.equal(rows.length,6);assert.equal(rows.filter(r=>r.startsWith('"sdk"')).length,1);
  assert(!csv.includes('landmarks'));
}).catch(error=>{console.error(error);process.exitCode=1;});

// SDK 1.1.15 values are already amperes. Preserve old SELF values/units even
// under a newer header, and never join different units into one curve.
const currentUnitExports=[];
for(const model of ['TsenseI_v3_2','TsenseI_v3_2_withid']){
  run('resetData()');
  consume({type:'session',metadata:{model_id:model,hand_side:'left',telemetry_format:'a32-native-sdk-state/v2',
    telemetry_units:{self_current:'A',external_current:'A'}}});
  const sample=(t,format,unit,value)=>({type:'telemetry',t,hand_side:'left',linear_feedback_sign:-1,
    ...(format?{telemetry_format:format}:{}),linear:[{channel:'self',online:true,measurement_valid:true,
      position:1,pos_act_mm:-1,iq_meas:value,iq_ref:value*2,...(unit?{current_unit:unit}:{})}],rotary:[]});
  consume(sample(0,'a32-native-sdk-state/v1',null,-100));
  consume(sample(1,'a32-native-sdk-state/v2','A',-.1374));
  consume(sample(2,null,null,-120));
  // Re-exported legacy captures use v2 with explicit native units.
  consume(sample(3,'a32-native-sdk-state/v2','SDK native',-130));
  get('motor').value='linear:0';run('selectLoadedData()');
  assert.equal(run('motorCurrentUnit(state.telemetry[0],"linear",0)'),'SDK native');
  assert.equal(run('motorCurrentUnit(state.telemetry[1],"linear",0)'),'A');
  assert.equal(run('motorCurrentUnit(state.telemetry[2],"linear",0)'),'SDK native');
  assert.equal(run('motorCurrentUnit(state.telemetry[3],"linear",0)'),'SDK native');
  assert.equal(run('makeMotorSeries().current.find(s=>s.name==="实测 iq / A").points[0][1]'),-.1374);
  assert.equal(run('makeMotorSeries().current.find(s=>s.name==="实测 iq / A").sampleAt(2)'),null);
  assert.equal(run('JSON.stringify(seriesSegments(makeMotorSeries().current.find(s=>s.name==="实测 iq / 协议原值")).map(s=>s.length))'),'[1,2]');
  assert(run('motorCurrentAxisUnit()').includes('不可直接比较'));
  assert(get('dataNotice').textContent.includes('找零 Iq 参数仍为码值'));
  get('exportMotor').click();currentUnitExports.push(exported);
  const modern={connected:true,telemetry_age_ms:1,telemetry_format:'a32-native-sdk-state/v2',
    telemetry_units:{self_current:'A',external_current:'A'},self:[{online:true,position:1,iq:-.1374,iq_ref:.2748,current_unit:'A'}]};
  consume({type:'a32_sdk_sample',t:4,hardware:modern});
  assert.equal(run('motorCurrentUnit(state.telemetry.at(-1),"linear",0)'),'A');
  assert.equal(run('state.telemetry.at(-1).linear[0].iq_meas'),-.1374);
}
Promise.all(currentUnitExports.map(blob=>blob.text())).then(csvs=>{
  for(const csv of csvs){
    const [header,...events]=csv.replace(/^\uFEFF/,'').split(/\r?\n/).map(line=>line.split(',').map(value=>value.replace(/^"|"$/g,'')));
    const index=Object.fromEntries(header.map((name,i)=>[name,i]));
    assert.equal(events.length,4);
    for(const t of [0,2,3]){
      const row=events.find(event=>event[index.t_s]===String(t));
      assert.equal(row[index.current_unit],'SDK native');
      assert.equal(row[index.iq_measured_native],String([-100,0,-120,-130][t]));
      assert.equal(row[index.iq_measured_a],'');
    }
    const row=events.find(event=>event[index.t_s]==='1');
    assert.equal(row[index.current_unit],'A');assert.equal(row[index.iq_measured_a],'-0.1374');
    assert.equal(row[index.iq_ref_a],'-0.2748');assert.equal(row[index.iq_measured_native],'');
    assert.equal(row[index.iq_ref_native],'');
  }
}).catch(error=>{console.error(error);process.exitCode=1;});

Promise.all([a32Export.text(),...missingTelemetryExports.map(blob=>blob.text())]).then(([csv,...gapCsvs])=>{
  assert(csv.includes('"wall_time","wall_time_unix_ms","wall_time_source","control_period_ms"'));
  assert(csv.includes('"2026-09-28T06:00:01.001Z","'+(epoch+1001.25)+'","recorded_host_time"'));
  assert(csv.includes('"derived_from_started_at_and_t","10"'));
  assert(csv.indexOf('"2026-09-28T06:00:01.001Z"')<csv.indexOf('"2026-09-28T05:59:01.020Z"'));
  for(const gapCsv of gapCsvs){
    const rows=gapCsv.replace(/^\uFEFF/,'').split(/\r?\n/).map(line=>line.split(',').map(value=>value.replace(/^"|"$/g,'')));
    const [header,...events]=rows,index=Object.fromEntries(header.map((name,i)=>[name,i]));
    assert.equal(events.length,8); // Status markers do not become fabricated telemetry.
    for(const t of [1,3,7]){
      const row=events.find(event=>event[index.t_s]===String(t));
      for(const key of ['target_position','actual_position','iq_measured_native','iq_ref_native','grav_iq_native','mcu_temp_c'])assert.equal(row[index[key]],'');
      assert.equal(row[index.measurement_valid],'false');
      assert.equal(row[index.raw_position],t===7?'':'99'); // Cached source survives for investigation.
      assert.equal(row[index.raw_target],t===7?'':'99.1');
    }
    assert.equal(events.find(event=>event[index.t_s]==='4')[index.actual_position],'-5');
    assert.equal(events.find(event=>event[index.t_s]==='4')[index.raw_target],'5.1');
    assert.equal(events.find(event=>event[index.t_s]==='0')[index.raw_target],'0');
  }
  console.log('Control log viewer and usage tests passed');
}).catch(error=>{console.error(error);process.exitCode=1;});
