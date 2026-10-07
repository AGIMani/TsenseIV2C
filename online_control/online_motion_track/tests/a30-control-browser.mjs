// Actual public server and solver, fake camera/landmarks and in-memory A3 H7.
import {chromium} from "@playwright/test";
import {execFileSync} from "node:child_process";
import {mkdir,readFile,writeFile} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import assert from "node:assert/strict";
const root=fileURLToPath(new URL("../../../",import.meta.url)),out=process.env.TEST_OUTPUT_DIR||`${root}/logs/a30_control_browser`;
const skipHome=process.env.MOCK_SKIP_HOME==="1";
const partialFeedback=process.env.MOCK_PARTIAL_FEEDBACK==="1";
const selfOnly=process.env.MOCK_SELF_ONLY==="1";
const clippedFeedback=process.env.MOCK_CLIPPED_FEEDBACK==="1";
const collisionPause=process.env.MOCK_COLLISION_PAUSE==="1";
const continuous=process.env.MOCK_CONTINUOUS==="1";
const directPreviewTest=process.env.MOCK_DIRECT_PREVIEW==="1";
const model=process.env.HAND_MODEL||'TsenseI_v3_0';
await mkdir(out,{recursive:true});
const world=JSON.parse(execFileSync(process.env.HAND_PYTHON||"python",["-B","-X","utf8","-c",
  "import sys,json;sys.path[:0]=['.','src'];from hand_profiles import load_hand_profile;print(json.dumps(load_hand_profile(sys.argv[1]).create_retargeter().fk.compute().tolist()))",model],{cwd:root,encoding:"utf8",windowsHide:true}));
const recordedHands=process.env.MOCK_RECORDED_HAND?JSON.parse(await readFile(process.env.MOCK_RECORDED_HAND,'utf8')):null;
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH||"C:/Program Files/Google/Chrome/Application/chrome.exe",headless:true,
  args:["--use-fake-ui-for-media-stream","--use-fake-device-for-media-stream"]});
const page=await browser.newPage({ignoreHTTPSErrors:process.env.TEST_IGNORE_TLS==="1",viewport:{
  width:Number(process.env.TEST_VIEWPORT_WIDTH)||1440,height:Number(process.env.TEST_VIEWPORT_HEIGHT)||1100}}),errors=[],badCalls=[];
const screenshot=options=>process.env.TEST_SKIP_SCREENSHOTS==='1'?Promise.resolve():page.screenshot(options);
const controlResponses=[];
page.on('response',async response=>{
  if(!response.url().includes('/api/a30/'))return;
  try{
    const request=response.request(),body=request.postDataJSON(),state=await response.json();
    controlResponses.push({at:Date.now(),path:new URL(response.url()).pathname,status:response.status(),request:body,
      state:{error:state.error,last_error:state.last_error,direct_mode:state.direct_mode,browser_epoch:state.browser_epoch,
        sample_ticks:state.sample_ticks,stream_active:state.stream_active,tracking_transition:state.tracking_transition,
        target_ready:state.target_ready,sample_collision:state.sample_collision}});
    if(controlResponses.length>100)controlResponses.shift();
  }catch{}
});
page.on("pageerror",e=>errors.push(e.message));
if(continuous)await page.route('**/api/a30/control',async route=>{
  if(route.request().postDataJSON()?.resume_from_commanded_pose)
    await new Promise(r=>setTimeout(r,250)); // Rebase legitimately exceeds the old 150 ms watchdog.
  await route.continue();
});
if(process.env.LOCAL_STREAM_JS==='1')await page.route('**/a30-browser-hand-control.js',async route=>route.fulfill({contentType:'text/javascript',body:await readFile(new URL('../static/a30-browser-hand-control.js',import.meta.url),'utf8')}));
if(process.env.MOCK_SKIP_RENDER==='1')await page.route('**/viewer.js',async route=>{
  const response=await route.fetch();
  // Keep the production model, beforeRender callback and display sample checks;
  // omit GPU drawing when isolating host scheduling from headless raster cost.
  await route.fulfill({response,body:(await response.text()).replace('this.renderer.render(this.scene, this.camera);','/* headless timing isolation: no GPU draw */')});
});
page.on("request",r=>{if(r.url().includes("/api/v2/control"))badCalls.push(r.url());});
await page.addInitScript(({world,recordedHands,partialFeedback,selfOnly,clippedFeedback})=>{
  window.mockWrites=[];window.mockMissing=false;window.mockTelemetry=false;
  const NativeWorker=window.Worker;
  window.Worker=class{
    constructor(url,options){if(!String(url).includes('detector.worker'))return new NativeWorker(url,options);}
    postMessage(d){d.bitmap?.close();const current=recordedHands?recordedHands[(this.frame=(this.frame??-1)+1)%recordedHands.length]:world;
      this.timer=setTimeout(()=>this.onmessage?.({data:d.type==="init"?{type:"ready",delegate:"GPU"}:
      {type:"result",inference_ms:1,result:window.mockMissing?{landmarks:[],worldLandmarks:[],handedness:[]}:
        {handedness:[[{categoryName:"Left",score:.99}]],worldLandmarks:[current.map(([x,y,z])=>({x,y,z}))],landmarks:[current.map(([x,y,z])=>({x:x+.5,y:y+.5,z}))]}}}),25);}
    terminate(){clearTimeout(this.timer);}
  };
  const crc=b=>{let c=65535;for(const x of b){c^=x;for(let i=0;i<8;i++)c=c&1?(c>>>1)^0xa001:c>>>1;}return c;};
  const port={getInfo:()=>({usbVendorId:0x1209,usbProductId:0xa30}),async open(){
    let controller,seq=0;const state=Array(18).fill(0);const positions=Array(18).fill(0);const ext=[0,0];
    this.readable=new ReadableStream({start:c=>controller=c,cancel:()=>clearInterval(this.timer)});
    this.writable=new WritableStream({write:b=>{
      window.mockWrites.push(Array.from(b));
      if(b[5]===0x22){const slot=b[6]===0?b[7]-1:b[6]===1?b[7]+7:b[7]+15;state[slot]=b[8]===2?0:slot===4?3:2;}
      if(b[5]===0x20){const v=new DataView(b.buffer,b.byteOffset,b.byteLength);for(let i=0;i<18;i++)positions[i]=v.getFloat32(7+4*i,true);ext[0]=v.getFloat32(79,true);ext[1]=v.getFloat32(83,true);}
    }});
    this.timer=setInterval(()=>{
      if(!window.mockTelemetry)return;
      const b=new Uint8Array(492),v=new DataView(b.buffer);b.set([0xaa,0x55,0xe4,1,3,2]);
      v.setUint16(6,seq++,true);v.setUint16(8,3,true);v.setUint32(10,Math.floor(performance.now()),true);v.setUint32(14,0x3ffff,true);
      for(let i=0;i<18;i++){const o=18+i*24;b[o+3]=state[i];v.setFloat32(o+4,positions[i],true);v.setFloat32(o+16,positions[i],true);}
      if(clippedFeedback)v.setFloat32(22,-.16041614,true);
      for(let i=0;i<2;i++){const o=450+i*20;b[o]=5+4*i;b[o+1]=5;v.setUint32(o+2,Math.round(ext[i]/360*2097152)+0x80000000,true);v.setUint16(o+6,0x8000,true);v.setUint16(o+8,0x8000,true);b[o+16]=4;}
      if(partialFeedback){v.setUint32(14,0x3fbff,true);v.setUint16(8,0,true);b.fill(0,450,490);v.setFloat32(18+10*24+4,NaN,true);}
      const wire=selfOnly?b.slice(0,452):b,view=new DataView(wire.buffer);
      view.setUint16(2,selfOnly?444:484,true);
      view.setUint16(wire.length-2,crc(wire.subarray(0,-2)),true);controller.enqueue(wire);
      if(seq%8===1){
        const board=new Uint8Array(16),bv=new DataView(board.buffer);board.set([0xaa,0x55,8,0,3,3]);
        bv.setUint16(6,seq,true);bv.setInt16(8,2650,true);bv.setInt16(10,2700,true);bv.setUint16(12,24000,true);
        bv.setUint16(14,crc(board.subarray(0,-2)),true);controller.enqueue(board);
      }
    },25);
  },async close(){clearInterval(this.timer);}};
  Object.defineProperty(navigator,"serial",{value:{getPorts:async()=>[port],requestPort:async()=>port}});
},{world,recordedHands,partialFeedback,selfOnly,clippedFeedback});
try{
  await page.goto((process.env.WEB_URL||"http://127.0.0.1:8772")+"/?model="+encodeURIComponent(model));
  await page.waitForFunction(()=>!document.querySelector("#start").disabled);
  if(process.env.MEASURE_RATE==='1')await page.evaluate(async()=>{
    const {A30BrowserHandControl}=await import('/a30-browser-hand-control.js');
    const event=A30BrowserHandControl.prototype.event;
    window.testStreamEvents=[];
    A30BrowserHandControl.prototype.event=function(type,data){
      if(['trajectory_recovery','trajectory_recovered','control_disarm'].includes(type))
        window.testStreamEvents.push({at:performance.now(),type,...data});
      return event.call(this,type,data);
    };
  });
  if(directPreviewTest)await page.evaluate(async()=>{
    const {HandViewer}=await import('/viewer.js');
    const {A30BrowserHandControl}=await import('/a30-browser-hand-control.js');
    const accept=A30BrowserHandControl.prototype.streamSample;
    A30BrowserHandControl.prototype.streamSample=function(...args){window.directController=this;return accept.apply(this,args);};
    const original=HandViewer.prototype.setPose;
    window.directDraws=0;window.directMismatches=0;
    const {A30PreviewStream}=await import('/a30-preview-stream.js');
    const previewAccept=A30PreviewStream.prototype.accept;
    A30PreviewStream.prototype.accept=function(...args){previewAccept.apply(this,args);window.expectedDirectPose=this.latest?.pose;};
    HandViewer.prototype.setPose=function(pose){
      if(this.element.id==='model-stage'&&document.querySelector('#direct-preview').checked&&!window.directController?.armed&&window.expectedDirectPose){
        window.directDraws++;
        if(Object.keys(pose).some(n=>pose[n]!==window.expectedDirectPose[n]))window.directMismatches++;
      }
      return original.call(this,pose);
    };
  });
  if(continuous)await page.evaluate(async()=>{
    const {HandViewer}=await import('/viewer.js');
    const setPose=HandViewer.prototype.setPose;
    window.testModelSamples=0;window.testModelMismatches=0;
    HandViewer.prototype.setPose=function(pose){
      const control=window.testControl;
      if(control?.armed&&control.lastWrittenSample){
        window.testModelSamples++;
        if(Object.keys(pose).some(n=>pose[n]!==control.lastWrittenSample.pose[n]))window.testModelMismatches++;
      }
      return setPose.call(this,pose);
    };
    const {A30BrowserHandControl}=await import('/a30-browser-hand-control.js');
    const accept=A30BrowserHandControl.prototype.streamSample;
    A30BrowserHandControl.prototype.streamSample=function(bytes,...args){
      window.testControl=this;
      if(window.testStaleOnce){
        window.testStaleOnce=false;bytes=bytes.slice();
        const view=new DataView(bytes.buffer);
        view.setFloat64(16,view.getFloat64(16,true)-1,true);
      }
      return accept.call(this,bytes,...args);
    };
    const {WorkerRecorder}=await import('/worker-recorder.js');
    const exportFile=WorkerRecorder.prototype.export;
    window.testExportCount=0;
    WorkerRecorder.prototype.export=function(...args){
        if(window.testControl?.armed)throw new Error('Recording export blocked the armed control page');
        window.testExportCount++;
      return exportFile.apply(this,args);
    };
    const start=WorkerRecorder.prototype.start;
    WorkerRecorder.prototype.start=function(...args){
      window.testRecorder=this;
      return start.apply(this,args);
    };
  });
  if(collisionPause)await page.evaluate(async()=>{
    // Inject only the server's pause status. Keep the actual HTTP stream,
    // watchdog, serial writer, UI and recording code running in Chrome.
    const {A30BrowserHandControl}=await import('/a30-browser-hand-control.js');
    const accept=A30BrowserHandControl.prototype.streamSample;
    A30BrowserHandControl.prototype.streamSample=function(bytes,...args){
      if(window.mockCollisionPause){bytes=bytes.slice(0,108);new DataView(bytes.buffer).setUint32(24,4,true);}
      return accept.call(this,bytes,...args);
    };
  });
  assert(await page.locator("#hardware-panel").isVisible());
  await page.locator('input[data-joint="left_hand_index_DIP_joint"]').uncheck();
  if(directPreviewTest){
    // Lifecycle changes end the recording; their four-model coverage lives in
    // a3-direct-browser.mjs. Keep this hardware-mock run in one direct session.
    await page.locator('label:has(#direct-preview)').click();
    await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('直控已开启'));
    assert.equal(await page.locator('#collision-mode').inputValue(),'off');
    assert(await page.locator('#collision-mode').isDisabled());
  }else await page.locator("#collision-mode").selectOption(process.env.MOCK_COLLISION_MODE||"stop");
  await page.locator("#refresh-ports").click();
  assert.equal(await page.locator("#serial-port option").count(),2);
  await page.locator("#serial-port").selectOption("0");
  await page.locator("#connect-serial").click();
  await page.waitForFunction(()=>document.querySelector("#connect-serial").textContent==="断开串口");
  assert.match(await page.locator("#arm-blockers").innerText(),/首次开启请先接收一包/);
  await page.locator('input[data-joint="left_hand_mid_DIP_joint"]').uncheck();
  await page.locator('input[data-joint="left_hand_mid_DIP_joint"]').check();
  assert(await page.evaluate(()=>window.mockWrites.some(b=>b[5]===0x21&&b[6]===1&&b[7]===1&&b[8]===1)));
  assert(await page.evaluate(()=>!window.mockWrites.some(b=>b[5]===0x20)),"power switching before telemetry writes no position frame");
  await page.evaluate(()=>window.mockTelemetry=true);
  await page.waitForFunction(()=>document.querySelector("#serial-diagnostics").textContent.includes("载荷"));
  assert.match(await page.locator("#serial-diagnostics").innerText(),new RegExp(`载荷 ${selfOnly?444:484} B`));
  await page.waitForFunction(()=>document.querySelector("#serial-diagnostics").textContent.includes("VIN 24.00 V"));
  await page.waitForTimeout(100);
  if(!skipHome){await page.locator("#home-hand").click();
  await page.waitForFunction(()=>document.querySelector("#homing-result-dialog").open,{},{timeout:30000});
  assert.equal(await page.locator("#homing-result-rows tr").count(),4);
  await screenshot({path:`${out}/homing-result.png`,fullPage:true});
  await page.locator("#homing-result-close").click();}
  await page.evaluate(()=>{window.mockMissing=true;window.mockTelemetry=false;});
  await page.waitForTimeout(650);
  await page.locator("#start").click();
  await page.waitForFunction(()=>!document.querySelector("#arm-hand").disabled,{},{timeout:15000});
  await page.locator("#arm-hand").click();
  await page.waitForFunction(()=>document.querySelector("#control-stats").textContent.match(/本机写入 [1-9]/),{},{timeout:15000});
  assert.match(await page.locator("#serial-diagnostics").innerText(),/不阻断下发/);
  await page.waitForTimeout(700);
  assert.equal(await page.locator("#arm-hand").innerText(),"关闭实时下发");
  if(collisionPause){
    await page.evaluate(()=>window.mockCollisionPause=true);
    await page.waitForFunction(()=>document.querySelector('#arm-blockers').textContent.includes('手势安全后自动恢复'));
    const paused=await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length);
    await page.waitForTimeout(1200);
    assert.equal(await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length),paused);
    assert.equal(await page.locator('#arm-hand').innerText(),'关闭实时下发');
    assert(!(await page.locator('#arm-hand').isDisabled()));
    await screenshot({path:`${out}/collision-paused.png`,fullPage:true});
    await page.evaluate(()=>window.mockCollisionPause=false);
    await page.waitForFunction(n=>window.mockWrites.filter(b=>b[5]===0x20).length>n,paused);
    await page.waitForFunction(()=>document.querySelector('#arm-blockers').textContent==='正在实时下发');
  }
  await page.evaluate(()=>{window.mockMissing=false;window.mockTelemetry=true;});
  await page.waitForFunction(()=>document.querySelector("#control-stats").textContent.includes("跟随中"));
  if(directPreviewTest){
    await page.locator('#arm-hand').click();
    assert(!(await page.locator('#arm-hand').isDisabled()));
    assert.equal(await page.locator('#arm-hand').innerText(),'开启实时下发');
    const count=await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length);
    await page.waitForFunction(()=>window.directDraws>=5);
    assert.equal(await page.evaluate(()=>window.directMismatches),0,'model uses direct targets without stale trajectory overwrites');
    assert.equal(await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length),count);
    await page.locator('#arm-hand').click();
    await page.waitForFunction(n=>window.mockWrites.filter(b=>b[5]===0x20).length>n+20,count);
    assert.equal(await page.locator('#arm-hand').innerText(),'关闭实时下发');
    assert(await page.evaluate(()=>window.directController.directMode&&window.directController.armed));
    await screenshot({path:`${out}/direct-downlink.png`,fullPage:true});
  }
  if(continuous){
    const previous=await page.evaluate(()=>({epoch:window.testControl.epoch,n:window.mockWrites.length,
      power:window.mockWrites.filter(b=>b[5]===0x21).length}));
    await page.evaluate(()=>window.testStaleOnce=true);
    await page.waitForFunction(p=>window.testControl.epoch>p.epoch&&window.mockWrites.length>p.n+5,previous);
    assert.equal(await page.locator('#arm-hand').innerText(),'关闭实时下发');
    assert.equal(await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x21).length),previous.power);
    const beforeStall=await page.evaluate(()=>({epoch:window.testControl.epoch,n:window.mockWrites.length}));
    await page.evaluate(()=>{
      const until=performance.now()+953;
      while(performance.now()<until){} // Reproduce the recorded browser-thread stall; fake USB only.
    });
    await page.waitForFunction(p=>window.testControl.epoch>p.epoch&&window.mockWrites.length>p.n+5,beforeStall);
    assert.equal(await page.locator('#arm-hand').innerText(),'关闭实时下发');
  }
  if(process.env.MEASURE_RATE==='1'){
    const before=await page.evaluate(()=>({at:performance.now(),n:window.mockWrites.filter(b=>b[5]===0x20).length}));
    await page.waitForTimeout(Number(process.env.MOCK_RUN_MS||3000));
    const after=await page.evaluate(()=>({at:performance.now(),n:window.mockWrites.filter(b=>b[5]===0x20).length}));
    const result={scope:'Chrome + configured test server + simulated camera/USB + actual recording worker; no physical device',
      recorded_moving_hand:!!recordedHands,
      events:await page.evaluate(t=>window.testStreamEvents.filter(e=>e.at>=t),before.at),
      position_hz:(after.n-before.n)*1000/(after.at-before.at),page_rate:await page.locator('#control-stats').innerText()};
    await writeFile(`${out}/stream-rate.json`,JSON.stringify(result,null,2));console.log(JSON.stringify(result));
    assert(result.position_hz>100,JSON.stringify(result));
  }
  if(partialFeedback)assert.match(await page.locator("#hardware-status").innerText(),/反馈缺失 3\/20/);
  if(selfOnly&&!partialFeedback)assert.match(await page.locator("#hardware-status").innerText(),/反馈缺失 2\/20/);
  assert(await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).every(b=>b.length===108)));
  if(clippedFeedback)assert(await page.evaluate(()=>{
    const first=Uint8Array.from(window.mockWrites.find(b=>b[5]===0x20));
    return new DataView(first.buffer).getFloat32(7,true)===0;
  }),"negative DIP feedback must never be copied into the first hold frame");
  assert(await page.evaluate(()=>window.mockWrites.some(b=>b[5]===0x21&&b[6]===1&&b[7]===3&&b[8]===2)));
  assert(await page.evaluate(()=>!window.mockWrites.some(b=>b[5]===0x21&&b[6]===1&&b[7]===3&&b[8]===1)),"disabled DIP never re-enabled during arm or homing");
  if(skipHome)assert(await page.evaluate(()=>!window.mockWrites.some(b=>b[5]===0x22)));
  await page.locator('input[data-joint="left_hand_mid_DIP_joint"]').uncheck();
  const disabledAt=await page.evaluate(()=>window.mockWrites.length);
  assert(await page.evaluate(()=>window.mockWrites.some(b=>b[5]===0x21&&b[6]===1&&b[7]===1&&b[8]===2)));
  await page.waitForTimeout(150);
  assert(await page.evaluate(n=>!window.mockWrites.slice(n).some(b=>b[5]===0x20),disabledAt));
  await page.locator('input[data-joint="left_hand_mid_DIP_joint"]').check();
  assert(await page.evaluate(()=>window.mockWrites.some(b=>b[5]===0x21&&b[6]===1&&b[7]===1&&b[8]===1)));
  await page.waitForFunction(()=>!document.querySelector("#arm-hand").disabled);
  const rearmAt=await page.evaluate(()=>window.mockWrites.length);
  await page.locator("#arm-hand").click();
  await page.waitForFunction(n=>window.mockWrites.slice(n).filter(b=>b[5]===0x20).length>=2,rearmAt);
  assert(await page.evaluate(()=>window.mockWrites.every(b=>[0x20,0x21,0x22].includes(b[5]))));
  await page.evaluate(()=>{window.mockMissing=true;window.mockTelemetry=false;});
  await page.waitForFunction(()=>document.querySelector("#control-stats").textContent.includes("保持姿态"));
  const n=await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length);
  await page.waitForTimeout(800);
  assert((await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length))>n);
  assert.equal(await page.locator("#arm-hand").innerText(),"关闭实时下发");
  await page.evaluate(()=>{window.mockMissing=false;window.mockTelemetry=true;});
  await page.waitForFunction(()=>document.querySelector("#control-stats").textContent.includes("跟随中"));
  assert.equal(await page.locator("#arm-hand").innerText(),"关闭实时下发");
  await screenshot({path:`${out}/streaming.png`,fullPage:true});
  if(continuous){
    const before=await page.evaluate(()=>window.mockWrites.length);
    await page.evaluate(()=>{
      window.testRecorder.fail(new Error('测试记录容量异常'));
    });
    await page.waitForFunction(n=>window.mockWrites.length>n+10,before);
    assert.equal(await page.locator('#arm-hand').innerText(),'关闭实时下发');
    assert(await page.evaluate(()=>!window.testRecorder.active&&!!window.testRecorder.error));
    assert.equal(await page.evaluate(()=>window.testExportCount),0);
  }
  await page.locator("#close-control").click();
  await page.waitForFunction(()=>document.querySelector("#connect-serial").textContent==="连接串口");
  if(continuous)await page.waitForFunction(()=>window.testExportCount===1);
  const [download]=await Promise.all([page.waitForEvent("download"),page.locator("#download-local").click()]);
  const saved=`${out}/session.jsonl`;await download.saveAs(saved);
  const rows=(await readFile(saved,"utf8")).trim().split(/\r?\n/).map(JSON.parse);
  for(const type of [...(skipHome?[]:["homing_complete"]),"joint_power","a30_conversion","retarget","planner_sample","conversion","h7","downlink","usb_tx","telemetry"])
    assert(rows.some(r=>r.type===type),type);
  if(collisionPause)for(const type of ['control_pause','control_resume'])assert(rows.some(r=>r.type===type),type);
  if(directPreviewTest){
    assert(rows.some(r=>r.type==='direct_command'&&r.sent===false&&r.source==='planner_preview_stream'));
    assert(!rows.some(r=>r.type==='direct_filter_detail'&&r.algorithm==='adaptive_exponential/v1'));
    const filtered=rows.filter(r=>r.type==='direct_command'&&r.sent===true);
    assert(filtered.length>20);
    for(const r of filtered){
      const h7=rows.find(h=>h.seq>r.seq&&h.type==='h7');
      assert.deepEqual(r.joints,h7.joints_ui);
      assert.deepEqual(h7.joints_ruckig_ui,{});
    }
  }
  if(continuous){
    assert(rows.some(r=>r.type==='trajectory_recovery'&&r.reason==='stale_sample'));
    assert(rows.some(r=>r.type==='session_end'&&r.reason==='recording_error'));
    assert(rows.some(r=>r.type==='model_sample'&&r.source==='successful_usb_write'));
    assert.equal(await page.evaluate(()=>window.testModelMismatches),0);
    assert((await page.evaluate(()=>window.testModelSamples))>20);
    const display=rows.filter(r=>r.type==='model_sample'&&r.source==='successful_usb_write');
    await writeFile(`${out}/model-sync.json`,JSON.stringify({samples:display.length,
      mismatches:await page.evaluate(()=>window.testModelMismatches),
      write_to_render_ms:display.map(r=>r.write_to_render_ms)},null,2));
  }
  assert.equal(Object.keys(rows[0].metadata.slot_maps.linear).length,9);
  assert.equal(Object.keys(rows[0].metadata.slot_maps.rotary).length,11);
  const conversion=rows.find(r=>r.type==="conversion");
  assert.equal(rows[0].metadata.joint_enabled.left_hand_index_DIP_joint,false);
  assert.equal(rows[0].metadata.joint_enable_mode,"motor_power");
  if(partialFeedback)assert(rows.some(r=>r.type==="feedback_seed"&&r.fallback_joints.length===3));
  if(selfOnly&&!partialFeedback)assert(rows.some(r=>r.type==="feedback_seed"&&r.fallback_joints.length===2));
  if(clippedFeedback)assert(rows.some(r=>r.type==="feedback_seed"&&r.clipped_joints.some(j=>j.joint==="left_hand_pinky_DIP_joint"&&j.raw<0&&j.applied===0)));
  assert(rows.some(r=>r.type==="joint_enable"&&r.joint==="left_hand_mid_DIP_joint"&&!r.enabled));
  assert(rows.some(r=>r.type==="joint_power"&&r.joint==="left_hand_mid_DIP_joint"&&!r.enabled&&r.sent));
  assert.equal(conversion.linear_mm.length,9);assert.equal(conversion.rotary_deg.length,11);
  await page.locator("#diagnostic-panel summary").click();
  const [diagDownload]=await Promise.all([page.waitForEvent("download"),page.locator("#download-diagnostics").click()]);
  const diagPath=`${out}/diagnostic.jsonl`;await diagDownload.saveAs(diagPath);
  const diagnostics=(await readFile(diagPath,"utf8")).trim().split(/\r?\n/).map(JSON.parse);
  for(const event of ["control_action_requested","control_action_completed","control_arm","control_disarm","feedback_seed","telemetry","usb_tx"])
    assert(diagnostics.some(r=>r.event===event),`diagnostic ${event}`);
  if(collisionPause)for(const event of ['control_pause','control_resume'])assert(diagnostics.some(r=>r.event===event),`diagnostic ${event}`);
  assert(diagnostics.some(r=>r.event==="api_action"&&r.action==="browser_rebase"));
  await page.goto((process.env.WEB_URL||"http://127.0.0.1:8772")+"/data-viewer");
  await page.locator("#file").setInputFiles(saved);
  await page.waitForFunction(()=>document.querySelector("#motor").options.length===20);
  assert((await page.locator("#motor").innerText()).includes("thumb_rotaback_joint2"));
  await screenshot({path:`${out}/data-viewer.png`,fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(badCalls,[]);
  console.log(`A3.0 browser: power WITHOUT telemetry/${selfOnly?444:484}B telemetry/${skipHome?"NO homing":"partial homing"}/arm WITHOUT hand or fresh telemetry/${directPreviewTest?"direct":"Ruckig"}/108B stream/input gap hold and resume/local log PASS`);
}catch(e){
  const state=await page.evaluate(()=>({notice:document.querySelector('#notice').textContent,
    hardware:document.querySelector('#hardware-status').textContent,events:window.testStreamEvents,
    control:window.directController?.snapshot()}));
  await writeFile(`${out}/failure.json`,JSON.stringify({state,controlResponses,errors,badCalls},null,2));
  console.log('BROWSER FAILURE',JSON.stringify(state),`details: ${out}/failure.json`);throw e;
}
finally{await browser.close();}
