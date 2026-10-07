// Uses real public Python solver, simulated MediaPipe/camera and memory USB.
import {chromium} from "@playwright/test";
import {execFileSync} from "node:child_process";
import {mkdir,readFile} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import assert from "node:assert/strict";
const root=fileURLToPath(new URL("../../../",import.meta.url)),out=`${root}/logs/browser_serial`;
await mkdir(out,{recursive:true});
const world=JSON.parse(execFileSync(process.env.HAND_PYTHON||"python",["-X","utf8","-c","import sys,json;sys.path[:0]=['.','src'];from hand_profiles import load_hand_profile;s=load_hand_profile('TsenseI_v2').create_retargeter();print(json.dumps(s.fk.compute().tolist()))"],{cwd:root,encoding:"utf8"}));
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH||"C:/Program Files/Google/Chrome/Application/chrome.exe",headless:true,args:["--use-fake-ui-for-media-stream","--use-fake-device-for-media-stream"]});
const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],actions=[];
page.on("pageerror",e=>errors.push(e.message));
page.on("request",r=>{if(r.url().includes("/api/v2/control"))actions.push(r.postDataJSON().action);});
const partial = process.env.MOCK_HOMING_PARTIAL === "1";
const skipHome = process.env.MOCK_SKIP_HOME === "1";
await page.addInitScript(({world,partial})=>{
  window.mockWrites=[];window.mockMissing=false;window.mockTelemetry=true;
  const NativeWorker=window.Worker;
  window.Worker=class{
    constructor(url,options){if(!String(url).includes('detector.worker'))return new NativeWorker(url,options);}
    postMessage(data){data.bitmap?.close();this.timer=setTimeout(()=>this.onmessage?.({data:data.type==="init"?{type:"ready",delegate:"GPU"}:{type:"result",inference_ms:1,result:window.mockMissing?{landmarks:[],worldLandmarks:[],handedness:[]}:{handedness:[[{categoryName:"Left",score:.99}]],worldLandmarks:[world.map(([x,y,z])=>({x,y,z}))],landmarks:[world.map(([x,y,z])=>({x:x+.5,y:y+.5,z}))]}}}),25);}
    terminate(){clearTimeout(this.timer);}
  };
  const crc=bytes=>{let c=65535;for(const b of bytes){c^=b;for(let i=0;i<8;i++)c=c&1?(c>>>1)^0xa001:c>>>1;}return c;};
  const port={getInfo:()=>({usbVendorId:0x1209,usbProductId:0x1234}),async open(){
    let controller,seq=0,side=0;const lm=Array(14).fill(0),rm=Array(7).fill(0);
    this.readable=new ReadableStream({start:c=>controller=c,cancel:()=>clearInterval(this.timer)});
    this.writable=new WritableStream({write:bytes=>{
      window.mockWrites.push(Array.from(bytes));const bus=bytes[6],motor=bytes[7];
      if(bytes[5]===0x2b)side=bytes[7];
      if(bytes[5]===0x12){const slot=[0,6,12][bus]+motor-1;lm[slot]=bytes[8]===1?(partial&&slot===0?3:partial&&slot===5?1:2):0;}
      if(bytes[5]===0x25){const slot=[0,2,6,4][bus]+(bytes[7]===1?0:1);rm[slot]=0;if(!partial||slot!==2)setTimeout(()=>rm[slot]=1,220);}
    }});
    this.timer=setInterval(()=>{
      if(!window.mockTelemetry)return;
      const b=new Uint8Array(788),v=new DataView(b.buffer);b.set([0x55,0xaa,12,3,3,2]);
      v.setUint16(6,seq++,true);v.setUint32(10,Math.floor(performance.now()),true);
      for(let i=0;i<14;i++)b[14+32*i]=lm[i];
      for(let i=0;i<7;i++)b[462+44*i]=rm[i];
      b[463]=side;v.setUint16(786,crc(b.subarray(0,786)),true);controller.enqueue(b);
    },25);
  },async close(){clearInterval(this.timer);}};
  Object.defineProperty(navigator,"serial",{value:{getPorts:async()=>[port],requestPort:async()=>port}});
},{world,partial});
try{
  const url=process.env.WEB_TEST_URL||"http://127.0.0.1:8769";
  await page.goto(url+"/?model=TsenseI_v2");
  await page.waitForFunction(()=>!document.querySelector("#start").disabled);
  await page.locator("#collision-mode").selectOption("stop");
  await page.locator("#connect-serial").click();
  await page.waitForFunction(()=>document.querySelector("#connect-serial").textContent==="断开串口");
  await page.waitForTimeout(100);
  if(!skipHome){await page.locator("#home-hand").click();
  await page.waitForFunction(()=>document.querySelector("#home-hand").textContent==="停止找零");
  await page.waitForFunction(()=>document.querySelector("#homing-result-dialog").open,{},{timeout:35000});
  assert.equal(await page.locator("#homing-result-rows tr").count(),partial?3:0);
  if(partial) {
    const content=await page.locator("#homing-result-dialog").innerText();
    for(const text of ["无名指 MCP","小拇指 DIP","无名指 swing 摆动","不会阻止开启实时下发"])assert(content.includes(text),text);
    await page.screenshot({path:`${out}/partial-homing-dialog.png`,fullPage:true});
  }
  await page.locator("#homing-result-close").click();}
  await page.locator("#start").click();
  await page.waitForFunction(()=>!document.querySelector("#arm-hand").disabled);
  await page.locator("#arm-hand").click();
  await page.waitForFunction(()=>window.mockWrites.filter(b=>b[5]===0x20).length>=2,{},{timeout:10000});
  if(skipHome)assert(await page.evaluate(()=>!window.mockWrites.some(b=>[0x12,0x25].includes(b[5]))));
  await page.evaluate(()=>window.mockMissing=true);
  await page.waitForFunction(()=>document.querySelector("#arm-hand").textContent==="开启实时下发");
  const n=await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length);
  await page.waitForTimeout(250);
  await page.screenshot({path:`${out}/v2-local-serial.png`,fullPage:true});
  assert.equal(await page.evaluate(()=>window.mockWrites.filter(b=>b[5]===0x20).length),n);
  await page.evaluate(()=>window.mockMissing=false);
  await page.waitForFunction(()=>!document.querySelector("#arm-hand").disabled);
  await page.locator("#arm-hand").click();
  await page.waitForFunction(n=>window.mockWrites.filter(b=>b[5]===0x20).length>n,n);
  await page.locator("#hand-model").selectOption("TsenseI_v3_2");
  await page.waitForFunction(()=>document.querySelector("#model-caption").textContent.includes("V3.2")&&!document.querySelector("#start").disabled);
  assert.equal(await page.locator("#hardware-panel").isVisible(),false);
  const [download]=await Promise.all([page.waitForEvent("download"),page.locator("#download-local").click()]);
  const saved=`${out}/browser-recording.jsonl`;await download.saveAs(saved);
  const rows=(await readFile(saved,"utf8")).trim().split(/\r?\n/).map(JSON.parse);
  for(const type of ["session","mediapipe_raw","mediapipe","retarget","control_target","planner_sample","h7","conversion","downlink","usb_tx","telemetry","joint_power",...(skipHome?[]:["homing_complete"]),"session_end"])assert(rows.some(r=>r.type===type),type);
  assert.equal(rows[0].schema,"hand-control-session/v2");
  assert(rows.filter(r=>r.type==="downlink"&&r.sent).every(r=>r.wire_hex.length===192));
  assert(actions.every(action=>["status","tracking_start","browser_rebase"].includes(action)),actions.join(","));
  await page.locator("#side").selectOption("right");
  await page.waitForFunction(()=>document.querySelector("#model-caption").textContent.includes("RIGHT")&&!document.querySelector("#start").disabled);
  await page.locator("#hand-model").selectOption("TsenseI_v2");
  await page.waitForFunction(()=>document.querySelector("#model-caption").textContent.includes("V2")&&!document.querySelector("#start").disabled);
  await page.setViewportSize({width:390,height:844});await page.screenshot({path:`${out}/v2-local-mobile.png`,fullPage:true});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.reload();await page.waitForFunction(()=>document.querySelector("#local-recordings").options.length>1);
  await page.locator("#local-recordings").selectOption({index:1});
  const [recovered]=await Promise.all([page.waitForEvent("download"),page.locator("#recover-recording").click()]);
  await recovered.saveAs(`${out}/recovered.jsonl`);assert.equal(await readFile(`${out}/recovered.jsonl`,"utf8"),await readFile(saved,"utf8"));
  const viewer=await browser.newPage();await viewer.goto(url+"/data-viewer");await viewer.locator('input[type="file"]').setInputFiles(saved);
  await viewer.waitForTimeout(300);await viewer.screenshot({path:`${out}/native-data-viewer.png`,fullPage:true});
  assert.equal(errors.length,0,errors.join("\n"));
  console.log(JSON.stringify({ok:true,rows:rows.length,localWrites:n,serverActions:[...new Set(actions)],recording:saved}));
}catch(error){console.error(await page.locator("#hardware-status").textContent(),await page.locator("#notice").textContent(),errors);await page.screenshot({path:`${out}/error.png`,fullPage:true});throw error;}
finally{await browser.close();}
