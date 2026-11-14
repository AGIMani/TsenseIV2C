import {chromium} from "@playwright/test";
import {readFile,mkdir} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import assert from "node:assert/strict";
const root=fileURLToPath(new URL("../../../",import.meta.url)),out=`${root}/logs/diagnostic_browser`;
await mkdir(out,{recursive:true});
const browser=await chromium.launch({...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{}),headless:true});
const page=await browser.newPage({ignoreHTTPSErrors:process.env.TEST_IGNORE_TLS==="1",viewport:{width:1440,height:1000}}),errors=[];
page.on("pageerror",e=>errors.push(e.message));
await page.route('**/viewer.js', route => route.fulfill({contentType:'text/javascript', body:`
  export class HandViewer {
    constructor(){this.robot={root:{visible:true}};}
    async load(){return {initial_pose:{}};}
    setPose(){} dispose(){} resetView(){}
  }
  export class SkeletonViewer {setPoints(){} dispose(){} resetView(){}}
`}));
await page.addInitScript(()=>{
  window.testFileBytes=new Uint8Array();window.testFileClosed=0;
  window.showDirectoryPicker=async()=>({name:"test-logs",getFileHandle:async()=>({async createWritable(){
    let offset=0,bytes=window.testFileBytes.slice();
    return {async seek(n){offset=n;},async write(text){const add=new TextEncoder().encode(text),next=new Uint8Array(Math.max(bytes.length,offset+add.length));next.set(bytes);next.set(add,offset);bytes=next;offset+=add.length;},
      async truncate(n){bytes=bytes.slice(0,n);},async close(){window.testFileBytes=bytes;window.testFileClosed++;},async abort(){}};
  }})});
});
await page.route('**/api/a32/control', async route => {
  const action = route.request().postDataJSON()?.action;
  if (action === 'connect') return route.fulfill({status:400, contentType:'application/json', body:JSON.stringify({error:'TEST SDK connection refused'})});
  return route.continue();
});
const base=process.env.WEB_URL||"http://127.0.0.1:8773";
async function download(button,name){
  const [d]=await Promise.all([page.waitForEvent("download"),page.locator(button).click()]);
  assert.match(d.suggestedFilename(),/^hand_diagnostic_/);await d.saveAs(`${out}/${name}`);
  return (await readFile(`${out}/${name}`,"utf8")).trim().split(/\r?\n/).map(JSON.parse);
}
try{
  await page.goto(base+"/?model=TsenseI_v3_2_withid");await page.waitForFunction(()=>!document.querySelector("#start").disabled);
  await page.locator("label:has(#recording)").click();assert.equal(await page.locator("#recording").isChecked(),false);
  await page.locator("#diagnostic-panel summary").click();
  await page.locator("#diagnostic-directory").click();
  await page.waitForFunction(()=>document.querySelector("#diagnostic-status").textContent.includes("自动写入"));
  await page.locator("#a32-connect").click();
  await page.waitForFunction(()=>document.querySelector("#notice").textContent.includes("TEST SDK connection refused"));
  const rows=await download("#download-diagnostics","current.jsonl");
  assert(rows.some(r=>r.event==="button_press"&&r.id==="a32-connect"));
  assert(rows.some(r=>r.event==="api_error"&&r.action==="connect"&&r.error.message.includes("TEST SDK")&&r.error.stack));
  assert(rows.some(r=>r.event==="setting_change"&&r.id==="recording"&&r.value===false));
  assert(rows.filter(r=>r.event).every(r=>r.wall_time&&r.page_id));
  await page.waitForFunction(()=>new TextDecoder().decode(window.testFileBytes).includes("api_error"));
  const disk=await page.evaluate(()=>({closed:window.testFileClosed,rows:new TextDecoder().decode(window.testFileBytes).trim().split('\n').map(JSON.parse)}));
  assert(disk.closed>=2);assert(disk.rows.some(r=>r.event==="api_error"));
  const seqs=disk.rows.filter(r=>r.diagnostic_seq).map(r=>r.diagnostic_seq);assert.equal(new Set(seqs).size,seqs.length);
  await page.screenshot({path:`${out}/panel.png`,fullPage:true});
  const oldPageId=rows.find(r=>r.event==="page_open").page_id;
  await page.reload();await page.waitForFunction(()=>!document.querySelector("#start").disabled);
  await page.locator("#diagnostic-panel summary").click();await page.locator("#diagnostic-history-refresh").click();
  await page.waitForFunction(()=>document.querySelector("#diagnostic-history").options.length>=3);
  await page.locator("#diagnostic-history").selectOption({index:2});
  const recovered=await download("#diagnostic-history-download","recovered.jsonl");
  assert(recovered.some(r=>r.page_id===oldPageId&&r.event==="api_error"));
  // Failed browser storage keeps a bounded, downloadable memory fallback.
  const fallback=await page.evaluate(async()=>{
    const {DiagnosticLogger}=await import('/diagnostic-logger.js');const log=new DiagnosticLogger();await log.ready;
    log.storageFailed(new Error('TEST quota'));log.record('after_storage_failure',{message:'日志仍可下载'});
    const file=await log.export(),text=await (await fetch(file.url)).text();URL.revokeObjectURL(file.url);
    clearInterval(log.timer);clearInterval(log.storage.timer);return text.trim().split('\n').map(JSON.parse);
  });
  assert(fallback.some(r=>r.event==="memory_fallback"));assert(fallback.some(r=>r.event==="after_storage_failure"));
  assert.deepEqual(errors,[]);
  console.log("Diagnostic log: angle-recording OFF / action error + stack / directory UTF-8 append / reload recovery / quota fallback PASS");
}finally{await browser.close();}
