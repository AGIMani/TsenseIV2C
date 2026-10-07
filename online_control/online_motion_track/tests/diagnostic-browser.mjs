import {chromium} from "@playwright/test";
import {readFile,mkdir} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import assert from "node:assert/strict";
const root=fileURLToPath(new URL("../../../",import.meta.url)),out=`${root}/logs/diagnostic_browser`;
await mkdir(out,{recursive:true});
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH||"C:/Program Files/Google/Chrome/Application/chrome.exe",headless:true});
const page=await browser.newPage({ignoreHTTPSErrors:process.env.TEST_IGNORE_TLS==="1",viewport:{width:1440,height:1000}}),errors=[];
page.on("pageerror",e=>errors.push(e.message));
await page.addInitScript(()=>{
  const port={getInfo:()=>({usbVendorId:0x1209}),async open(){throw new Error("TEST USB connection refused");}};
  Object.defineProperty(navigator,"serial",{value:{getPorts:async()=>[port],requestPort:async()=>port}});
  window.testFileBytes=new Uint8Array();window.testFileClosed=0;
  window.showDirectoryPicker=async()=>({name:"test-logs",getFileHandle:async()=>({async createWritable(){
    let offset=0,bytes=window.testFileBytes.slice();
    return {async seek(n){offset=n;},async write(text){const add=new TextEncoder().encode(text),next=new Uint8Array(Math.max(bytes.length,offset+add.length));next.set(bytes);next.set(add,offset);bytes=next;offset+=add.length;},
      async truncate(n){bytes=bytes.slice(0,n);},async close(){window.testFileBytes=bytes;window.testFileClosed++;},async abort(){}};
  }})});
});
const base=process.env.WEB_URL||"http://127.0.0.1:8773";
async function download(button,name){
  const [d]=await Promise.all([page.waitForEvent("download"),page.locator(button).click()]);
  assert.match(d.suggestedFilename(),/^hand_diagnostic_/);await d.saveAs(`${out}/${name}`);
  return (await readFile(`${out}/${name}`,"utf8")).trim().split(/\r?\n/).map(JSON.parse);
}
try{
  await page.goto(base+"/?model=TsenseI_v3_0");await page.waitForFunction(()=>!document.querySelector("#start").disabled);
  await page.locator(".record-control").click();assert.equal(await page.locator("#recording").isChecked(),false);
  await page.locator("#diagnostic-panel summary").click();
  await page.locator("#diagnostic-directory").click();
  await page.waitForFunction(()=>document.querySelector("#diagnostic-status").textContent.includes("自动写入"));
  await page.locator("#refresh-ports").click();await page.locator("#serial-port").selectOption("0");
  await page.locator("#connect-serial").click();
  await page.waitForFunction(()=>document.querySelector("#notice").textContent.includes("TEST USB connection refused"));
  const rows=await download("#download-diagnostics","current.jsonl");
  assert(rows.some(r=>r.event==="control_action_requested"&&r.action==="connect"));
  assert(rows.some(r=>r.event==="control_action_failed"&&r.error.message.includes("TEST USB")&&r.error.stack));
  assert(rows.some(r=>r.event==="arm_readiness"&&r.blocked));
  assert(rows.some(r=>r.event==="setting_change"&&r.id==="recording"&&r.value===false));
  assert(rows.filter(r=>r.event).every(r=>r.wall_time&&r.page_id));
  await page.waitForFunction(()=>new TextDecoder().decode(window.testFileBytes).includes("control_action_failed"));
  const disk=await page.evaluate(()=>({closed:window.testFileClosed,rows:new TextDecoder().decode(window.testFileBytes).trim().split('\n').map(JSON.parse)}));
  assert(disk.closed>=2);assert(disk.rows.some(r=>r.event==="control_action_failed"));
  const seqs=disk.rows.filter(r=>r.diagnostic_seq).map(r=>r.diagnostic_seq);assert.equal(new Set(seqs).size,seqs.length);
  await page.screenshot({path:`${out}/panel.png`,fullPage:true});
  const oldPageId=rows.find(r=>r.event==="page_open").page_id;
  await page.reload();await page.waitForFunction(()=>!document.querySelector("#start").disabled);
  await page.locator("#diagnostic-panel summary").click();await page.locator("#diagnostic-history-refresh").click();
  await page.waitForFunction(()=>document.querySelector("#diagnostic-history").options.length>=3);
  await page.locator("#diagnostic-history").selectOption({index:2});
  const recovered=await download("#diagnostic-history-download","recovered.jsonl");
  assert(recovered.some(r=>r.page_id===oldPageId&&r.event==="control_action_failed"));
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
