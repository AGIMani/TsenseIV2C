// Real Chrome + deployed Form2 + in-memory USB. Never authorizes a real device.
import {chromium} from '@playwright/test';
import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
const base=process.env.MOTION_URL||'https://192.168.88.229:8767';
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
const page=await browser.newPage({ignoreHTTPSErrors:process.env.TEST_IGNORE_TLS==='1',viewport:{width:1400,height:1000}});
const errors=[];page.on('pageerror',e=>errors.push(e.message));
await page.addInitScript(()=>{
  window.positionWrites=0;
  const crc=b=>{let c=65535;for(const x of b){c^=x;for(let i=0;i<8;i++)c=c&1?(c>>>1)^0xa001:c>>>1;}return c;};
  const port={getInfo:()=>({usbVendorId:0x0483,usbProductId:0x5740}),async open(){
    let input,seq=0;
    this.readable=new ReadableStream({start:c=>input=c,cancel:()=>clearInterval(this.timer)});
    this.writable=new WritableStream({write:bytes=>{
      for(let at=0;at<bytes.length;){
        const size=8+bytes[at+2]+(bytes[at+3]<<8),b=bytes.subarray(at,at+size);
        if(crc(b.subarray(0,-2))!==(b.at(-2)|b.at(-1)<<8))throw Error('Bad wire CRC');
        if(b[5]===0x20)window.positionWrites++;
        at+=size;
      }
    }});
    this.timer=setInterval(()=>{
      const b=new Uint8Array(492),v=new DataView(b.buffer);
      b.set([0xaa,0x55,0xe4,1,3,2]);v.setUint16(6,seq++,true);v.setUint32(10,seq,true);
      v.setUint16(490,crc(b.subarray(0,-2)),true);input.enqueue(b);
    },4);
  },async setSignals(){},async close(){clearInterval(this.timer);}};
  Object.defineProperty(navigator,'serial',{value:{getPorts:async()=>[port],requestPort:async()=>port,addEventListener(){}}});
});
try{
  await page.goto(base);
  await page.waitForFunction(()=>document.querySelector('#control').textContent==='释放控制');
  await page.locator('#modelSelect').selectOption('TsenseI_v3_0');
  await page.waitForFunction(()=>document.querySelector('#dof').textContent.includes('20'));
  await page.waitForFunction(()=>document.querySelector('#modelNote').textContent==='');
  await page.locator('#ports').selectOption('0');await page.locator('#connect').click();
  await page.waitForFunction(()=>document.querySelector('#a30Telemetry').textContent.includes('遥测正常'));
  await page.locator('#realtimeJointControl input').check();
  await page.waitForFunction(()=>window.positionWrites>10,{},{timeout:5000});
  const before=await page.evaluate(()=>({at:performance.now(),n:window.positionWrites}));
  await page.waitForTimeout(3000);
  const after=await page.evaluate(()=>({at:performance.now(),n:window.positionWrites}));
  const result={scope:'Chrome + node0 HTTPS + mock USB, no physical device',position_hz:(after.n-before.n)*1000/(after.at-before.at),
    page_rate:await page.locator('#serialNote').innerText()};
  assert(result.position_hz>100,JSON.stringify(result));
  await mkdir('logs/online-control-browser',{recursive:true});
  await writeFile('logs/online-control-browser/motion-rate.json',JSON.stringify(result,null,2));
  await page.screenshot({path:'logs/online-control-browser/motion.png',fullPage:true});
  await page.locator('#stop').click();
  await page.waitForFunction(()=>document.querySelector('#connect').textContent==='连接本机串口');
  const count=await page.evaluate(()=>window.positionWrites);
  await page.waitForTimeout(250);assert.equal(await page.evaluate(()=>window.positionWrites),count);
  await page.locator('#control').click();assert.deepEqual(errors,[]);
  console.log(JSON.stringify(result));
} catch(e){console.log('ERRORS',errors,'PAGE',(await page.locator('body').innerText()).slice(0,2500));throw e;} finally {await browser.close();}
