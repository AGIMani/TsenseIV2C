// Run only against isolated test servers with empty motion data; never connects hardware.
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE || '@playwright/test');
import assert from 'node:assert/strict';
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH,headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1440,height:1000}});
const errors=[],failed=[];let loaded=0;
page.on('pageerror',e=>errors.push(e.message));
page.on('response',r=>{if(/\.(js|mjs|wasm|stl|urdf|task)(\?|$)/.test(r.url())){loaded++;if(!r.ok())failed.push([r.status(),r.url()]);}});
try{
 await page.addInitScript(()=>{if(navigator.mediaDevices)navigator.mediaDevices.getUserMedia=()=>{throw Error('Camera must not be requested');};});
 await page.goto(process.env.WEB_URL+'/?source=webxr');
 await page.waitForFunction(()=>!document.querySelector('#start').disabled);
 assert.equal(await page.locator('#input-source').inputValue(),'webxr');
 assert.equal(await page.locator('#model-stage canvas').count(),1);
 await page.screenshot({path:process.env.MOTION_QA_OUTPUT+'/webxr-resource-desktop.png',fullPage:true});
 await page.goto(process.env.WEB_URL+'/q');
 await page.waitForLoadState('networkidle');
 assert.match(await page.locator('body').innerText(),/Quest|WebXR/);
 await page.screenshot({path:process.env.MOTION_QA_OUTPUT+'/webxr-resource-capture.png',fullPage:true});
 assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);assert(loaded>10);
 console.log(JSON.stringify({ok:true,webxr_desktop:true,quest_capture:true,assets_loaded:loaded,errors,failed}));
}finally{await browser.close();}
