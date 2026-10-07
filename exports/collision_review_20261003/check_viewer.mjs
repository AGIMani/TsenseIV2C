import {chromium} from '/home/onkiall/projects/hand/online_control/online_motion_track/node_modules/@playwright/test/index.mjs';
const browser=await chromium.launch({headless:true,args:['--no-sandbox']});
const page=await browser.newPage({viewport:{width:860,height:1050},deviceScaleFactor:1});
const errors=[];
page.on('pageerror',error=>errors.push(error.message));
await page.goto('file:///home/onkiall/projects/hand/exports/collision_review_20261003/viewer-preview.html');
const frame=page.frameLocator('iframe');
await frame.locator('[data-field="summary"]').filter({hasText:'启用检测'}).waitFor();
const models=await frame.locator('[data-field="model"] option').count();
const results=[];
for(let m=0;m<models;m++)for(const side of ['left','right']){
  await frame.locator('[data-field="model"]').selectOption(String(m));
  await frame.locator('[data-field="side"]').selectOption(side);
  const options=await frame.locator('[data-field="geom"] option').evaluateAll(els=>els.map(e=>e.value));
  for(const value of options){
    await frame.locator('[data-field="geom"]').selectOption(value);
    const current=await frame.locator('[data-field="selection"]').textContent();
    if(!current)throw Error('Selection did not render');
  }
  results.push({model:m,side,geometries:options.length-1,summary:await frame.locator('[data-field="summary"]').textContent()});
}
await frame.locator('[data-field="model"]').selectOption('3');
await frame.locator('[data-field="side"]').selectOption('left');
await frame.locator('[data-field="geom"]').selectOption('-1');
await page.screenshot({path:'/home/onkiall/projects/hand/exports/collision_review_20261003/viewer-desktop.png',fullPage:true});
await frame.locator('[data-field="geom"]').selectOption('0');
const before=await frame.locator('[data-field="selection"]').textContent();
const canvas=frame.locator('canvas'),area=await canvas.boundingBox();
await page.mouse.move(area.x+area.width/2,area.y+area.height/2);
await page.mouse.down();await page.mouse.move(area.x+area.width/2+100,area.y+area.height/2+40,{steps:6});await page.mouse.up();
if(await frame.locator('[data-field="selection"]').textContent()!==before)throw Error('Drag changed selection');
await page.screenshot({path:'/home/onkiall/projects/hand/exports/collision_review_20261003/viewer-selected.png',fullPage:true});
await page.setViewportSize({width:380,height:1100});
await page.screenshot({path:'/home/onkiall/projects/hand/exports/collision_review_20261003/viewer-mobile.png',fullPage:true});
console.log(JSON.stringify({results,errors}));
await browser.close();
if(errors.length)process.exit(1);
