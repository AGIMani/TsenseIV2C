// Original STL rendering + actual Python retarget; synthetic landmarks/camera only.
import {chromium} from "@playwright/test";
import {execFileSync} from "node:child_process";
import {mkdir, readFile} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import assert from "node:assert/strict";

const root = fileURLToPath(new URL("../../../", import.meta.url));
const out = `${root}/logs/a30_validation`;
await mkdir(out, {recursive: true});
const world = JSON.parse(execFileSync(process.env.HAND_PYTHON || "python", ["-X", "utf8", "-c",
  "import sys,json;sys.path[:0]=['.','src'];from hand_profiles import load_hand_profile;p=load_hand_profile('TsenseI_v3_0');print(json.dumps({s:p.create_retargeter(s).fk.compute().tolist() for s in ('left','right')}))"],
  {cwd: root, encoding: "utf8"}));
const browser = await chromium.launch({executablePath: process.env.CHROME_PATH || "C:/Program Files/Google/Chrome/Application/chrome.exe",
  headless: true, args: ["--use-fake-ui-for-media-stream", "--use-fake-device-for-media-stream"]});
const page = await browser.newPage({viewport: {width: 1440, height: 1050}});
const errors = [], controlRequests = [];
page.on("pageerror", e => errors.push(e.message));
page.on("request", r => {if (r.url().includes("/api/v2/control")) controlRequests.push(r.url());});
await page.addInitScript(world => {
  const NativeWorker=window.Worker;
  window.Worker = class {
    constructor(url,options){if(!String(url).includes('detector.worker'))return new NativeWorker(url,options);}
    postMessage(data) {
      data.bitmap?.close();
      const side = document.querySelector("#side").value;
      this.timer = setTimeout(() => this.onmessage?.({data: data.type === "init"
        ? {type: "ready", delegate: "GPU"}
        : {type: "result", inference_ms: 1, result: {
          handedness: [[{categoryName: side === "left" ? "Left" : "Right", score: .99}]],
          worldLandmarks: [world[side].map(([x,y,z]) => ({x,y,z}))],
          landmarks: [world[side].map(([x,y,z]) => ({x: x+.5,y: y+.5,z}))]}}}), 25);
    }
    terminate() {clearTimeout(this.timer);}
  };
}, world);
const ready = () => page.waitForFunction(() => !document.querySelector("#start").disabled);
try {
  await page.goto((process.env.WEB_URL || "http://127.0.0.1:8766") + "/?model=TsenseI_v3_0");
  await ready();
  assert.equal(await page.locator("#hand-model").inputValue(), "TsenseI_v3_0");
  assert.equal(await page.locator("#hardware-panel").isVisible(), true);
  assert.equal(await page.locator("#a30-integration").isVisible(), true);
  assert.equal(await page.locator("#collision-mode option").count(), 4);
  assert.equal(await page.locator("#collision-mode").inputValue(), "mink");
  await page.screenshot({path: `${out}/a30-left.png`, fullPage: true});
  for (const side of ["left", "right"]) {
    if (side === "right") {await page.locator("#side").selectOption(side); await ready();}
    // Exercise both A3.2 Mink and V2 stop policies through the A3.0 page.
    if (side === "right") await page.locator("#collision-mode").selectOption("stop");
    await page.locator("#start").click();
    await page.waitForFunction(() => document.querySelector("#tracking-text").textContent.includes("正在跟随"));
    assert.match(await page.locator("#model-caption").innerText(), new RegExp(`A3.0 / ${side.toUpperCase()}`));
    await page.screenshot({path: `${out}/a30-${side}-tracking.png`, fullPage: true});
    await page.locator("#start").click();
    await page.waitForFunction(() => document.querySelector("#start span").textContent === "开始识别");
    const [download] = await Promise.all([page.waitForEvent("download"), page.locator("#download-local").click()]);
    const path = `${out}/a30-${side}.jsonl`; await download.saveAs(path);
    const rows = (await readFile(path, "utf8")).trim().split(/\r?\n/).map(JSON.parse);
    for (const type of ["mediapipe", "retarget", "control_target", "a30_conversion"]) assert(rows.some(r => r.type === type), type);
    const conversions = rows.filter(r => r.type === "a30_conversion");
    const expectedMode = side === "left" ? "mink" : "stop";
    assert(rows.some(r => r.type === "control_target" && r.self_collision?.mode === expectedMode));
    for (const c of conversions) {
      assert(c.valid && c.conversion_ready && !c.hardware_enabled);
      assert.equal(Object.keys(c.linear_strokes_mm).length, 9);
      assert.equal(Object.keys(c.rotary_positions_deg).length, 11);
      for (const suffix of ["index_DIP", "mid_DIP", "ring_DIP", "pinky_DIP", "thumb_PIP"]) {
        const joint = `${side}_hand_${suffix}_joint`;
        assert(!(joint in c.linear_strokes_mm));
        assert(Math.abs(c.rotary_positions_deg[joint] - c.joint_angles_rad[joint]*180/Math.PI) < 1e-10);
      }
    }
    assert(!rows.some(r => ["usb_tx", "downlink"].includes(r.type)));
  }
  for (const model of ["TsenseI_v3_2", "TsenseI_v2", "TsenseI_v3_0"]) {
    await page.locator("#hand-model").selectOption(model); await ready();
    assert.equal(await page.locator("#hardware-panel").isVisible(), model !== "TsenseI_v3_2");
  }
  await page.setViewportSize({width: 390, height: 844});
  await page.screenshot({path: `${out}/a30-mobile.png`, fullPage: true});
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
  assert.deepEqual(errors, []);
  assert.deepEqual(controlRequests, []);
  console.log("A3.0 browser OK: both hands retarget, local JSONL, three model switches, no device writes");
} finally {await browser.close();}
