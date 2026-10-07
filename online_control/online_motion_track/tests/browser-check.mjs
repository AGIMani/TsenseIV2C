// Run against main.py using the installed Chrome; fake video is test-only.
import { chromium } from "@playwright/test";
import { mkdir } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import assert from "node:assert/strict";

const url = process.env.WEB_URL || "http://127.0.0.1:8766";
const output = new URL(
  "../../../logs/web_v3_2_validation/",
  import.meta.url,
).pathname.replace(/^\/([A-Za-z]:)/, "$1");
await mkdir(output, { recursive: true });
const browser = await chromium.launch({
  executablePath:
    process.env.CHROME_PATH ||
    "C:/Program Files/Google/Chrome/Application/chrome.exe",
  headless: true,
  args: [
    "--use-fake-ui-for-media-stream",
    "--use-fake-device-for-media-stream",
  ],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 1060 } });
const errors = [];
page.on("pageerror", (error) => errors.push(error.message));
page.on("console", (message) => {
  if (message.type() === "error") console.log("CONSOLE", message.text());
});
try {
  await page.goto(url);
  await page.locator("#start").waitFor({ state: "visible" });
  await page.waitForFunction(() => !document.querySelector("#start").disabled, {
    timeout: 30000,
  });
  assert.equal(await page.locator(".workspace > .panel").count(), 4);
  assert.equal(await page.locator("#skeleton-stage canvas").count(), 1);
  assert.equal(await page.locator("#model-stage canvas").count(), 1);
  console.log(
    "WEBGL",
    await page.evaluate(() => {
      const gl = document
        .querySelector("#model-stage canvas")
        .getContext("webgl2");
      const debug = gl.getExtension("WEBGL_debug_renderer_info");
      return debug
        ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL)
        : gl.getParameter(gl.RENDERER);
    }),
  );
  await page.screenshot({ path: `${output}/web-desktop.png`, fullPage: true });
  await page.locator("#skeleton-stage canvas").waitFor();
  await page.screenshot({ path: `${output}/web-skeleton.png`, fullPage: true });
  await page.locator("#side").selectOption("right");
  await page.waitForFunction(() => !document.querySelector("#start").disabled);
  assert.match(await page.locator("#model-caption").innerText(), /RIGHT/);
  await page.screenshot({ path: `${output}/web-right.png`, fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: `${output}/web-mobile.png`, fullPage: true });
  assert(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  );
  await page.setViewportSize({ width: 1440, height: 1060 });
  await page.locator(".record-control").click();
  assert.equal(await page.locator("#recording").isChecked(), false);
  await page.locator("#start").click();
  await page.waitForFunction(
    () =>
      document.querySelector("#camera-state").textContent === "实时" ||
      (!document.querySelector("#notice").hidden &&
        document.querySelector("#tracking-text").textContent.includes("失败")),
    { timeout: 60000 },
  );
  console.log(
    "DETECTOR",
    await page.locator("#gpu-badge").innerText(),
    "NOTICE",
    await page.locator("#notice").innerText(),
  );
  assert.equal(await page.locator("#camera-state").innerText(), "实时");
  await page.waitForFunction(
    () => Number(document.querySelector("#fps").textContent) >= 1,
    { timeout: 30000 },
  );
  console.log("FPS", await page.locator("#fps").innerText());
  await page.screenshot({
    path: `${output}/web-running-fake-camera.png`,
    fullPage: true,
  });
  await page.locator("#start").click();
  await page.waitForFunction(
    () => document.querySelector("#camera-state").textContent === "已停止",
  );
  assert(
    await page.evaluate(
      () => document.querySelector("#video").srcObject === null,
    ),
  );
  assert.deepEqual(errors, []);
  await page.close();

  // A deterministic FK gesture source tests the complete transport/solver/UI
  // path separately from the real GPU detector above. No production test hook.
  const root = fileURLToPath(new URL("../../../", import.meta.url));
  const fixtures = JSON.parse(
    execFileSync(
      process.env.HAND_PYTHON || "E:/conda/envs/hand_dex/python.exe",
      [
        "-c",
        `
import sys,json,numpy as np
sys.path[:0]=['.','src']
from hand_profiles import load_hand_profile
p=load_hand_profile('TsenseI_v3_2');s=p.create_retargeter('left');q=p.init_pose()
for n,(lo,hi) in p.joint_limits().items():
 if any('_'+f+'_' in n for f in ('index','mid','ring','pinky')) and any('_'+j+'_' in n for j in ('MCP','PIP','DIP')): q[n]=float(np.clip(1.2,lo,hi))
extended={n:0. if '_mid_' in n else v for n,v in q.items()}
print(json.dumps([s.fk.compute(a).tolist() for a in (q,extended)]))
`,
      ],
      { cwd: root, encoding: "utf8" },
    ),
  );
  const synthetic = await browser.newPage({
    viewport: { width: 1440, height: 1060 },
  });
  await synthetic.addInitScript(
    ({ fixtures }) => {
      window.gesture = 0;
      window.testHandLabel = "Left";
      const NativeWorker=window.Worker;
      window.Worker = class {
        constructor(url,options){if(!String(url).includes('detector.worker'))return new NativeWorker(url,options);}
        postMessage(data) {
          if (data.type === "init") {
            this.timer = setTimeout(
              () =>
                this.onmessage?.({ data: { type: "ready", delegate: "GPU" } }),
              10,
            );
          } else {
            data.bitmap.close();
            this.timer = setTimeout(() => {
              const result =
                window.gesture === null
                  ? { handedness: [], landmarks: [], worldLandmarks: [] }
                  : {
                      handedness: [[{ categoryName: window.testHandLabel }]],
                      landmarks: [
                        fixtures[window.gesture].map((_, id) => {
                          if (!id) return { x: 0.5, y: 0.86, z: 0 };
                          const finger = Math.floor((id - 1) / 4);
                          const bone = (id - 1) % 4;
                          return {
                            x: 0.19 + finger * 0.145 + bone * 0.012,
                            y: 0.65 - bone * 0.14 + (finger === 0 ? 0.1 : 0),
                            z: 0,
                          };
                        }),
                      ],
                      worldLandmarks: [
                        fixtures[window.gesture].map(([x, y, z]) => ({
                          x,
                          y,
                          z,
                        })),
                      ],
                    };
              this.onmessage?.({
                data: { type: "result", result, inference_ms: 0 },
              });
            }, 20);
          }
        }
        terminate() {
          clearTimeout(this.timer);
        }
      };
    },
    { fixtures },
  );
  const frames = [];
  synthetic.on("response", async (response) => {
    if (response.url().endsWith("/api/frame") && response.ok())
      frames.push(await response.json());
  });
  await synthetic.goto(url);
  await synthetic.waitForFunction(
    () => !document.querySelector("#start").disabled,
  );
  await synthetic.locator("#start").click();
  await synthetic.waitForFunction(() =>
    document.querySelector("#tracking-text").textContent.includes("正在跟随"),
  );
  await new Promise((resolve) => {
    const check = setInterval(() => {
      if (frames.length >= 12) {
        clearInterval(check);
        resolve();
      }
    }, 50);
    setTimeout(() => {
      clearInterval(check);
      resolve();
    }, 15000);
  });
  assert(frames.at(-1)?.pose.left_hand_mid_PIP_joint > 0.9);
  assert(
    await synthetic.evaluate(() => {
      const c = document.querySelector("#camera-overlay");
      return c
        .getContext("2d")
        .getImageData(0, 0, c.width, c.height)
        .data.some((v, i) => i % 4 === 3 && v > 0);
    }),
    "camera joint overlay must contain drawn pixels",
  );
  await synthetic.screenshot({
    path: `${output}/web-synthetic-fist.png`,
    fullPage: true,
  });
  await synthetic.evaluate(() => {
    window.gesture = 1;
  });
  const before = frames.length;
  for (
    let attempts = 0;
    attempts < 100 &&
    (frames.length < before + 8 ||
      frames.at(-1).pose.left_hand_mid_PIP_joint > 0.15);
    attempts++
  ) {
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
  assert(frames.at(-1).pose.left_hand_mid_PIP_joint < 0.15);
  assert.equal(
    await synthetic.locator("#skeleton-status").innerText(),
    "21 点 · 实时",
  );
  await synthetic.screenshot({
    path: `${output}/web-synthetic-middle.png`,
    fullPage: true,
  });
  await synthetic.evaluate(() => {
    window.gesture = null;
  });
  await synthetic.waitForFunction(
    () => document.querySelector("#skeleton-status").textContent === "未检测",
  );
  assert(
    await synthetic.evaluate(() => {
      const c = document.querySelector("#camera-overlay");
      return !c
        .getContext("2d")
        .getImageData(0, 0, c.width, c.height)
        .data.some((v, i) => i % 4 === 3 && v > 0);
    }),
    "lost detection clears the overlay",
  );
  await synthetic.locator("#start").click();
  await synthetic.waitForFunction(() =>
    document.querySelector("#record-status").textContent.includes("记录已保存"),
  );
  console.log(
    "SYNTHETIC PIPELINE",
    frames.length,
    "frames, middle extended, recording saved",
  );
  await synthetic.close();
  console.log("BROWSER CHECK PASSED", output);
} finally {
  await browser.close();
}
