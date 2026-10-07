import { chromium } from "@playwright/test";
import { readFile } from "node:fs/promises";
import assert from "node:assert/strict";

const base = process.env.WEB_URL || "http://127.0.0.1:8766";
const browser = await chromium.launch({
  executablePath:
    process.env.CHROME_PATH ||
    "C:/Program Files/Google/Chrome/Application/chrome.exe",
  headless: true,
});
try {
  const page = await browser.newPage();
  const original = await readFile(
    new URL("../static/detector.worker.js", import.meta.url),
    "utf8",
  );
  const patched = original.replace(
    "new OffscreenCanvas(1, 1)",
    "(() => { const c = new OffscreenCanvas(1, 1); c.getContext = () => null; return c; })()",
  );
  assert.notEqual(original, patched);
  await page.route("**/detector.worker.js", (route) =>
    route.fulfill({ contentType: "text/javascript", body: patched }),
  );
  await page.goto(`${base}/api/health`);
  const result = await page.evaluate(
    () =>
      new Promise((resolve, reject) => {
        const worker = new Worker("/detector.worker.js", { type: "module" });
        const timer = setTimeout(() => {
          worker.terminate();
          reject(new Error("worker timeout"));
        }, 45000);
        worker.onmessage = async ({ data }) => {
          if (data.type === "ready") {
            window.workerReady = data;
            const canvas = new OffscreenCanvas(640, 480);
            canvas.getContext("2d").fillRect(0, 0, 640, 480);
            const bitmap = await createImageBitmap(canvas);
            worker.postMessage({ type: "frame", bitmap, timestamp_ms: 1 }, [
              bitmap,
            ]);
          } else {
            clearTimeout(timer);
            worker.terminate();
            resolve({ ready: window.workerReady, frame: data });
          }
        };
        worker.postMessage({ type: "init" });
      }),
  );
  assert.equal(result.ready.delegate, "CPU");
  assert(result.ready.fallbackReason);
  assert.equal(result.frame.type, "result");
  console.log(
    "GPU failure -> CPU fallback + real blank-image detection passed",
  );
  await page.unroute("**/detector.worker.js");
  await page.addInitScript(() => {
    navigator.mediaDevices.getUserMedia = async () => {
      throw new DOMException("test permission denied", "NotAllowedError");
    };
  });
  await page.goto(base);
  await page.waitForFunction(() => !document.querySelector("#start").disabled);
  await page.locator("#start").click();
  await page.waitForFunction(() =>
    document.querySelector("#notice").textContent.includes("相机权限未开启"),
  );
  assert.equal(await page.locator("#start span").innerText(), "开始识别");
  assert(
    await page.evaluate(
      () => document.querySelector("#video").srcObject === null,
    ),
  );
  console.log("Permission denial releases state and allows retry");
} finally {
  await browser.close();
}
