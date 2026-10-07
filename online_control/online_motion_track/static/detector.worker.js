// MediaPipe runs off the UI thread. A transferred frame is always released.
import {
  HandLandmarker,
  FilesetResolver,
} from "/vendor/mediapipe/vision_bundle.mjs";

let detector;
self.onmessage = async ({ data }) => {
  try {
    if (data.type === "init") {
      // tasks-vision 1.x module workers need the ESM WASM loader (second arg).
      const fileset = await FilesetResolver.forVisionTasks(
        "/vendor/mediapipe/wasm",
        true,
      );
      const options = {
        baseOptions: {
          modelAssetPath: "/hand_landmarker.task",
          delegate: "GPU",
        },
        runningMode: "VIDEO",
        numHands: 2,
        minHandDetectionConfidence: 0.5,
        minHandPresenceConfidence: 0.5,
        minTrackingConfidence: 0.5,
      };
      let delegate = "GPU",
        fallbackReason = null;
      try {
        detector = await HandLandmarker.createFromOptions(fileset, {
          ...options,
          canvas: new OffscreenCanvas(1, 1),
        });
      } catch (error) {
        fallbackReason = String(error.message || error);
        delegate = "CPU";
        // Reload the ESM factory if a partially constructed GPU task failed.
        fileset.wasmLoaderPath += "?fallback=cpu";
        detector = await HandLandmarker.createFromOptions(fileset, {
          ...options,
          baseOptions: { ...options.baseOptions, delegate: "CPU" },
        });
      }
      self.postMessage({ type: "ready", delegate, fallbackReason });
    } else if (data.type === "frame") {
      const started = performance.now();
      try {
        const result = detector.detectForVideo(data.bitmap, data.timestamp_ms);
        self.postMessage({
          type: "result",
          result,
          inference_ms: performance.now() - started,
        });
      } finally {
        data.bitmap.close();
      }
    }
  } catch (error) {
    self.postMessage({ type: "error", error: String(error.message || error) });
  }
};
