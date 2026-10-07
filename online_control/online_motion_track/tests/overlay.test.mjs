import test from "node:test";
import assert from "node:assert/strict";
import { landmarkPixel } from "../static/overlay.js";

test("landmarks follow mirrored preview and vertical/horizontal letterboxing", () => {
  assert.deepEqual(landmarkPixel({ x: 0, y: 0 }, 640, 480, 640, 480), [640, 0]);
  assert.deepEqual(landmarkPixel({ x: 1, y: 1 }, 640, 480, 640, 480), [0, 480]);
  assert.deepEqual(
    landmarkPixel({ x: 0.25, y: 0.5 }, 640, 480, 640, 480),
    [480, 240],
  );
  assert.deepEqual(
    landmarkPixel({ x: 0, y: 0 }, 640, 640, 640, 480),
    [640, 80],
  );
  assert.deepEqual(
    landmarkPixel({ x: 1, y: 1 }, 800, 480, 640, 480),
    [80, 480],
  );
});
