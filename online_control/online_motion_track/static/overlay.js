// Normalized MediaPipe points -> mirrored, object-fit:contain video pixels.
export function landmarkPixel(point, width, height, videoWidth, videoHeight) {
  const scale = Math.min(width / videoWidth, height / videoHeight);
  const w = videoWidth * scale,
    h = videoHeight * scale;
  return [(width - w) / 2 + (1 - point.x) * w, (height - h) / 2 + point.y * h];
}

export class CameraOverlay {
  constructor(canvas, video) {
    this.canvas = canvas;
    this.video = video;
    this.hands = [];
    this.selected = -1;
    this.resize = new ResizeObserver(() => this.draw());
    this.resize.observe(canvas.parentElement);
  }
  setHands(hands = [], selected = -1) {
    this.hands = hands;
    this.selected = selected;
    this.draw();
  }
  clear() {
    this.setHands();
  }
  draw() {
    const { canvas, video } = this;
    const w = canvas.clientWidth,
      h = canvas.clientHeight;
    const dpr = Math.min(devicePixelRatio || 1, 2);
    if (
      canvas.width !== Math.round(w * dpr) ||
      canvas.height !== Math.round(h * dpr)
    ) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    if (!video.videoWidth || !video.videoHeight) return;
    this.hands.forEach((hand, index) => {
      if (
        hand.length !== 21 ||
        hand.some((p) => !Number.isFinite(p.x) || !Number.isFinite(p.y))
      )
        return;
      const p = hand.map((point) =>
        landmarkPixel(point, w, h, video.videoWidth, video.videoHeight),
      );
      const selected = index === this.selected;
      ctx.strokeStyle = selected ? "#42e4e7" : "#bec9cd";
      ctx.lineWidth = selected ? 2.2 : 1.5;
      ctx.shadowColor = "#071f2c";
      ctx.shadowBlur = 3;
      ctx.beginPath();
      for (const start of [1, 5, 9, 13, 17]) {
        const chain = [0, start, start + 1, start + 2, start + 3];
        ctx.moveTo(...p[0]);
        chain.slice(1).forEach((id) => ctx.lineTo(...p[id]));
      }
      ctx.moveTo(...p[5]);
      [9, 13, 17].forEach((id) => ctx.lineTo(...p[id]));
      ctx.stroke();
      ctx.shadowBlur = 0;
      p.forEach(([x, y], id) => {
        ctx.beginPath();
        ctx.arc(
          x,
          y,
          [4, 8, 12, 16, 20].includes(id) ? 4.3 : 3.2,
          0,
          2 * Math.PI,
        );
        ctx.fillStyle = selected ? "#edffff" : "#c2cfd4";
        ctx.fill();
        ctx.strokeStyle = selected ? "#05798e" : "#657e88";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.font = "10px system-ui";
        ctx.lineWidth = 2.5;
        ctx.strokeStyle = "#063343";
        ctx.strokeText(String(id), x + 6, y - 5);
        ctx.fillStyle = "#f1ffff";
        ctx.fillText(String(id), x + 6, y - 5);
      });
    });
  }
  dispose() {
    this.resize.disconnect();
    this.clear();
  }
}
