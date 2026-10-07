export class DetectorClient {
  constructor() {
    this.worker = new Worker("/detector.worker.js", { type: "module" });
    this.pending = null;
    this.worker.onmessage = ({ data }) => {
      if (!this.pending) return;
      const { resolve, reject, timer } = this.pending;
      clearTimeout(timer);
      this.pending = null;
      data.type === "error" ? reject(new Error(data.error)) : resolve(data);
    };
    this.worker.onerror = (event) =>
      this.fail(new Error(event.message || "识别线程初始化失败"));
  }
  fail(error) {
    if (this.pending) {
      clearTimeout(this.pending.timer);
      this.pending.reject(error);
      this.pending = null;
    }
  }
  ask(message, transfer = []) {
    if (this.pending) return Promise.reject(new Error("识别帧仍在处理中"));
    return new Promise((resolve, reject) => {
      const timer = setTimeout(
        () => this.fail(new Error("识别线程响应超时，请重新开始")),
        45000,
      );
      this.pending = { resolve, reject, timer };
      this.worker.postMessage(message, transfer);
    });
  }
  close() {
    this.fail(new Error("识别已停止"));
    this.worker.terminate();
  }
}

