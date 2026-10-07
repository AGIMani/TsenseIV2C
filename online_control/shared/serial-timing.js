// Each timer starts from a message task, avoiding nested timers' 4 ms floor.
// This is host scheduling, not a guarantee of USB/H7 arrival time.
export function makePacer() {
  const channel = typeof MessageChannel === 'function' ? new MessageChannel() : null;
  const jobs = new Set();
  let closed = false;
  const start = job => {
    if (!jobs.has(job)) return;
    job.timer = setTimeout(() => { jobs.delete(job); job.resolve(); }, Math.max(0, job.ms));
  };
  if (channel) channel.port1.onmessage = () => {
    for (const job of jobs) if (job.timer === undefined) start(job);
  };
  return {
    wait(ms) {
      if (closed) return Promise.resolve();
      return new Promise(resolve => {
        const job = {ms, resolve}; jobs.add(job);
        if (channel) channel.port2.postMessage(0); else start(job);
      });
    },
    close() {
      closed = true;
      channel?.port1.close(); channel?.port2.close();
      for (const job of jobs) { clearTimeout(job.timer); job.resolve(); }
      jobs.clear();
    }
  };
}

export function intervalSummary(values) {
  if (!values.length) return {samples:0};
  const sorted = Array.from(values).sort((a,b) => a-b);
  const percentile = p => sorted[Math.ceil(sorted.length*p)-1];
  return {samples:sorted.length, min_ms:sorted[0], max_ms:sorted.at(-1),
    p50_ms:percentile(.5), p95_ms:percentile(.95), p99_ms:percentile(.99),
    within_1_3_ms_percent:100*sorted.filter(ms=>ms>=1&&ms<=3).length/sorted.length};
}
