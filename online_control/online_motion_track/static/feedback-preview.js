const FRESHNESS_MS = 500;
const RETRY_MS = 500;

// Only measured joints are sent to the viewer. Missing joints and disconnected
// devices leave the displayed angles untouched until new measurements arrive.
export class FeedbackPreview {
  constructor({ request, display, changed, now = () => performance.now() }) {
    this.request = request;
    this.display = display;
    this.changed = changed;
    this.now = now;
    this.generation = 0;
    this.config = null;
    this.connected = false;
    this.supported = true;
    this.disposed = false;
    this.inFlight = null;
    this.clearMeasurement();
  }

  configure(modelId, side, jointNames) {
    if (this.disposed) return;
    this.generation += 1;
    this.config = { modelId, side, jointNames: [...new Set(jointNames)] };
    this.clearMeasurement();
    this.publish();
  }

  clearMeasurement({ preservePose = false } = {}) {
    this.sample = null;
    if (!preservePose) {
      this.hasPose = false;
      this.poseSample = null;
    }
    this.outcome = null;
    this.attemptedSample = null;
    this.retryAt = Infinity;
  }

  update({ connected, telemetry, receivedAt, identity, supported = true }) {
    if (this.disposed) return;
    const nextConnected = Boolean(connected);
    const nextSupported = Boolean(supported);
    if (nextConnected !== this.connected || nextSupported !== this.supported) {
      this.generation += 1;
      this.clearMeasurement({ preservePose: true });
    }
    this.connected = nextConnected;
    this.supported = nextSupported;
    if (this.connected && this.supported) {
      if (telemetry != null && Number.isFinite(receivedAt)) {
        const key = identity === undefined ? telemetry : identity;
        const timestamp = receivedAt;
        if (!this.sample || !Object.is(this.sample.key, key)) {
          this.sample = { key, telemetry, receivedAt: timestamp };
        } else {
          // HTTP polling can return the same device measurement in a new object.
          // Its identity and acquisition time, not the poll time, set freshness.
          this.sample.receivedAt = Math.min(this.sample.receivedAt, timestamp);
        }
      } else if (this.sample) {
        this.generation += 1;
        this.sample = null;
        this.outcome = null;
        this.attemptedSample = null;
      }
    }
    this.publish();
  }

  age(sample = this.sample) {
    return sample ? Math.max(0, this.now() - sample.receivedAt) : null;
  }

  publish() {
    if (this.disposed) return;
    // New raw packets do not make the pose already on screen any younger, and
    // do not establish that the last conversion failure has recovered.
    const ageMs = this.hasPose ? this.age(this.poseSample) : this.age();
    const outcome = this.outcome;
    let status;
    if (!this.supported) status = 'unsupported';
    else if (!this.connected) status = 'disconnected';
    else if (!this.sample) status = 'waiting';
    else if (this.age() >= FRESHNESS_MS) status = 'stale';
    else if (outcome?.status === 'incomplete' || outcome?.status === 'error') status = outcome.status;
    else if (this.hasPose && ageMs >= FRESHNESS_MS) status = 'stale';
    else status = outcome?.status || 'waiting';
    this.changed({
      status,
      ageMs,
      missingCount: outcome?.missingCount || 0,
      hasPose: this.hasPose,
      error: outcome?.error || null,
    });
  }

  active(generation) {
    return !this.disposed && this.connected && this.supported && generation === this.generation;
  }

  tick() {
    if (this.disposed) return;
    this.publish();
    if (this.inFlight) return this.inFlight;
    const sample = this.sample;
    const config = this.config;
    if (!this.connected || !this.supported || !sample || !config?.jointNames.length) return;
    if (this.age(sample) >= FRESHNESS_MS) return;
    if (sample === this.attemptedSample && this.now() < this.retryAt) return;

    const generation = this.generation;
    this.attemptedSample = sample;
    this.retryAt = Infinity;
    const work = Promise.resolve()
      .then(() => {
        if (!this.active(generation) || this.age(sample) >= FRESHNESS_MS) return null;
        return this.request({ model_id: config.modelId, side: config.side, telemetry: sample.telemetry });
      })
      .then(result => {
        if (!this.active(generation)) return;
        if (this.age(sample) >= FRESHNESS_MS) {
          this.outcome = { sample, status: 'stale' };
          this.publish();
          return;
        }
        const pose = result?.pose;
        if (!pose || typeof pose !== 'object' || Array.isArray(pose)) {
          throw new Error('遥测转换未返回有效关节姿态');
        }
        const reportedMissing = new Set(result.missing_joints || []);
        const measuredNames = config.jointNames.filter(name => Number.isFinite(pose[name]) && !reportedMissing.has(name));
        const missingCount = config.jointNames.length - measuredNames.length;
        if (measuredNames.length) {
          this.display(Object.fromEntries(measuredNames.map(name => [name, pose[name]])));
          this.hasPose = true;
          this.poseSample = sample;
        }
        this.outcome = { sample, status: missingCount ? 'incomplete' : 'live', missingCount };
        this.publish();
      })
      .catch(error => {
        if (!this.active(generation)) return;
        this.retryAt = this.now() + RETRY_MS;
        this.outcome = { sample, status: 'error', error: error?.message || String(error) };
        this.publish();
      })
      .finally(() => {
        if (this.inFlight === work) this.inFlight = null;
      });
    this.inFlight = work;
    return work;
  }

  dispose() {
    this.disposed = true;
    this.generation += 1;
    this.clearMeasurement();
  }
}
