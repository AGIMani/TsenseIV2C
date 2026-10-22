// Display-only One Euro smoothing. Detection, recording and control keep raw
// samples. Coordinates and speed are measured in image-width units, so a
// portrait/landscape video or resized canvas does not change the tuning.
const cloneHands = hands => hands.map(hand => hand.map(point => ({...point})));
const validHand = hand => Array.isArray(hand) && hand.length === 21 &&
  hand.every(point => point && Number.isFinite(point.x) && Number.isFinite(point.y));

export class OverlaySmoothing {
  constructor() { this.reset(); }

  reset() {
    this.tracks = new Map();
    this.timestampMs = null;
    this.aspect = null;
    this.hands = [];
  }

  update(hands, timestampMs, labels = [], aspect = 1) {
    if (!Number.isFinite(timestampMs) || !Number.isFinite(aspect) || aspect <= 0) {
      this.reset();
      return [];
    }
    // A resize only redraws; repeated/late observations cannot advance state.
    if (this.timestampMs !== null && timestampMs <= this.timestampMs) {
      return cloneHands(this.hands);
    }
    const dt = this.timestampMs === null ? Infinity : (timestampMs - this.timestampMs) / 1000;
    if (dt > .3 || aspect !== this.aspect) this.tracks.clear();
    const nextTracks = new Map();
    const result = hands.map((hand, index) => {
      if (!validHand(hand)) return [];
      const output = hand.map(point => ({...point}));
      const label = labels[index];
      // MediaPipe may reorder its two results. Never identify a hand by array
      // index; ambiguous/duplicate labels are displayed fresh, without history.
      const key = (label === 'Left' || label === 'Right') &&
        labels.filter(value => value === label).length === 1 ? label :
        hands.length === 1 && !label ? 'single' : null;
      if (key === null) return output;
      const raw = hand.map(point => [point.x, point.y / aspect]);
      const center = [0, 5, 9, 13, 17].reduce((sum, id) =>
        [sum[0] + raw[id][0] / 5, sum[1] + raw[id][1] / 5], [0, 0]);
      let track = this.tracks.get(key);
      // Reacquiring a hand elsewhere must not drag a ghost across the image.
      if (track && Math.hypot(center[0] - track.center[0], center[1] - track.center[1]) > .35) track = null;
      if (!track) {
        track = {raw, center, value: raw.map(p => [...p]), velocity: raw.map(() => [0, 0])};
      } else {
        const inversePeriod = 1 / (2 * Math.PI * dt);
        const derivativeWeight = 2 / (2 + inversePeriod);
        for (let i = 0; i < 21; i++) {
          for (let axis = 0; axis < 2; axis++) {
            const derivative = (raw[i][axis] - track.raw[i][axis]) / dt;
            track.velocity[i][axis] += derivativeWeight * (derivative - track.velocity[i][axis]);
          }
          // Slow observations stay stable; fast movements open the bandwidth
          // immediately. One scalar XY weight, no window, queue or prediction.
          const speed = Math.max(0, Math.hypot(...track.velocity[i]) - .025);
          const cutoff = Math.min(60, 1.5 + 50 * speed);
          const weight = cutoff / (cutoff + inversePeriod);
          for (let axis = 0; axis < 2; axis++) {
            track.value[i][axis] += weight * (raw[i][axis] - track.value[i][axis]);
          }
          output[i].x = track.value[i][0];
          output[i].y = track.value[i][1] * aspect;
        }
        track.raw = raw;
        track.center = center;
      }
      nextTracks.set(key, track);
      return output;
    });
    // Missing hands are removed immediately. Re-entry starts at the new sample.
    this.tracks = nextTracks;
    this.timestampMs = timestampMs;
    this.aspect = aspect;
    this.hands = result;
    return cloneHands(result);
  }
}
