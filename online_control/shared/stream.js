// Common push transport for playback and tracking. No request per sample.
export async function consumeRecords(response, accept, {signal} = {}) {
  if (!response.ok) throw Error((await response.json()).error || response.statusText);
  const reader = response.body.getReader();
  let buffer = new Uint8Array(0);
  const abort = () => reader.cancel().catch(() => {});
  signal?.addEventListener('abort', abort, {once:true});
  try {
    while (!signal?.aborted) {
      const {value, done} = await reader.read();
      if (done) { if (!signal?.aborted) throw Error('下发数据流已断开'); break; }
      if (buffer.length + value.length > 1048576) throw Error('下发数据流拥塞');
      if (!buffer.length) buffer = value;
      else { const next = new Uint8Array(buffer.length + value.length); next.set(buffer); next.set(value,buffer.length); buffer = next; }
      let offset = 0;
      while (offset + 4 <= buffer.length) {
        const size = new DataView(buffer.buffer, buffer.byteOffset + offset, 4).getUint32(0,true);
        if (!size || size > 264000) throw Error('下发数据流长度无效');
        if (offset + 4 + size > buffer.length) break;
        const record = buffer.subarray(offset+4,offset+4+size);
        // A network chunk can contain many valid 2ms samples. Preserve them;
        // callers enforce sample age before USB instead of discarding the batch.
        if (!signal?.aborted) await accept(record);
        offset += 4 + size;
      }
      buffer = buffer.slice(offset);
    }
  } finally { signal?.removeEventListener('abort',abort); await reader.cancel().catch(()=>{}); reader.releaseLock(); }
}
