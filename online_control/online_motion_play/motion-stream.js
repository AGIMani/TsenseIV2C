// Playback push transport. Fixed parser storage; no request per sample.
export async function consumeRecords(response, accept, {signal} = {}) {
  if (!response.ok) throw Error((await response.json()).error || response.statusText);
  const reader = response.body.getReader();
  const buffer = new Uint8Array(1048576);
  let length = 0;
  const abort = () => reader.cancel().catch(() => {});
  signal?.addEventListener('abort', abort, {once:true});
  try {
    while (!signal?.aborted) {
      const {value, done} = await reader.read();
      if (done) { if (!signal?.aborted) throw Error('下发数据流已断开'); break; }
      if (length + value.length > buffer.length) throw Error('下发数据流拥塞');
      buffer.set(value,length); length += value.length;
      let offset = 0;
      while (offset + 4 <= length) {
        const size = new DataView(buffer.buffer, buffer.byteOffset + offset, 4).getUint32(0,true);
        if (!size || size > 264000) throw Error('下发数据流长度无效');
        if (offset + 4 + size > length) break;
        const record = buffer.subarray(offset+4,offset+4+size);
        // The callback must copy retained data before the buffer is reused.
        // The USB consumer coalesces obsolete positions and validates frames.
        if (!signal?.aborted) await accept(record);
        offset += 4 + size;
      }
      buffer.copyWithin(0,offset,length); length -= offset;
    }
  } finally { signal?.removeEventListener('abort',abort); await reader.cancel().catch(()=>{}); reader.releaseLock(); }
}
