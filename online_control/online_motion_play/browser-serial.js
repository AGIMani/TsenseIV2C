import {isA30Model} from "../shared/hand-models.js";
import {serialOptions,portDescription} from '../shared/serial-options.js';
// Physical USB is owned exclusively by the visitor's browser. Protocol helpers
// match online_control/online_motion_track; the server retains the shared Form2 planners.
import {consumeRecords} from './motion-stream.js';
import {PositionGuard,makePacer} from './position-guard.js';
import {intervalSummary} from '../shared/serial-timing.js';
import * as v2 from './serial-protocol.js';
import * as a30 from './a30-serial-protocol.js';

const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const encode = bytes => btoa(Array.from(bytes, n => String.fromCharCode(n)).join(''));
const decode = value => Uint8Array.from(atob(value), c => c.charCodeAt(0));
const textEncoder = new TextEncoder(), textDecoder = new TextDecoder();

export function serialRequest(token, seq, chunks, size) {
  if (!/^[a-f0-9]{32}$/.test(token) || seq > 0xffffffff) throw Error('串口会话无效');
  const bytes = new Uint8Array(40 + size);
  bytes.set(textEncoder.encode('MSR1' + token));
  new DataView(bytes.buffer).setUint32(36, seq, true);
  let offset = 40;
  for (const chunk of chunks) { bytes.set(chunk, offset); offset += chunk.length; }
  return bytes;
}

export function serialResponse(bytes, expectedToken) {
  if (!(bytes instanceof Uint8Array) || bytes.length < 42 || bytes.length > 262144 + 1578 ||
      bytes[0]!==77 || bytes[1]!==83 || bytes[2]!==84 || bytes[3]!==49) throw Error('串口响应格式无效');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if(expectedToken)for(let i=0;i<32;i++)if(bytes[i+4]!==expectedToken.charCodeAt(i))throw Error('串口会话已变化');
  const token = expectedToken || textDecoder.decode(bytes.subarray(4, 36)), seq = view.getUint32(36, true);
  const count = view.getUint16(40, true), frames = [];
  if (count > 256) throw Error('串口响应帧数无效');
  let offset = 42;
  for (let i = 0; i < count; i++) {
    if (offset + 6 > bytes.length) throw Error('串口响应截断');
    const size = view.getUint16(offset, true), age_ms = view.getFloat32(offset+2, true); offset += 6;
    if (size < 8 || size > 1024 || offset + size > bytes.length) throw Error('串口响应帧长度无效');
    frames.push({bytes:bytes.subarray(offset, offset+size), age_ms}); offset += size;
  }
  if (offset !== bytes.length) throw Error('串口响应多余数据');
  return {token, seq, frames};
}
const bounded = async (promise, ms, message) => {
  let timer;
  try { return await Promise.race([promise, new Promise((_, reject) => { timer = setTimeout(() => reject(Error(message)), ms); })]); }
  finally { clearTimeout(timer); }
};

export function serialSupport() {
  if (!globalThis.isSecureContext) return '本机串口需要 HTTPS：请使用部署后的 HTTPS 地址，并信任公司的根证书。';
  if (!globalThis.navigator?.serial) return '此浏览器不支持本机串口，请用电脑上的 Chrome 或 Edge 打开此页面。';
  return '';
}

export function emergencyFrames(model) {
  if (isA30Model(model)) return Array.from({length:18}, (_, slot) => {
    const [bus, node] = a30.address(slot);
    return [a30.home(bus, node, 2, 0), a30.power(bus, node, 2)];
  }).flat();
  return [6,6,2].flatMap((count, bus) => Array.from({length:count}, (_, i) =>
    [v2.linearHome(bus,i+1,2,0), v2.linearPower(bus,i+1,false)]).flat())
    .concat(Array.from({length:4}, (_, bus) => v2.rotaryDrive(bus,0,0)));
}

export class BrowserSerial {
  constructor(api, changed = () => {}) {
    this.api = api; this.changed = changed; this.active = false; this.port = null;
    this.generation = 0; this.tail = Promise.resolve(); this.rxChunks = []; this.rxSize = 0;
    this.txCount = 0; this.rxCount = 0; this.closing = null; this.error = '';
    this.positionCount = 0; this.exchangeCount = 0; this.rttMs = 0;
    this.metrics = {positionHz:0, rxKBs:0, exchangeHz:0};
  }
  async connect(port, model, baudRate = 115200) {
    if (this.port || this.closing) throw Error('请先断开当前本机串口');
    this.port = port; this.deviceLabel = portDescription(port); this.baudRate=baudRate; this.model = model; this.error = ''; this.seq = 0; this.token = null;
    this.rxChunks = []; this.rxSize = 0; this.txCount = this.rxCount = 0;
    this.positionCount = this.exchangeCount = 0; this.rttMs = 0; this.rateSample = null;
    this.metrics = {positionHz:0, rxKBs:0, exchangeHz:0};
    const generation = ++this.generation;
    try {
      this.opening = port.open(serialOptions(baudRate));
      await this.opening;
      if (generation !== this.generation) throw Error('连接已取消');
      this.writer = port.writable.getWriter(); this.reader = port.readable.getReader();
      // Same CDC line configuration as the desktop Form2 transport.
      await port.setSignals({dataTerminalReady:true,requestToSend:false});
      const response = await bounded(this.api('connect',{port:'browser'}), 2500, '建立控制会话超时');
      if (generation !== this.generation) throw Error('连接已取消');
      this.streaming = !!response.result.stream;
      this.guard = response.result.max_velocity ? new PositionGuard(response.result.max_velocity) : null;
      this.pacer = makePacer(); this.lastPositionAt = null;
      this.intervals = new Float64Array(30000); this.intervalCount = 0;
      this.nextPositionAt = 0;
      this.pendingRecords=[];this.recordPool=[];this.draining=false;this.qScratch=new Float32Array(20);
      this.token = response.result.token; this.binary = response.result.transport === 'binary-v1';
      this.active = true; this.lastExchange = this.lastStream = performance.now();
      this.sampleMetrics();
      this.reading = this.readLoop(generation);
      this.timer = setInterval(() => {
        if (this.active && (performance.now()-this.lastExchange > 350 || this.streaming && performance.now()-this.lastStream > 200)) this.disconnect('控制链路超时，已停止并断开本机串口');
      },25);
      this.abortStream = new AbortController(); this.uplinkTimes = new Map([[0,performance.now()]]);
      this.exchanging = this.streaming ? this.uplinkLoop(generation) : this.exchangeLoop(generation);
      if (this.streaming) this.receiving = this.streamLoop(generation); this.changed();
    } catch (error) { await this.disconnect(error.message); throw error; }
  }
  async readLoop(generation) {
    try {
      while (this.active && generation === this.generation) {
        const {value,done} = await this.reader.read();
        if (!this.active || generation !== this.generation) break;
        if (done) throw Error('本机串口已拔出或关闭');
        if (this.rxSize + value.length > 65536) throw Error('本机遥测拥塞，已停止');
        if (value.length) { this.rxChunks.push(value); this.rxSize += value.length; this.rxCount += value.length; }
      }
    } catch (error) { if (this.active) void this.disconnect(error.message); }
    finally { try { this.reader?.releaseLock(); } catch {} }
  }
  async exchangeLoop(generation) {
    try {
      while (this.active && generation === this.generation) {
        const started = performance.now(), chunks = this.rxChunks, size = this.rxSize;
        this.rxChunks = []; this.rxSize = 0;
        const seq = ++this.seq;
        let response, packet;
        if (this.binary) {
          response = await bounded(this.api('serial_binary',serialRequest(this.token,seq,chunks,size)),180,'串口网络往返超时');
          packet = serialResponse(response);
        } else {
          const rx = new Uint8Array(size); let offset = 0;
          for (const chunk of chunks) { rx.set(chunk,offset); offset += chunk.length; }
          response = await bounded(this.api('serial',{token:this.token,seq,rx:encode(rx)}),180,'串口网络往返超时');
          packet = response.result;
        }
        if (!this.active || generation !== this.generation) break;
        if (packet.token !== this.token || packet.seq !== seq) throw Error('串口会话已变化');
        if (performance.now()-started > 150) throw Error('内网延迟超过 150 ms，已停止下发');
        this.lastExchange = performance.now();
        this.rttMs = this.lastExchange-started; this.exchangeCount++;
        await this.writePacket(packet, generation, started);
        // Binary requests wait server-side for new frames (up to 10 ms).
        // fetch supplies the event-loop yield without nested timer throttling.
        if (!this.binary) await sleep(Math.max(0,2-(performance.now()-started)));
      }
    } catch (error) { if (this.active && generation === this.generation) await this.disconnect(error.message); }
  }
  async writePacket(packet, generation, started) {
        // Validate the complete network record before any of its commands are
        // written, even though position frames subsequently use separate writes.
        for (const item of packet.frames) {
          let bytes=item.bytes||decode(item.data);
          if(bytes.length===192 && bytes[0]===74 && bytes[1]===81 && bytes[2]===80 && bytes[3]===49){
            const view=new DataView(bytes.buffer,bytes.byteOffset+4,80);
            for(let i=0;i<20;i++)if(!Number.isFinite(view.getFloat32(i*4,true)))throw Error('轨迹关节数据无效');
            bytes=bytes.subarray(84);
          }
          if(!Number.isFinite(item.age_ms)||item.age_ms<0||item.age_ms+performance.now()-started>200)throw Error('下发指令已过期');
          if(bytes.length<8||bytes.length>1024||bytes[0]!==0xaa||bytes[1]!==0x55||
            (bytes[2]|bytes[3]<<8)!==bytes.length-8||bytes[4]!==3||v2.crc16(bytes.subarray(0,-2))!==(bytes.at(-2)|bytes.at(-1)<<8))throw Error('下发帧校验失败');
        }
        const writes = []; let length = 0, deadline = Infinity, positions = 0;
        for (const item of packet.frames) {
          if (!this.active || generation !== this.generation) return;
          if (!Number.isFinite(item.age_ms) || item.age_ms < 0 || item.age_ms + performance.now()-started > 200) throw Error('下发指令已过期');
          let bytes = item.bytes || decode(item.data), q = null;
          if (bytes.length === 192 && bytes[0]===74 && bytes[1]===81 && bytes[2]===80 && bytes[3]===49) {
            const view = new DataView(bytes.buffer,bytes.byteOffset+4,80);
            q = this.qScratch; for(let i=0;i<20;i++)q[i]=view.getFloat32(i*4,true);
            bytes = bytes.subarray(84);
          }
          // Position samples are written individually at a target 2ms spacing.
          // Never let a network batch turn into a USB burst of old positions.
          if (bytes[5]===0x20) {
            if (length) {
              for (const command of writes) await this.write(command,generation,deadline);
              this.txCount += writes.length; writes.length=0; length=0;
            }
            await this.write(bytes,generation,started+200-item.age_ms,{q,position:true});
            if (!this.active || generation!==this.generation) return;
            this.txCount++; this.positionCount++;
            continue;
          }
          writes.push(bytes); length += bytes.length;
          deadline = Math.min(deadline, started + 200 - item.age_ms);
        }
        if (length) {
          // Preserve enable/home order while issuing a single Web Serial write.
          let bytes = writes[0];
          if (writes.length > 1) {
            bytes = new Uint8Array(length); let offset = 0;
            for (const frame of writes) { bytes.set(frame,offset); offset += frame.length; }
          }
          await this.write(bytes, generation, deadline);
          if (!this.active || generation !== this.generation) return;
          this.txCount += writes.length; this.positionCount += positions;
        }
  }
  async uplinkLoop(generation) {
    try {
      while (this.active && generation === this.generation) {
        const started = performance.now(), seq = ++this.seq;
        const chunks = this.rxChunks, size = this.rxSize; this.rxChunks = []; this.rxSize = 0;
        this.uplinkTimes.set(seq,started);
        for (const [id, at] of this.uplinkTimes) if (started-at > 1000) this.uplinkTimes.delete(id);
        const response = await bounded(this.api('serial_rx',serialRequest(this.token,seq,chunks,size)),180,'遥测上传超时');
        if (!this.active || generation !== this.generation) return;
        const packet = serialResponse(response);
        if (packet.token !== this.token || packet.seq !== seq || packet.frames.length) throw Error('遥测会话已变化');
        this.lastExchange = performance.now(); this.rttMs = this.lastExchange-started; this.exchangeCount++;
        if (this.rttMs > 150) throw Error('网络往返超过 150 ms');
        await sleep(Math.max(0,20-(performance.now()-started)));
      }
    } catch (error) { if (this.active && generation === this.generation) await this.disconnect(error.message); }
  }
  async streamLoop(generation) {
    try {
      const response = await this.api('serial_stream',{token:this.token,signal:this.abortStream.signal});
      await consumeRecords(response, bytes => {
        if (!this.active || generation !== this.generation) return;
        let storage=this.recordPool.pop();
        if(!storage||storage.length<bytes.length)storage=new Uint8Array(Math.max(1024,bytes.length));
        storage.set(bytes);
        const packet = serialResponse(storage.subarray(0,bytes.length),this.token), started = this.uplinkTimes.get(packet.seq);
        if (packet.token !== this.token || started === undefined)throw Error('下发数据流会话无效');
        if(performance.now()-started>150)throw Error(`下发网络积压 ${Math.round(performance.now()-started)} ms，已停止`);
        this.lastStream = performance.now();
        // The reusable parser buffer is valid only during this callback.
        // Retain one bounded record; independently drain USB to avoid replaying
        // a TCP backlog. Coalesce only adjacent position-only records.
        const positionOnly=packet.frames.length>0&&packet.frames.every(f=>f.bytes[5]===0x20||
          f.bytes.length===192&&f.bytes[0]===74&&f.bytes[1]===81&&f.bytes[2]===80&&f.bytes[3]===49);
        if(positionOnly && this.pendingRecords.at(-1)?.positionOnly)this.recordPool.push(this.pendingRecords.pop().storage);
        if(this.pendingRecords.length>=64)throw Error('串口命令积压，已停止');
        this.pendingRecords.push({packet,storage,started,received:performance.now(),positionOnly});
        void this.drainRecords(generation);
      }, {signal:this.abortStream.signal});
    } catch (error) { if (this.active && generation === this.generation) await this.disconnect(error.message); }
  }
  async drainRecords(generation){
    if(this.draining)return;this.draining=true;
    try{
      while(this.active&&generation===this.generation&&this.pendingRecords.length){
        const record=this.pendingRecords.shift();
        if(record.positionOnly&&performance.now()-record.received>20)throw Error('位置发送积压超过20 ms，已停止');
        try{await this.writePacket(record.packet,generation,record.started);}
        finally{this.recordPool.push(record.storage);}
      }
    }catch(error){if(this.active)void this.disconnect(error.message);}
    finally{this.draining=false;}
  }
  sampleMetrics() {
    const now = performance.now();
    if (!this.rateSample) this.rateSample = {at:now, positions:this.positionCount, rx:this.rxCount, exchanges:this.exchangeCount};
    const dt = (now-this.rateSample.at)/1000;
    if (dt >= 1) {
      this.metrics = {positionHz:(this.positionCount-this.rateSample.positions)/dt,
        rxKBs:(this.rxCount-this.rateSample.rx)/dt/1024, exchangeHz:(this.exchangeCount-this.rateSample.exchanges)/dt,
        position_write_intervals:intervalSummary(this.intervals.subarray(0,Math.min(this.intervalCount,this.intervals.length)))};
      this.rateSample = {at:now, positions:this.positionCount, rx:this.rxCount, exchanges:this.exchangeCount};
    }
    return this.metrics;
  }
  write(bytes, generation, deadline = Infinity, sample = null) {
    const next = this.tail.then(async () => {
      if (generation !== this.generation || !this.writer) return;
      if (sample?.position) {
        if(sample.q&&this.guard)this.guard.check(sample.q);
        // Like Form2, the planner owns smoothing. Do not turn a skipped
        // network sample into a jump error or a delta-dependent expiry here.
        const earliest=this.nextPositionAt;
        while (performance.now() < earliest) await this.pacer.wait(earliest-performance.now());
        if (generation !== this.generation || !this.active) return;
        const now=performance.now();
        sample.at=now;
      }
      if (performance.now() > deadline) throw Error('下发指令排队过期');
      try { await bounded(this.writer.write(bytes),150,'本机串口写入超时'); }
      catch (error) { void this.writer.abort(error).catch(() => {}); throw error; }
      if (sample?.position) {
        if (sample.q && this.guard) this.guard.commit(sample.q,sample.at);
        if (this.lastPositionAt!==null) this.intervals[this.intervalCount++ % this.intervals.length]=sample.at-this.lastPositionAt;
        this.lastPositionAt=sample.at;
        this.nextPositionAt=sample.at+2;
      }
    });
    this.tail = next.catch(() => {}); return next;
  }
  disconnect(reason = '') {
    if (this.closing) return this.closing;
    const wasActive = this.active;
    this.active = false; const generation = ++this.generation;
    clearInterval(this.timer); this.abortStream?.abort(); this.error = reason; this.changed();
    this.closing = (async () => {
      try { await this.opening; } catch {}
      // Cancel autonomous homing locally even when node0/network is unavailable.
      // Never issue a zero-position jump. Model-specific opcodes must not mix.
      if (wasActive && this.writer) {
        try {
          const frames = emergencyFrames(this.model);
          const bytes = new Uint8Array(frames.reduce((sum, frame) => sum+frame.length,0));
          let at = 0; for (const frame of frames) { bytes.set(frame,at); at += frame.length; }
          await this.write(bytes,generation);
        } catch { this.error = '停止指令未能写入 USB，请检查本机设备连接'; }
      }
      if (this.token) {
        try { await bounded(this.api('disconnect',{token:this.token}),1000,'断开会话超时'); } catch {}
      }
      try { await this.reader?.cancel(); await this.reading; } catch {}
      try { this.reader?.releaseLock(); } catch {}
      try { await this.tail; this.writer?.releaseLock(); } catch {}
      this.pacer?.close();
      try { await this.port?.close(); } catch {}
      this.writer = this.reader = this.port = null; this.rxChunks = []; this.rxSize = 0;
      this.changed();
    })().finally(() => { this.closing = null; });
    return this.closing;
  }
}
