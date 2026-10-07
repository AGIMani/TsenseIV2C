// Bounded serial flight recorder. Raw bytes are formatted only on errors/close.
import {intervalSummary} from '../../shared/serial-timing.js';
const details=e=>({name:e?.name||'Error',message:e?.message||String(e),stack:e?.stack||''});
export class SerialTrace {
  constructor({emit,now=()=>performance.now(),context=()=>({})}){
    this.emit=emit;this.now=now;this.context=context;
    this.id=globalThis.crypto?.randomUUID?.()||`${Date.now()}-${Math.random()}`;
    this.started=this.lastReport=now();this.firstError=null;this.ring=new Array(64);this.cursor=0;this.count=0;
    this.txCount=0;this.txBytes=0;this.txFailed=0;this.rxCount=0;this.rxBytes=0;this.maxWriteMs=0;
    this.lastRx=null;this.lastTx=null;
    this.positionIntervals=new Float64Array(4096);this.positionIntervalCount=0;this.lastPositionStart=null;
  }
  remember(direction,bytes,extra={}){
    this.ring[this.cursor]={at_ms:this.now(),direction,length:bytes.length,bytes:bytes.slice(0,256),...extra};
    this.cursor=(this.cursor+1)%this.ring.length;this.count=Math.min(this.count+1,this.ring.length);
  }
  received(bytes){this.rxCount++;this.rxBytes+=bytes.length;this.lastRx=this.now();this.remember('rx',bytes);}
  written(bytes,started,sent){
    const duration=this.now()-started;this.maxWriteMs=Math.max(this.maxWriteMs,duration);
    if(sent){this.txCount++;this.txBytes+=bytes.length;this.lastTx=this.now();}else this.txFailed++;
    if(sent&&bytes[5]===0x20){
      if(this.lastPositionStart!==null)this.positionIntervals[this.positionIntervalCount++%this.positionIntervals.length]=started-this.lastPositionStart;
      this.lastPositionStart=started;
    }
    this.remember('tx',bytes,{duration_ms:duration,sent});
  }
  recent(){
    return Array.from({length:this.count},(_,i)=>{
      const {bytes,...row}=this.ring[(this.cursor-this.count+i+this.ring.length)%this.ring.length];
      return {...row,hex:Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join(''),truncated:bytes.length<row.length};
    });
  }
  summary(){const at=this.now();return {tx_count:this.txCount,tx_bytes:this.txBytes,tx_failed:this.txFailed,
    rx_chunks:this.rxCount,rx_bytes:this.rxBytes,max_write_ms:this.maxWriteMs,
    position_write_intervals:intervalSummary(this.positionIntervals.subarray(0,Math.min(this.positionIntervalCount,this.positionIntervals.length))),
    rx_age_ms:this.lastRx===null?null:at-this.lastRx,tx_age_ms:this.lastTx===null?null:at-this.lastTx};}
  event(type,data={}){
    // Diagnostics must never replace a serial failure or reject a control write.
    try{this.emit(type,{serial_session_id:this.id,serial_elapsed_ms:this.now()-this.started,
      ...this.context(),...data});}catch{}
  }
  error(phase,error,extra={}){
    const detail={phase,...details(error),at_ms:this.now()};
    this.firstError??=detail;
    this.event('serial_io_error',{error:detail,first_error:this.firstError,io:this.summary(),recent_io:this.recent(),...extra});
  }
  flush(force=false){
    if(!force&&this.now()-this.lastReport<1000)return;
    this.lastReport=this.now();this.event('serial_io_stats',{io:this.summary(),first_error:this.firstError});
  }
}
