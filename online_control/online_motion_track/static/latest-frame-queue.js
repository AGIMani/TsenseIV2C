// One request in flight and one replaceable pending frame. No historic backlog.
export class LatestFrameQueue {
  constructor({process,onResult,onError,onDrop=()=>{},now=()=>performance.now(),maxAgeMs=120}) {
    Object.assign(this,{process,onResult,onError,onDrop,now,maxAgeMs});
    this.pending=null;this.running=false;this.closed=false;
    this.stats={offered:0,processed:0,replaced:0,expired:0};
  }
  offer(frame) {
    if(this.closed)return;
    this.stats.offered++;
    if(this.pending){this.stats.replaced++;this.onDrop(this.pending,'queue_replaced');}
    this.pending=frame;
    if(!this.running)void this.drain();
  }
  close(){
    if(this.closed)return;
    this.closed=true;
    if(this.pending)this.onDrop(this.pending,'queue_closed');
    if(this.current)this.onDrop(this.current,'request_cancelled');
    this.pending=this.current=null;
  }
  async drain(){
    if(this.running||this.closed)return;
    this.running=true;
    try{
      while(!this.closed&&this.pending){
        const frame=this.pending;this.pending=null;
        if(this.now()-frame.packet.timestamp_ms>this.maxAgeMs){this.stats.expired++;this.onDrop(frame,'queue_expired');continue;}
        frame.requestStartedAt=this.now();
        this.current=frame;
        const result=await this.process(frame);
        if(this.closed)return;
        this.current=null;
        this.stats.processed++;
        await this.onResult(frame,result);
      }
    }catch(error){
      if(!this.closed){
        if(this.current){this.onDrop(this.current,'request_failed',error);this.current=null;}
        this.close();await this.onError(error);
      }
    }finally{this.running=false;}
  }
}
