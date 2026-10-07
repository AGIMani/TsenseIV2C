export class WorkerSerial {
  constructor(clientId,changed=()=>{}) {
    this.clientId=clientId;this.changed=changed;this.active=false;this.port=null;this.error='';
    this.metrics={positionHz:0,rxKBs:0,exchangeHz:0};this.pending=new Map();this.seq=0;
  }
  init() {
    if(this.worker)return;
    this.worker=new Worker(new URL('./serial-worker.js',import.meta.url),{type:'module'});
    this.worker.onmessage=({data})=>{
      if(data.type==='state'){
        Object.assign(this,data.state); if(!data.state.connected && !this.connecting)this.port=null;
        this.changed();return;
      }
      const request=this.pending.get(data.id);if(!request)return;
      this.pending.delete(data.id);clearTimeout(request.timer);
      data.error?request.reject(Error(data.error)):request.resolve(data.result);
    };
    this.worker.onerror=event=>{
      this.error='串口 Worker 异常，请重新连接';this.active=false;this.port=null;
      for(const p of this.pending.values()){clearTimeout(p.timer);p.reject(Error(this.error));}
      this.pending.clear();this.worker.terminate();this.worker=null;clearInterval(this.heartbeat);this.changed();
    };
    this.heartbeat=setInterval(()=>this.worker?.postMessage({type:'heartbeat'}),250);
  }
  call(type,body={}) {
    this.init();const id=++this.seq;
    return new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>{
        this.pending.delete(id);
        this.worker?.postMessage({type:'disconnect',reason:'串口 Worker 请求超时，已取消连接'});
        reject(Error('串口 Worker 响应超时'));
      },6000);
      this.pending.set(id,{resolve,reject,timer});this.worker.postMessage({type,id,...body});
    });
  }
  async connect(port,model,baudRate) {
    if(this.port || this.closing)throw Error('请先断开当前串口');
    this.connecting=true;this.port=port;this.error='';
    try {await this.call('connect',{clientId:this.clientId,info:port.getInfo(),model,baudRate});}
    catch(error){this.port=null;this.error=error.message;throw error;}
    finally{this.connecting=false;}
  }
  disconnect(reason='') {
    if(this.closing)return this.closing;
    if(!this.worker)return Promise.resolve();
    this.active=false;
    this.closing=this.call('disconnect',{reason}).finally(()=>{this.port=null;this.closing=null;});
    return this.closing;
  }
  sampleMetrics(){return this.metrics;}
}
