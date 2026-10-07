// Map server monotonic time to the browser using the fastest recent round trip.
// A slow solve/status response must not move the estimated clock backwards.
export class StreamClock {
  update(session,server,requestedAt,receivedAt) {
    if(![server,requestedAt,receivedAt].every(Number.isFinite)||receivedAt<requestedAt)
      throw Error('控制时钟测量无效');
    if(this.session!==session){this.session=session;this.measurements=[];}
    this.measurements=this.measurements.filter(m=>receivedAt-m.receivedAt<30000);
    const measurement={server,local:(requestedAt+receivedAt)/2,rtt:receivedAt-requestedAt,receivedAt};
    this.measurements.push(measurement);
    this.anchor=this.measurements.reduce((best,m)=>m.rtt<=best.rtt?m:best);
    return this.anchor;
  }
  age(serverAt,now) {return now-this.anchor.local-(serverAt-this.anchor.server)*1000;}
}
