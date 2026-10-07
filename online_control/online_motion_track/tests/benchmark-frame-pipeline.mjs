// Deterministic scheduling model, NOT a live camera or hardware measurement.
// Uses the production queue; no wall-clock sleeps or browser dependencies.
import assert from 'node:assert/strict';
import {LatestFrameQueue} from '../static/latest-frame-queue.js';

class Clock {
  at=0;events=[];serial=0;
  later(delay,fn){this.events.push({at:this.at+delay,fn,serial:this.serial++});}
  async run(until){
    while(this.events.length){
      this.events.sort((a,b)=>a.at-b.at||a.serial-b.serial);
      const event=this.events.shift();
      if(event.at>until)break;
      this.at=event.at;event.fn();
      // Flush async process/onResult continuations at the same virtual instant.
      for(let i=0;i<8;i++)await Promise.resolve();
    }
  }
}
const median=values=>{const a=[...values].sort((a,b)=>a-b);return a[Math.floor(a.length/2)];};
async function simulate({overlap,apiMs,inferMs=16,fps=30}){
  const clock=new Clock(),period=1000/fps,delivered=[],ages=[];
  let sequence=0,inFlight=0,peak=0,inferences=0;
  const nextFrame=()=>clock.later((Math.floor(clock.at/period+1e-9)+1)*period-clock.at,capture);
  const queue=new LatestFrameQueue({now:()=>clock.at,
    process:frame=>new Promise(resolve=>{
      peak=Math.max(peak,++inFlight);
      clock.later(apiMs,()=>{inFlight--;resolve({});});
    }),
    onResult:frame=>{delivered.push(clock.at);ages.push(clock.at-frame.packet.timestamp_ms);if(!overlap)nextFrame();},
    onError:error=>{throw error;},
  });
  function capture(){
    const frame={packet:{timestamp_ms:clock.at},id:sequence++};
    inferences++;
    clock.later(inferMs,()=>{queue.offer(frame);if(overlap)nextFrame();});
  }
  nextFrame();await clock.run(10000);queue.close();
  const intervals=delivered.slice(1).map((at,i)=>at-delivered[i]);
  return {fps,api_ms:apiMs,inference_ms:inferMs,inferences,delivered:delivered.length,
    median_target_interval_ms:median(intervals),median_capture_to_result_ms:median(ages),
    max_capture_to_result_ms:Math.max(...ages),peak_requests:peak,queue:queue.stats};
}
const result={kind:'virtual-clock model, not live camera',cases:[]};
for(const apiMs of [20,50,180]){
  const serial=await simulate({overlap:false,apiMs});
  const parallel=await simulate({overlap:true,apiMs});
  assert.equal(parallel.peak_requests,1);
  assert.ok(parallel.delivered>serial.delivered);
  // Slow requests retain at most one recent pending detection, not a FIFO.
  assert.ok(parallel.max_capture_to_result_ms<apiMs+16+1000/30+1e-6);
  if(apiMs===20){
    assert.ok(Math.abs(serial.median_target_interval_ms-2000/30)<1e-6);
    assert.ok(Math.abs(parallel.median_target_interval_ms-1000/30)<1e-6);
  }
  result.cases.push({serial,parallel});
}
console.log(JSON.stringify(result,null,2));
