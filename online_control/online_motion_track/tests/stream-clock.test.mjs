import test from 'node:test';
import assert from 'node:assert/strict';
import {StreamClock} from '../static/stream-clock.js';

test('slow status replies cannot replace a recent low-latency clock measurement',()=>{
  const clock=new StreamClock();
  clock.update('s',1,999,1001);
  clock.update('s',1.3,1100,1302); // Waited 200 ms for server work.
  assert(Math.abs(clock.age(1.301,1303)-2)<1e-6);
  assert(Math.abs(clock.age(1.1,1303)-203)<1e-6); // A genuinely old sample is still old.
});

test('session changes reset the clock and nonfinite measurements are rejected',()=>{
  const clock=new StreamClock();
  clock.update('a',1,999,1001);
  clock.update('b',10,1999,2001);
  assert.equal(clock.age(10,2003),3);
  assert.throws(()=>clock.update('b',NaN,2000,2001));
  assert.throws(()=>clock.update('b',10,2000,1999));
});
