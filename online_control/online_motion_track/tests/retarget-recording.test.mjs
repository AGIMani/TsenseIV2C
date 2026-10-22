import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const app=readFileSync(new URL('../static/app.js',import.meta.url),'utf8');
// Execute the production response-recording block before its DOM rendering.
const block=app.slice(app.indexOf('function applyFrameUpdate'),app.indexOf('    // Mode switches may race'))+'}';
function record(hand,update){
  const rows=[];
  const apply=vm.runInNewContext(block+';applyFrameUpdate;',{
    recorder:{record:(type,data)=>rows.push({type,...data})},$:()=>({value:'left'}),
    isV2:()=>false,isA30:()=>false,isA32Model:model=>['TsenseI_v3_2','TsenseI_v3_2_withid'].includes(model),performance:{now:()=>120},
  });
  apply({frameId:12,hand,expected:'Left',timestamp_ms:100,inferredAt:105,requestStartedAt:110},update);
  return rows;
}
for(const [hand,reason] of [[null,'no_hand'],[{label:'Right'},'wrong_hand'],[{label:'Left'},'solve_rejected']]){
  test(`records ${reason} instead of silently dropping the response`,()=>{
    const rows=record(hand,{tracking:false,message:'held',retarget_joint_angles_rad:null});
    assert.equal(rows.length,1);assert.equal(rows[0].type,'retarget_skipped');
    assert.equal(rows[0].reason,reason);assert.equal(rows[0].frame_id,12);
    assert.equal(rows[0].message,'held');assert.equal(rows[0].hand_side,'left');
  });
}
test('valid tracking records raw solver angles and collision metadata without redundant target angles',()=>{
  const rows=record({label:'Left'},{tracking:true,retarget_joint_angles_rad:{q:.1},
    collision_safe_target_joint_angles_rad:{q:.2},pose:{q:.3},solve_ms:10});
  assert.deepEqual(rows.map(r=>r.type),['retarget','control_target']);
  assert.equal(rows[0].joints.q,.1);assert.equal(rows[1].joints,undefined);
});
test('Mink and direct angles are recorded only when explicitly returned by their executed stage',()=>{
  const rows=record({label:'Left'},{tracking:true,retarget_joint_angles_rad:{q:.1},
    mink_joint_angles_rad:{q:.2},direct_joint_angles_rad:{q:.3}});
  assert.deepEqual(rows.map(r=>r.type),['retarget','mink','direct_command','control_target']);
  assert.equal(rows[1].joints.q,.2);assert.equal(rows[2].joints.q,.3);
});
test('missing raw output is distinguished from a rejected solve',()=>{
  const rows=record({label:'Left'},{tracking:true,pose:{q:.3}});
  assert.equal(rows[0].reason,'missing_retarget');assert.equal(rows[0].tracking,true);
});

test('completed solve is preserved when downstream collision/control rejects it',()=>{
  const rows=record({label:'Left'},{tracking:false,retarget_joint_angles_rad:{q:.1},pose:{q:0},message:'collision rejected'});
  assert.deepEqual(rows.map(r=>r.type),['retarget','control_target_skipped']);
  assert.equal(rows[0].joints.q,.1);assert.equal(rows[0].control_accepted,false);
  assert.equal(rows[1].message,'collision rejected');
});

for(const pose of [{},{q:null},{q:NaN}])test(`invalid solve ${JSON.stringify(pose)} is explicitly marked`,()=>{
  const rows=record({label:'Left'},{tracking:true,retarget_joint_angles_rad:pose});
  assert.deepEqual(rows.map(r=>r.type),['retarget_skipped']);assert.equal(rows[0].reason,'invalid_retarget');
});
