// Replay saved payloads through the SAME decoder as both online_control apps.
// Non-telemetry lines (including commands) and original payload_hex stay intact.
import {createReadStream, createWriteStream} from 'node:fs';
import {writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {createInterface} from 'node:readline';
import {once} from 'node:events';
import {telemetry,telemetryMetadata} from '../online_control/shared/a30-telemetry.js';

const [input,output]=process.argv.slice(2);
if(!input||!output||resolve(input)===resolve(output))throw Error('Usage: node tools/redecode_a30_log.mjs INPUT.jsonl NEW_OUTPUT.jsonl');
const target=createWriteStream(output,{flags:'wx'});
await once(target,'open');
const report={input:resolve(input),output:resolve(output),records:0,telemetry_frames:0,
  problematic_frames:0,issues_by_joint:{},examples:[],note:'按 A3_USB_Downlink.pyw 的 <5f2h> 复核。异常原始字节不能恢复为可信测量值，不补零、不推测其他固件格式。'};
try{
  for await(const line of createInterface({input:createReadStream(input),crlfDelay:Infinity})){
    if(!line.trim())continue;
    const row=JSON.parse(line);report.records++;
    let text=line;
    if(row.type==='session'&&row.metadata?.model_id==='TsenseI_v3_0'){
      row.metadata={...row.metadata,...telemetryMetadata(row.metadata.hand_side||'left')};text=JSON.stringify(row);
    }
    if((row.type||row.event)==='telemetry'&&typeof row.payload_hex==='string'&&/^(?:[0-9a-f]{888}|[0-9a-f]{968})$/i.test(row.payload_hex)){
      const decoded=telemetry(Uint8Array.from(Buffer.from(row.payload_hex,'hex')));
      report.telemetry_frames++;
      if(decoded.decode_issues.length){
        report.problematic_frames++;
        for(const issue of decoded.decode_issues){
          const key=issue.joint_suffix+':'+issue.field+':'+issue.reason;
          report.issues_by_joint[key]=(report.issues_by_joint[key]||0)+1;
        }
        if(report.examples.length<3)report.examples.push({t:row.t,telemetry_seq:decoded.telemetry_seq,issues:decoded.decode_issues});
      }
      text=JSON.stringify({...row,...decoded,redecoded_from_payload:true});
    }
    if(!target.write(text+'\n'))await once(target,'drain');
  }
  target.end();await once(target,'finish');
  await writeFile(output+'.report.json',JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({records:report.records,telemetry_frames:report.telemetry_frames,problematic_frames:report.problematic_frames,output:report.output}));
}catch(error){target.destroy();throw error;}
