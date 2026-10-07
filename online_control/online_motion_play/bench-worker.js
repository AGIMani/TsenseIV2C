import {runBenchmark} from './bench-worker-core.js';
onmessage=async({data})=>{
  try{postMessage({result:await runBenchmark({...data,onProgress:p=>postMessage(p)})});}
  catch(error){postMessage({error:error.stack});}
};
