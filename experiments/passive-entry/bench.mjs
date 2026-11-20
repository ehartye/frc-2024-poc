import {design,simulate} from './physics.mjs';
import {writeFileSync} from 'node:fs';
const seat=design.seatXmm,outer=741.812,advance=outer-seat;
export function bench(){
  const frames=[];let from=[0,0,0],t=0;
  function segment(steps,to,phase){
    for(let i=0;i<steps;i++){
      const u=i/steps,v=u*u*(3-2*u);
      frames.push([Number((t*.05).toFixed(2)),...to.map((n,k)=>from[k]+v*(n-from[k])),phase]);t++;
    }from=to;
  }
  segment(12,[0,0,0],'rest');segment(32,[advance,0,0],'entry');segment(10,from,'seat');
  segment(16,[advance,0,design.liftMm],'lift');segment(8,from,'hold');
  segment(36,[advance+350,0,design.liftMm],'carry');segment(40,[advance+350,420,design.liftMm],'carry');
  segment(16,[advance+350,420,0],'lower');segment(10,from,'floor');
  segment(32,[350,420,0],'withdraw');segment(20,from,'settle');
  frames.push([t*.05,...from,'settle']);return frames;
}
if(process.argv[1]?.endsWith('bench.mjs')){
  const frames=bench(), results={};
  for(const e of [-6,-3,0,3,6]){
    const r=simulate(frames,[outer,e]);
    results[e]={...r,trace:undefined};
  }writeFileSync(new URL('bench.json',import.meta.url),JSON.stringify(results,null,2)+'\n');
}
