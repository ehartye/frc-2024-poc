import {readFileSync,writeFileSync} from 'node:fs';
import {join} from 'node:path';
import {simulate} from './physics.mjs';
import {bench} from './bench.mjs';
const d=JSON.parse(readFileSync(join(process.argv[2],'clocks.json')));
const summary={};
for(const [caseName,y] of [['clear',3],['slow-receiver',-6]]){
  const p=d[caseName].plans[1];const selected={};
  for(const [name,options] of Object.entries({nominal:{},halfStep:{steps:100},fine:{grid:192,steps:100},
    finer:{grid:384,steps:100},
    wideSoft:{widthError:.5,stiffness:.08},narrowStiff:{widthError:-.5,stiffness:.18},
    slick:{friction:.02},carpetHigh:{carpet:.55},heightLow:{heightError:-.5}})){
    const r=simulate(p.frames,[p.sourceMm[0],p.sourceMm[1]+y],options),carry=r.stats.carry;
    selected[name]={minFloorMm:carry.minBottom,maxTopMm:r.maxTop,minSideN:carry.minSide,
      coreGapMm:r.coreGap,maxSidePairN:Math.max(...Object.values(r.stats).map(s=>s.maxRail)),
      readyS:r.lastContact,padErrorMm:+Math.hypot(p.padMm[0]-r.final[0],p.padMm[1]-r.final[1]).toFixed(3),
      endSpeedMmS:r.endSpeed, peakLateralMm:carry.maxLateral,peakAxialErrorMm:carry.maxXDeviation};
  }
  summary[caseName]=selected;
}
const frames=bench(), views={};
for(const e of [-6,-3,0,3,6,12,20]){
  const r=simulate(frames,[741.812,e]);
  views[e]={trace:r.trace.map(([time,x,y,z,bottom,rail,floor,tilt],i)=>
    [time,+(x-frames[i][1]).toFixed(2),+(y-frames[i][2]).toFixed(2),z,frames[i][3],bottom,rail,floor,frames[i][4]]),
    endSpeed:r.endSpeed, maxDepth:r.maxDepth};
}
writeFileSync(new URL('sensitivity.json',import.meta.url),JSON.stringify(summary,null,2)+'\n');
writeFileSync(join(process.argv[2],'section-traces.json'),JSON.stringify(views)+'\n');
