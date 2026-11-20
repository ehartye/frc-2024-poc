import {readFileSync,writeFileSync} from 'node:fs';
import {join} from 'node:path';
import {simulate,design} from './physics.mjs';
const directory=process.argv[2], clocks=JSON.parse(readFileSync(join(directory,'clocks.json')));
const summary={}, offsets={}, cache=new Map();
const residual={clear:[-3,3],late:[3,-3],misaligned:[-6,6],'late-offset':[6,-6],'slow-receiver':[3,-6],directDispatch:[3,-6]};
for(const [name,record] of Object.entries(clocks)){
  offsets[name]={};const pieces={};
  for(const [i,p] of record.plans.entries()){
    const y=residual[name][i], key=JSON.stringify([p.frames,y]);
    if(!cache.has(key))cache.set(key,simulate(p.frames,[p.sourceMm[0],p.sourceMm[1]+y]));
    const r=cache.get(key), dx=r.final[0]-p.padMm[0],dy=r.final[1]-p.padMm[1];
    const datum=p.frames.at(-1)[0];
    const carry=r.stats.carry;
    const phases=Object.values(r.stats);
    const minX=Math.min(...phases.map(s=>s.minX)),maxX=Math.max(...phases.map(s=>s.maxX));
    const maxY=Math.max(...phases.map(s=>s.maxLateral));
    const floorAtRest=r.trace.at(-1)[6];
    const passed=floorAtRest>design.massKg*9.80665*.95 && minX>=405 && maxX<=752 && maxY<=6.5 && carry.minBottom>.25 && carry.minSide>.05 && r.coreGap>.2 && r.maxTop<57.0
      && Math.hypot(dx,dy)<76.2 && r.lastContact<datum-.25 && r.endSpeed<2;
    pieces[p.id]={qualified:passed,initialResidualMm:y,settledOffsetMm:[dx,dy].map(v=>+v.toFixed(3)),timeS:datum,
      floorAtRestN:floorAtRest,minRelativeXmm:minX,maxRelativeXmm:maxX,maxRelativeYmm:maxY,peakRailN:Math.max(...phases.map(s=>s.maxRail)), contactStartS:r.firstContact, contactEndS:r.lastContact,maxTopMm:r.maxTop, maxTiltDeg:r.maxTiltDeg,
      minCarryFloorMm:carry.minBottom,minSideLoadN:carry.minSide,coreGapMm:r.coreGap,
      maxDeformationMm:r.maxDepth,peakAxialDeviationMm:carry.maxXDeviation,maxCarryLateralMm:carry.maxLateral,
      maxEntryLateralMm:r.stats.entry.maxLateral,minCarryXmm:carry.minX, endSpeedMmS:r.endSpeed};
    offsets[name][p.id]=pieces[p.id].settledOffsetMm;
  }
  const schedule=record.schedule, accepted=[];
  for(const [id,piece] of Object.entries(pieces)){if(!piece.qualified)break;accepted.push(id);}
  const ids=xs=>xs.filter(id=>accepted.includes(id));
  summary[name]={pieces,accepted,received:ids(schedule.received),stored:ids(schedule.stored),plannedFinish:schedule.stop,
    carrierReset:accepted.length===2&&schedule.carrierReset,receiverReady:accepted.length===2&&schedule.receiverReady,minimumPlanGapIn:schedule.minimumPlanGapIn,
    receiverEvents:schedule.history.filter(e=>e.actor==='B2'&&accepted.includes(e.piece)),
    finalCustody:Object.fromEntries(Object.entries(schedule.custody).map(([id,owner])=>[id,accepted.includes(id)?owner:'unqualified']))};
}
writeFileSync(join(directory,'settled.json'),JSON.stringify(offsets)+'\n');
writeFileSync(new URL('field.json',import.meta.url),JSON.stringify(summary,null,2)+'\n');
