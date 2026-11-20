/** Capture the four unchanged peers and phase-opening stock from the held-out roster. */
import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';
const dir=fileURLToPath(new URL('.',import.meta.url)), root=resolve(dir,'../..');
const lib=name=>import(pathToFileURL(resolve(process.argv[2],name)));
const {evidenceReader}=await lib('study-evidence.mjs');
const {deriveRoleDemand}=await lib('role-handoff.mjs');
const {compareRoles}=await lib('strategy-lab.mjs');
const {createFieldRehearsal}=await lib('field-rehearsal.mjs');
const {createMatchSeries}=await lib('rehearsal-series-core.mjs');
const read=evidenceReader(root);
const selection=JSON.parse(readFileSync(resolve(dir,'role-selection.json')));
const profile=read(selection.profile), study=read(selection.study);
const report=compareRoles(profile,study), demand=deriveRoleDemand(selection,read,report);
if(JSON.stringify(demand)!==JSON.stringify(JSON.parse(readFileSync(resolve(dir,'role-demand.json')))))throw Error('Held-out role changed');
const cell=report.cells.find(c=>c.role===selection.role&&c.context===selection.context);
const match=profile.rehearsal.series.matches.find(m=>m.id===cell.match);
const engine=createFieldRehearsal(profile), run=createMatchSeries(engine,[{...match,roster:cell.roster}]);
run.advance(engine.phaseSeconds('AUTO'));
const snapshots={}, shapes={};
const stock=Object.fromEntries(Object.entries(run.sim.game.stock).map(([k,a])=>[k,a.map(v=>v.id)]));
const initial=Object.fromEntries(run.sim.robots.map(r=>[r.id,r.p.slice()]));
for(let tick=0;tick<=Math.round(135/engine.dt);tick++){
  for(const r of run.sim.robots.filter(r=>!['B2','B3'].includes(r.id))){
    const trace=snapshots[r.id]??=[];
    const pos=r.p.map(n=>+n.toFixed(6));
    trace.push([tick,...pos]);
    shapes[r.id]=[r.length,r.width,r.bumper,r.heading];
  }
  if(tick<Math.round(135/engine.dt))run.advance(engine.dt);
}
// Keep velocity changes; linear interpolation recovers every original 50 ms location.
for(const [key,rows] of Object.entries(snapshots)){
  snapshots[key]=rows.filter((r,i)=>i===0||i===rows.length-1||[1,2].some(j=>
    Math.abs(rows[i+1][j]-2*r[j]+rows[i-1][j])>0.000003));
}
const result={match:cell.match,dt:engine.dt,initial,stock,shapes,traces:snapshots};
writeFileSync(resolve(dir,'peer-tracks.json'),JSON.stringify(result)+'\n');
