import fs from 'node:fs';import path from 'node:path';import {fileURLToPath} from 'node:url';
import {loadScoreObservations} from 'file:///%USERPROFILE%/repos/agent-frc/scripts/score-observations.mjs';
import {createFieldRehearsal} from 'file:///%USERPROFILE%/repos/agent-frc/scripts/field-rehearsal.mjs';
import {createMatchSeries} from 'file:///%USERPROFILE%/repos/agent-frc/scripts/rehearsal-series-core.mjs';
const src='R:/kickoff/physical-match.json';
const p=await loadScoreObservations(JSON.parse(fs.readFileSync(src)),src);const eng=createFieldRehearsal(p);
const series=createMatchSeries(eng,[p.rehearsal.series.matches[0]]);series.advance(15+111);
const frames=[];for(let i=0;i<=480;i++){const s=series.sim;frames.push({t:i*.05,actors:s.robots.map(r=>({id:r.id,p:r.p.slice(),heading:r.heading,length:r.length,width:r.width,bumper:r.bumper}))});if(i<480)series.advance(.05)}
const dest=path.join(path.dirname(fileURLToPath(import.meta.url)),'park-inputs.json');
fs.writeFileSync(dest,JSON.stringify({source:src,id:p.rehearsal.series.matches[0].id,phase:'TELEOP 111..135 s',dt:.05,frames,obstacles:p.rehearsal.obstacles,targets:p.rehearsal.endgame.targets}));
console.log(frames[0],frames.at(-1))

