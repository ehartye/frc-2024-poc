/** Observe the first retained midfield piece at the prescribed home-lane endpoint. */
import {readFileSync, writeFileSync} from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';

const plugin = process.argv[2];
if (!plugin) throw Error('Supply the Agent FRC scripts directory.');
const moduleAt = name => pathToFileURL(path.resolve(plugin,name)).href;
const {createFieldRehearsal} = await import(moduleAt('field-rehearsal.mjs'));
const {createMatchSeries} = await import(moduleAt('rehearsal-series-core.mjs'));
const read = relative => readFileSync(new URL(relative,import.meta.url));
const profileBytes = read('../../kickoff/crescendo-field.json');
const profile = JSON.parse(profileBytes);
const gauge = JSON.parse(read('../../cad/service-readiness/r11-reopen.json'));
const mast = JSON.parse(read('../../cad/service-readiness/reopen.json'));
const hash = value => createHash('sha256').update(value).digest('hex');
const revision = hash(profileBytes);
const criterion = {criterionId:'retained-note-at-home',focalRobot:'B3',windowScope:'whole TELEOP',
  criterion:'B3 reaches its home-lane endpoint with the distinct center NOTE in its own cargo.'};
const demand = {...criterion,requiredPieces:1,windowS:135,basis:'Blue 3 offers one distinct retained center NOTE at its home lane during TELEOP.'};
const episodes = [];

for (const [id,matchId,condition] of [
  ['clear','wing-opening','Partner on a separate lane'],
  ['queue','shared-center','Competing midfield requests with all six robots'],
  ['empty','wing-opening','Requested station starts empty'],
  ['jam','wing-opening','Focal robot stays in the pit with the displaced mast guide jammed'],
  ['service','wing-opening','Focal robot held in the pit by the failed disconnected pack gauge'],
]) {
  const field = structuredClone(profile);
  const station = field.rehearsal.stations.find(s=>s.id==='center-2');
  if (id==='empty') station.stock=0;
  const match = field.rehearsal.series.matches.find(m=>m.id===matchId);
  if (id==='service' || id==='jam') {
    const interference=id==='service'?gauge.highDressingControlMm3:mast.offsetCarrierGuideControlMm3;
    if (!(interference>0)) throw Error('Missing service no-go witness.');
    match.roster=match.roster.filter(r=>!(r.side==='blue' && r.slot===3));
  }
  const engine=createFieldRehearsal(field);
  const run=createMatchSeries(engine,[match]);
  run.advance(engine.phaseSeconds('AUTO'));
  const clock=engine.phaseSeconds('TELEOP');
  if (clock!==demand.windowS) throw Error('Field clock changed.');
  const home=field.rehearsal.programs.midfield.steps.at(-1).at;
  const events=[];
  let witness;
  for (let tick=1;tick<=Math.round(clock/engine.dt);tick++) {
    run.advance(engine.dt);
    const robot=run.sim.robots.find(r=>r.id==='B3');
    if (!robot || witness) continue;
    const acquired=run.sim.game.events.find(e=>e.phase==='TELEOP' && e.robot==='B3' &&
        e.operation==='acquire' && e.outcome==='acquired' && e.station===station.id);
    if (acquired && robot.visits===1 && Math.hypot(robot.p[0]-home[0],robot.p[1]-home[1])<0.01) {
      const piece=acquired.pieces[0];
      if (!run.sim.game.cargo.B3.some(article => article.id === piece)) throw Error('Piece missing at arrival.');
      witness=Number((tick*engine.dt).toFixed(3));
      events.push({timeS:acquired.timeS,kind:'acquire',piece},
        {timeS:witness,kind:'accept',piece,witness:'B3 retains this piece at the registered home-lane endpoint.'});
    }
  }
  if (!run.done) throw Error('Incomplete full window.');
  episodes.push({id,runId:matchId+'-'+id,revision,condition,...criterion,variation:{match:matchId,stock:station.stock,pitCondition:id},evidence:'field-rehearsal',
    windowS:[0,clock],initialPieces:station.stock?['center-2/1']:[],events});
}
writeFileSync(new URL('transport-runs.json',import.meta.url),JSON.stringify({demand,episodes},null,2)+'\n');
