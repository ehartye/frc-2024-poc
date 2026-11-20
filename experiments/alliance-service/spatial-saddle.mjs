/** Replay the two suspended poses from the R8 season evidence using Agent FRC 0.39.0. */
import {createHash} from 'node:crypto';
import {readFileSync} from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';

const scripts = process.argv[2];
if (!scripts) throw Error('Supply the Agent FRC 0.39.0 scripts directory.');
const {reviewElevation} = await import(pathToFileURL(path.resolve(scripts, 'hardening.mjs')).href);
const sourceHashes = {};
function readSeason(name) {
  const data = readFileSync(new URL(name, import.meta.url));
  sourceHashes[name] = createHash('sha256').update(data).digest('hex');
  return JSON.parse(data);
}
const inventory = readSeason('load-inventory.json');
const screen = readSeason('../stage-climb/static-screen.json');
const hold = readSeason('../stage-climb/solo-trajectory.json');
if (screen.nativeSha256 !== hold.nativeSha256) throw Error('R8 archive mismatch');

// R8 chassis datum from stage-climb/analyze.py (HOOK). Vectors use +Z up.
const seatMm = [0.5, -30, 480];
const initial = screen.newNominalCenterMm.map((n, i) => (n - seatMm[i]) / 1000);
const q = hold.outcome.engaged.atFiveSeconds.quaternionXYZW;
function cross([a,b,c], [x,y,z]) { return [b*z-c*y, c*x-a*z, a*y-b*x]; }
const axis = q.slice(0,3);
const first = cross(axis, initial);
const second = cross(axis, first);
const settled = initial.map((v,i) => v + 2*(q[3]*first[i] + second[i]));

const model = inventory.model;
model.geometry = 'spatial-vertical-supports';
model.supports[0].xyM = [0,0];
delete model.supports[0].xM;
model.cases = [
  {id:'initial-contact', mode:'raise', asymmetric:true, cgXYM:initial.slice(0,2),
   basis:'Initial unrotated R8 nominal center relative to its free saddle.'},
  {id:'settled-hold', mode:'power-loss', cgXYM:settled.slice(0,2),
   basis:'The engaged R8 trajectory supplies the settled orientation at five seconds after cutoff.'},
].map(pose => ({...pose, massKg:screen.newNominalKg, accelZMps2:0, appliedMomentXYNm:[0,0]}));

const report = reviewElevation({
  schema:'agent-frc/elevation-study@1', inventory:{pointer:'/inventory'},
  mechanisms:[{id:inventory.inventory[0],source:{pointer:'/model'}}],
}, () => inventory);
const cases = report.mechanisms[0].cases.map(({id,totalVerticalN,balance,status}) => ({
  id, totalVerticalN, residualXYNm:balance.unbalancedMomentXYNm,
  equilibrium:balance.momentEquilibrium, loadPathStatus:status,
}));
if (cases[0].equilibrium !== 'unbalanced' || cases[1].equilibrium !== 'balanced' ||
    cases[1].loadPathStatus !== 'open') throw Error('Unexpected saddle verdict');
console.log(JSON.stringify({
  checkerCommit:'d508f4bcb5d02edfea8766f8839628310564192c', sourceHashes,
  r8ArchiveSha256:screen.nativeSha256, seatMm, nominalMassKg:screen.newNominalKg,
  initialCgRelativeToSeatM:initial, settledOrientationXYZW:q, settledCgRelativeToSeatM:settled,
  cases, scope:'Static rotational equilibrium at two modeled R8 poses. The R13 ground-supported selection retires this mast. Actual whole-robot capacity remains unknown.',
},null,2));
