"""Short native-wheel comparison; each evolution is at most0.1s."""
import copy,json,pathlib,hashlib,time,argparse
import mujoco
from native_drive import NativeDrive
HERE=pathlib.Path(__file__).resolve().parent
FILES=['native_drive.py','native_drive_run.py','test_native_drive.py']
def pins(root):
 p={f:HERE/f for f in FILES};p.update({'source/'+f:pathlib.Path(root)/f for f in ['sim/drivetrain/config.json','sim/drivetrain/plant.py','cad/drivetrain/candidate.json']});return {n:hashlib.sha256(v.read_bytes()).hexdigest() for n,v in p.items()}
def evolve(sim,demands=(0,0),modes=('coast','coast'),duration=.1):
 start=time.perf_counter();initial=sim.observe();rows=[]
 for i in range(round(duration/sim.m.opt.timestep)):
  e=sim.step(demands,modes)
  if i%20==0:rows.append(sim.observe())
 return dict(initial=initial,final=sim.observe(),samples=rows,lastElectrical=e,demands=list(demands),modes=list(modes),runtimeSeconds=time.perf_counter()-start)
def run(root,out):
 out=pathlib.Path(out)
 if out.exists():raise ValueError('fresh output required')
 before=pins(root);cases=[]
 for dt in [.00025,.000125]:
  base=NativeDrive(root,dt);settle=evolve(base);cases.append(dict(kind='gravity-settle',dt=dt,result=settle))
  drive=copy.deepcopy(base);r=evolve(drive,[.3,.3],('drive','drive'));cases.append(dict(kind='drive-from-settled',dt=dt,result=r))
  turn=copy.deepcopy(base);cases.append(dict(kind='turn-from-settled',dt=dt,result=evolve(turn,[-.2,.2],('drive','drive'))))
  for mode in ['coast','brake']:
   p=copy.deepcopy(drive);cases.append(dict(kind=mode+'-same-powered-checkpoint',dt=dt,result=evolve(p,[0,0],(mode,mode))))
 zero=NativeDrive(root,mu=0.);evolve(zero);cases.append(dict(kind='zero-friction-from-own-settle',dt=.00025,result=evolve(zero,[.3,.3],('drive','drive'))))
 suspended=NativeDrive(root);suspended.m.opt.gravity[:]=0;mujoco.mj_forward(suspended.m,suspended.d);suspended.initial_energy=sum(suspended.d.energy)
 cases.append(dict(kind='suspended-no-gravity-fresh-gap',dt=.00025,result=evolve(suspended,[.3,.3],('drive','drive'))))
 if before!=pins(root):raise ValueError('source changed')
 report=dict(schema='native-wheel-engine-audit/1',engine=mujoco.__version__,inputs=before,cases=cases,acceptance=False,limits=['XYZ/yaw chassis: roll/pitch constrained, not load-transfer qualified.','Six source cylinders preserve1.778mm center drop and25.4mm width; no tread compliance calibration.','0.5kg perwheel and42kg base are hypotheses; baseCoGheight176.2mm is hypothetical.','Two CIMs torque central master hinge per side, four soft1:1joint equalities approximate chain; drift reported.','Physical wheel spin plus armature sums to reviewed sideinertia once; not measured distribution.','Fixed12V, no battery or regen supply model. Native groundcontacts replace custom tireforces.','Constraint-work integral is a numerical diagnostic including softcontact/equality work, not measured dissipation or closed physical energyledger.','Initialgap1mm. Paired control forks copy exactsettled/poweredstate, never reset velocities.','No pickup/intake/fullrobotCAD/capture/structural acceptance.'])
 out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8');return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('output',type=pathlib.Path);p.add_argument('--source-root',type=pathlib.Path,default=HERE.parents[1]);a=p.parse_args();r=run(a.source_root,a.output);print('CASES',len(r['cases']))
