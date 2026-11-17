"""Source-bound, short two-sided transverse dynamics; no acquisition handoff."""
import pathlib,sys,json,hashlib,time,math
from dataclasses import asdict,replace
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/pressure'))
import two_cassette as c
import run_cassette_coupling as baseline
import cassette_cam_dynamics as mechanical
FILES=['sim/pressure/two_cassette.py','sim/pressure/run_two_cassette.py','tests/test_two_cassette.py','sim/pressure/run_cassette_coupling.py']
def inventory():
 pins=baseline.inventory()
 for path in FILES:pins[path]=hashlib.sha256((R/path).read_bytes()).hexdigest()
 return pins

def trajectory(s,p,dt,torques):
 cfg=c.one.config();initial=s;rows=[];maximum=0.
 try:start=c.ports(s,p,cfg,torques)
 except ValueError as error:return dict(outcome='UNKNOWN_INITIAL_DOMAIN',reason=str(error),initial=asdict(s),parameters=asdict(p),dt=dt,duration=.002,torques=torques,rows=[])
 for _ in range(round(.002/dt)):
  before=s
  try:s=c.step(s,p,cfg,dt,torques);r=c.ports(s,p,cfg,torques)
  except ValueError as error:return dict(outcome='UNKNOWN_TRIAL_DOMAIN',reason=str(error),initial=asdict(initial),initialPorts=start,parameters=asdict(p),dt=dt,duration=.002,torques=torques,rows=rows,final=asdict(before),maxResidualJ=maximum)
  residual=r['energyJ']-start['energyJ']-(s.actuator_work-initial.actuator_work)+s.cam_heat-initial.cam_heat
  maximum=max(maximum,abs(residual));rows.append(dict(state=asdict(s),ports=r,residualJ=residual))
 return dict(outcome='HORIZON',initial=asdict(initial),initialPorts=start,parameters=asdict(p),dt=dt,duration=.002,torques=torques,rows=rows,final=asdict(s),maxResidualJ=maximum)
def run(output):
 if output.exists():raise ValueError('fresh output required')
 pins=inventory();started=time.perf_counter();p=c.Parameters();one=p.positive
 m,hold=mechanical.equilibrium(.2,mechanical.Parameters(one.cassette_mass,one.cam_inertia,one.cam_stiffness,one.cam_damping,one.torque_limit))
 s=c.State(0.,0.,m.u,0.,m.theta,0.,m.u,0.,m.theta,0.)
 no=replace(p,positive=replace(one,spring_free_length=.007),negative=replace(one,spring_free_length=.007))
 cases=[('symmetric_holding',s,p,.0005,(hold,hold)),('symmetric_half',s,p,.00025,(hold,hold)),('torque_removed',s,p,.0005,(0.,0.)),('zero_preload',s,no,.0005,(0.,0.)),('zero_preload_half',s,no,.00025,(0.,0.)),('offset_control',replace(s,y=.0001),p,.0005,(hold,hold)),('asymmetric_preload',s,replace(p,negative=replace(one,spring_free_length=.007)),.0005,(hold,0.)),('asymmetric_preload_half',s,replace(p,negative=replace(one,spring_free_length=.007)),.00025,(hold,0.))]
 rows={}
 for name,state,params,dt,torques in cases:
  rows[name]=trajectory(state,params,dt,torques);print(name,rows[name]['outcome'],rows[name].get('final',rows[name]['initial']),rows[name].get('maxResidualJ'),flush=True)
 grids=[]
 for n,phase in [(512,0.),(1024,0.),(1024,math.pi/(2*1024))]:
  one=replace(p.positive,resolution=n,grid_phase_rad=phase);params=c.Parameters(one,one);grids.append(dict(resolution=n,phase=phase,ports=c.ports(s,params,c.one.config(),(hold,hold))))
 if inventory()!=pins:raise ValueError('source drift')
 result=dict(schema='two-cassette-transverse/1',inputs=pins,cases=rows,gridControls=grids,runtimeSeconds=time.perf_counter()-started,acceptance=False,limits=['Negative-side mechanism is a hypothetical reflection of the positive-side force geometry, not adopted CAD or clearance/chain routing proof.','Only NOTE y, two cassette travels and two cam angles are dynamic; fixed X/Z/orientation/cradle constraints have zero velocity/work but nonzero reactions.','No feed friction, gravity, floor, rotor/electrical model or calibrated actuator. External mechanical torque only.','Initial stored spring/contact/cam energy is declared; not an acquisition checkpoint handoff.','Pressure computed at every stage; no spring-force normal budget or active-area renormalization.','No latch, capture, retention, lift or structural qualification. Finite quadrature/time controls are not a convergence certificate.'])
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
