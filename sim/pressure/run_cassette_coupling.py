"""Fresh bounded transverse mechanical fixtures, not a checkpoint handoff."""
import pathlib,sys,json,hashlib,time,math
from dataclasses import asdict,replace
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/pressure'));sys.path.insert(0,str(R/'cad/assembly'))
import cassette_coupling as c
import cassette_cam_dynamics as mechanical
FILES=['sim/pressure/'+x for x in ['cassette_coupling.py','run_cassette_coupling.py','finite_area.py','finite_transport.py','fast_geometry.py']]+['cad/assembly/'+x for x in ['intake_components.json','belt_contact_geometry.py','cassette_cam_dynamics.py','powered_cam_contact.py','cassette_spring_law.py','powered-cam-interface.json']]+['tests/test_cassette_pressure_coupling.py']
sha=lambda f:hashlib.sha256((R/f).read_bytes()).hexdigest()
def inventory():
 pins={f:sha(f) for f in FILES}
 for f,h in json.loads((R/'cad/assembly/powered-cam-interface.json').read_text())['geometrySourcePins'].items():
  if sha(f)!=h:raise ValueError('upstream source changed '+f)
  pins[f]=h
 return pins

def trajectory(s,p,dt,duration,torque):
 cfg=c.config();initial=s;start=c.ports(s,p,cfg,torque);rows=[];maximum=0.
 for i in range(round(duration/dt)):
  s=c.step(s,p,cfg,dt,torque);r=c.ports(s,p,cfg,torque)
  residual=r['energyJ']-start['energyJ']-(s.actuator_work-initial.actuator_work)+(s.cam_heat-initial.cam_heat)
  maximum=max(maximum,abs(residual));rows.append(dict(state=asdict(s),ports=r,residualJ=residual))
 return dict(initial=asdict(initial),initialPorts=start,parameters=asdict(p),dt=dt,duration=duration,torque=torque,rows=rows,final=asdict(s),maxResidualJ=maximum)

def run(output):
 if output.exists():raise ValueError('fresh output required')
 pins=inventory();cfg=c.config();p=c.Parameters();mp=mechanical.Parameters(p.cassette_mass,p.cam_inertia,p.cam_stiffness,p.cam_damping,p.torque_limit)
 m,hold=mechanical.equilibrium(.2,mp);s=c.State(0.,0.,m.u,0.,m.theta,0.);rows={};started=time.perf_counter()
 for name,params,torque,dt in [('holding_input',p,hold,.0005),('holding_input_half',p,hold,.00025),('torque_removed',p,0.,.0005),('zero_spring_preload',replace(p,spring_free_length=.007),0.,.0005),('zero_spring_preload_half',replace(p,spring_free_length=.007),0.,.00025)]:
  rows[name]=trajectory(s,params,dt,.002,torque);print(name,rows[name]['final'],rows[name]['maxResidualJ'],flush=True)
 grids=[]
 for n,phase in [(512,0.),(1024,0.),(1024,math.pi/(2*1024))]:
  q=replace(p,resolution=n,grid_phase_rad=phase);grids.append(dict(resolution=n,phase=phase,result=c.ports(s,q,cfg,hold)))
 if inventory()!=pins:raise ValueError('source drift')
 result=dict(schema='transverse-note-cassette-cam/1',inputs=pins,acceptance=False,rows=rows,gridControls=grids,runtimeSeconds=time.perf_counter()-started,environment=dict(pythonVersion=sys.version.split()[0],numpyVersion=c.np.__version__),limits=['Transverse NOTE y is dynamic; X/Z and orientation are fixed ideal constraints with zero velocity/work, not a free 6DOF body.','Only positive-side cassette exists in this fixture; negative-side geometry is not queried or adopted.','Fixed cradle; reaction forces/moments are reported but there is no moving-base dynamics.','Pressure is recomputed every RK4 stage and endpoint using the area law, never spring force as prescribed NOTE normal.','One-sided normal-only coupling: mu=0 and fixed belt rotors. No feed, floor, gravity, acquisition, or score simulation.','Initialized with stored spring/contact energy as a declared isolated fixture, not an actual ground-acquisition handoff.','Holding_input applies no-NOTE nominal mechanical equilibrium torque, not an ideal cam-angle constraint or guarantee of equilibrium with NOTE.','Mass/inertia, pressure stiffness, cam compliance/damping and mechanical torque are explicit unmeasured hypotheses; no servo electrical or thermal model.','Grid/phase controls and RK4 residual retained; finite samples and small time refinement are not a full spatial/time convergence certificate.','Resting stop reaction can balance force with zero work; moving hard-stop impact remains UNKNOWN and rejects atomically.'])
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
