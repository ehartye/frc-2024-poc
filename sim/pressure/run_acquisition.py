"""Fresh original rest/no-contact approach under the new pressure law; no checkpoint swap."""
import pathlib,sys,json,hashlib,argparse,importlib.util,math,copy,time
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/acquisition'),str(R/'sim/integrated')]
from pressure_acquisition import PressureAcquisition
import run_free_planar,free_geometry,obstacle_gate
from free_planar import DomainExit
spec=importlib.util.spec_from_file_location('drive_validation',R/'sim/drivetrain/run.py');drive_source=importlib.util.module_from_spec(spec);spec.loader.exec_module(drive_source)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/pressure/'+n for n in ['finite_area.py','finite_transport.py','fast_geometry.py','pressure_acquisition.py','run_acquisition.py']]+['sim/integrated/'+n for n in ['coupled_acquisition.py','shared_supply.py','patch_registration.py']]+['tests/test_pressure_acquisition.py','tests/test_pressure_fast_geometry.py']
def run(output,dt=.001,horizon=.58,scenario='nominal'):
 if output.exists():raise ValueError('fresh output required')
 if dt not in (.001,.0005) or not 0<horizon<=1 or scenario not in ['nominal','no-pinch','command-off']:raise ValueError('bounded declared run')
 obstacles,floor,pins=run_free_planar.load(R/'sim/acquisition/free-obstacles.json');drive=json.loads((R/'sim/drivetrain/config.json').read_text());drive_source.validate(drive);note=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text());pins.update({f:sha(R/f) for f in FILES+drive_source.FILES})
 gate=obstacle_gate.make_gate(obstacles,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ']);geometry=free_geometry.make_geometry(cad,gate);initial=dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]);p=PressureAcquisition(drive,note,geometry,initial,cad,resolution=(1024,1024),k_area=1e7);loads=[drive['massKg']*9.80665/2]*2
 if p.initial_energy!=0 or p.prepared(p.drive.state,p.note,{})['potentialJ']!=0:raise ValueError('nonzero fresh energy')
 initial_state=dict(timeS=p.t,chassis=copy.deepcopy(p.drive.state),note=copy.deepcopy(p.note),energy=copy.deepcopy(p.energy),pressurePotentialJ=0.);trace=[];status='HORIZON';reason=None;first=None;timings=[];started=time.perf_counter()
 for i in range(round(horizon/dt)):
  dd=[.2,.2];pd=[0.,0.] if scenario=='no-pinch' else [.12,.12];modes=['drive','drive']
  if scenario=='command-off' and p.t>=.57:dd=[0.,0.];pd=[0.,0.];modes=['coast','coast']
  before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t));stepstart=time.perf_counter()
  try:row=p.step(dt,dd,pd,loads,modes=modes)
  except (DomainExit,ValueError) as e:
   if before!=(p.drive.state,p.note,p.energy,p.t):raise AssertionError('non-atomic rejection')
   status='UNKNOWN_DOMAIN';reason=str(e);break
  elapsed=time.perf_counter()-stepstart;timings.append(elapsed);row['commands']=dict(drive=dd,pinch=pd,modes=modes);row['contactTiming']='midpoint';row['contactSampleTimeS']=row['timeS']-dt/2;trace.append(row)
  if row['contacts'] and first is None:first=dict(acceptedTimeS=p.t,contactSampleTimeS=p.t-dt/2,relativeRobotXY=row['relativeRobotXY']);print('FIRST_CONTACT',first,flush=True)
  if i%100==0:print('STEP',i,'t',p.t,'relative',row['relativeRobotXY'],'stepSeconds',elapsed,'energyResidualJ',row['energyJ']['residual'],flush=True)
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 final=trace[-1] if trace else None
 result=dict(schema='fresh-pressure-acquisition/1',inputs=pins,scenario=scenario,stepS=dt,horizonS=horizon,pressureConfig=dict(kAreaNpm3=1e7,resolution=[1024,1024],uPositiveM=0.,normalDamping='none; conservative new pressure potential'),initialState=initial_state,firstContact=first,outcome=status,reason=reason,trace=trace,final=final,maxAbsEnergyResidualJ=max([abs(r['energyJ']['residual']) for r in trace],default=0.),runtimeSeconds=time.perf_counter()-started,stepRuntime=dict(count=len(timings),max=max(timings,default=0.),mean=sum(timings)/max(1,len(timings))),acceptance=False,limits=['Actual fresh original rest state, zero new pressure potential; no imported indexed checkpoint or hidden reference shift.','Same synchronous midpoint body/drive/rotors and single513-grid shared supply; scalar final preview/end gate precedes atomic commit.','New material-area penalty law and friction are uncalibrated; normal compression damping from the old point model is not carried over.','Planar flat ground constraint and prescribed tire-normal loads remain; nonplanar pressure wrench above1e-8 is rejected.','Full1024-square material grid, unchanged phase, conservative culling only.','Contacts/forces are midpoint samples; accepted states and pressureFeatures are end samples.','Known conservative obstacle candidates stop as UNKNOWN, not a verified collision or clearance statement.','No capture, lift, scoring, fullrobot clearance or realtime acceptance.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output);print(status,reason,'accepted',p.t,'runtime',result['runtimeSeconds'],'maxResidual',result['maxAbsEnergyResidualJ'],flush=True)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);a.add_argument('--step',type=float,default=.001);a.add_argument('--horizon',type=float,default=.58);a.add_argument('--scenario',default='nominal');v=a.parse_args();run(v.output,v.step,v.horizon,v.scenario)
