"""Fresh source-bound drivetrain + shared supply experiments, not a full robot cycle."""
import json,pathlib,sys,hashlib,copy,math,argparse
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(pathlib.Path(__file__).parent),str(R/'sim/integrated'),str(R/'sim/component-launcher')]
from plant import DrivePlant
from shared_supply import solve_supply,SupplyDomainError
from power_state import PowerState
FILES=['sim/drivetrain/plant.py','sim/drivetrain/config.json','sim/drivetrain/run.py','sim/integrated/shared_supply.py','sim/component-launcher/power_state.py','cad/drivetrain/candidate.json']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def validate(c):
 d=json.loads((R/'cad/drivetrain/candidate.json').read_text())
 if c['geometrySourceSha256']!=sha(R/'cad/drivetrain/candidate.json'):raise ValueError('source geometry pin')
 if c['trackM']!=d['centerTrackMm']/1000 or c['gearRatio']!=d['gearRatio'] or d['motorCount']!=4 or 'CIM' not in d['motorModel']:raise ValueError('source motor/track/gearing')
 wheels=[v for v in d['parts'] if v['role']=='wheel'];stations=[[[v['worldCenterMm'][0]/1000,v['worldCenterMm'][1]/1000] for v in sorted([x for x in wheels if x['worldCenterMm'][1]*side>0],key=lambda x:x['worldCenterMm'][0])] for side in [1,-1]]
 if c['contactStationsM']!=stations or c['wheelRadiusM']!=wheels[0]['sizeMm'][2]/2000:raise ValueError('source contact stations/wheel radius')

def run_case(c,name,dt=.001,duration=.3):
 c=copy.deepcopy(c)
 if name=='mass30':c['massKg']=30.
 if name=='mass60':c['massKg']=60.
 if name=='center-wheels-only':c['loadFractions']=[0.,1.,0.]
 p=DrivePlant(c);normal=[c['massKg']*9.80665/2]*2;policy=PowerState();last_bus=12.;trace=[];global_energy=dict(sourceWork=0.,sourceHeat=0.,syntheticMechanismWork=0.);peak_res=0.;minimum_bus=12.;status='HORIZON';row=None
 for i in range(round(duration/dt)):
  t=p.state['t'];source=6.5 if name=='brownout' and t>=.15 else 12.
  control=policy.update(t,last_bus,True,rearm=False);demands=(-.4,.4) if name in ['spin','center-wheels-only'] else (.4,.4)
  modes=('coast','coast') if not control['outputsEnabled'] or name=='coast' and t>=.15 else ('brake','brake') if name=='brake' and t>=.15 else ('drive','drive')
  ext=(-20.,0.) if name=='external-reaction' and t>=.1 else (0.,0.)
  drive_port=p.power_port(dt,demands,normal,modes=modes,external_force_world=ext)
  ports={'drive':drive_port}
  if name!='drive-only':ports['synthetic-mechanism']=lambda v:{'busPowerW':v*v/1.5}
  try:bus=solve_supply(ports,source,.02)
  except SupplyDomainError as e:status='UNKNOWN_SUPPLY: '+str(e);break
  if bus['ports']['drive']['infeasibleCurrentLimitVoltage']:status='UNSUPPORTED_CURRENT_LIMIT_VOLTAGE';break
  row=p.step(dt,demands,bus['busV'],normal,modes=modes,external_force_world=ext);last_bus=bus['busV'];minimum_bus=min(minimum_bus,last_bus)
  if abs(row['electrical']['busPowerW']-bus['ports']['drive']['busPowerW'])>1e-8:raise ValueError('port/step electrical mismatch')
  global_energy['sourceWork']+=bus['sourcePowerW']*dt;global_energy['sourceHeat']+=bus['sourceHeatW']*dt;global_energy['syntheticMechanismWork']+=bus['ports'].get('synthetic-mechanism',{}).get('busPowerW',0)*dt
  e=row['energyJ'];res=global_energy['sourceWork']-global_energy['sourceHeat']-global_energy['syntheticMechanismWork']+e['externalWork']-sum(e[k] for k in ['windingHeat','controllerBrakeHeat','motorDragHeat','gearDragHeat','tireSlipHeat'])-e['kinetic'];peak_res=max(peak_res,abs(res))
  row.update(busV=bus['busV'],busCurrentA=bus['busCurrentA'],sourceV=source,controller=control,modes=list(modes),demands=list(demands),globalEnergyJ=dict(global_energy,residual=res),externalForceWorldN=list(ext))
  if i%5==0:trace.append(row)
 if row is not None and (not trace or trace[-1]['state']['t']!=row['state']['t']):trace.append(row)
 return dict(name=name,status=status,config=c,stepS=dt,trace=trace,final=row,minimumBusV=minimum_bus,maxAbsGlobalResidualJ=peak_res,acceptance=False)

def run(output):
 if output.exists():raise ValueError('fresh output required')
 pins={f:sha(R/f) for f in FILES};c=json.loads((R/'sim/drivetrain/config.json').read_text());validate(c);cases=[]
 for name in ['drive-only','shared-load','spin','coast','brake','brownout','external-reaction','mass30','mass60','center-wheels-only']:
  case=run_case(c,name);cases.append(case);print(name,case['status'],None if case['final'] is None else case['final']['state']['x'],None if case['final'] is None else case['final']['state']['yaw_rate'],case['minimumBusV'],case['maxAbsGlobalResidualJ'],flush=True)
 case=run_case(c,'shared-load',dt=.0005);case['name']='shared-load-half';cases.append(case)
 if any(sha(R/f)!=v for f,v in pins.items()):raise ValueError('inputs changed')
 k=c['motorK'];res=c['motorResistanceOhm'];drag=c['motorViscousDragNmPerRadps'];predicted=k*12/(k*k+res*drag)*60/(2*math.pi)
 result=dict(schema='four-cim-shared-drive-experiment/1',inputs=pins,cases=cases,motorModelDiscrepancy=dict(predictedFreeRpm=predicted,sourceFreeRpm=c['source']['reportedFreeRpm'],predictedLockedRotorCurrentA=12/res,sourceStallCurrentA=c['source']['reportedStallCurrentA']),acceptance=False,limits=c['assumptions']+['Synthetic mechanism is a 1.5ohm electrical load, not an integrated acquisition/shooter.','Brownout open-circuit supply drop is injected; battery SOC/transient chemistry is not simulated.','No field boundaries or collision solver inside this physical plant; these belong to the integrated environment.'])
 output.parent.mkdir(parents=True,exist_ok=True);temporary=output.with_suffix(output.suffix+'.tmp');temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temporary.replace(output);return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);run(a.parse_args().output)
