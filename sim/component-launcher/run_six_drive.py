"""Source-bound powered-transfer experiment, not a complete robot or scoring claim."""
import pathlib,json,hashlib,math,copy,argparse
from transfer_plant import SixDrivePlant
from power_state import PowerState
R=pathlib.Path(__file__).resolve().parents[2]
H=pathlib.Path(__file__).parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def validate_reports(transfer,native):
 t=json.loads(transfer.read_text());n=json.loads(native.read_text())
 if t.get('schema')!='frc-current-note-transfer-screen/2' or t.get('assemblyPartCount')!=711 or t.get('invalidOrNonSolidInputs')!=[]:raise ValueError('Expected connected711 material report')
 if [x['advanceMm'] for x in t['rows']]!=list(range(0,481,10)):raise ValueError('Full transfer domain required')
 import sys
 sys.path.insert(0,str(R/'cad/assembly'))
 import build_connected_candidate as c
 expected=set(c.input_pins())|{'sim/component-launcher/transfer_screen.py','sim/component-launcher/extract_geometry.py'}
 if set(t['inputs'])!=expected:raise ValueError('Complete transfer dependency inventory required')
 for report in [t,n]:
  for f,h in report['inputs'].items():
   p=(R/f).resolve()
   if not p.is_relative_to(R.resolve()) or sha(p)!=h:raise ValueError('Stale or outside input:'+f)
 if n.get('transferSha256')!=sha(transfer):raise ValueError('Native/transfer lineage mismatch')
 expected_native=set(c.input_pins())|{'cad/components/a2-wheel-cartridge.json','sim/component-launcher/check_connected_wheel_profile.py','sim/component-launcher/extract_geometry.py'}
 if set(n['inputs'])!=expected_native:raise ValueError('Complete native dependency inventory required')
 source=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['wheel']
 if n['source']!=source:raise ValueError('Native wheel source identity')
 affected=[]
 for row in t['rows']:
  hits=[x for x in row['hits'] if x['contactGroup']=='unintended']
  if any('original wheel shell representation' not in x['name'] for x in hits):raise ValueError('Unresolved structural contact')
  if hits:affected.append(row['advanceMm'])
 if [r['advanceMm'] for r in n['rows']]!=affected:raise ValueError('Every shell-hit pose needs native witness')
 for row in n['rows']:
  values=row['nativeBareWheelOverlapMm3']
  if set(values)!= {'-1/-152.4','-1/152.4','1/-152.4','1/152.4'}:raise ValueError('Four native wheel witnesses required')
  if any(type(v) not in [int,float] or not math.isfinite(v) or v<0 or v>1e-5 for v in values.values()):raise ValueError('Native bare-wheel contact is not tread-only')
 for row in t['rows']:
  for group in ['pinch','launcher']:
   total=sum(x['overlapMm3'] for x in row['hits'] if x['contactGroup']==group)
   if not math.isclose(total,row[group+'OverlapMm3'],rel_tol=1e-12,abs_tol=1e-10):raise ValueError('Contact accounting mismatch')
 return t,n

def simulate(config,rows,scenario='nominal'):
 if scenario not in ['nominal','zero-pinch','low-friction','sensor-loss','load-step','power-off','low-preload','nine-volt','half-step']:raise ValueError('Scenario')
 c=copy.deepcopy(config)
 if scenario=='low-friction':c['friction']=c['pinchFriction']=.3
 if scenario=='low-preload':c['normalEachN']=5.;c['pinchNormalEachN']=2.
 if scenario=='nine-volt':c['busOpenCircuitV']=9.
 if scenario=='half-step':c['stepS']/=2
 p=SixDrivePlant(c,rows);policy=PowerState();last_bus=c['busOpenCircuitV'];released=None;latched=False;trace=[];events=[];previous=None;outcome='HORIZON';tick=0;maxres=0.;minbus=c['busOpenCircuitV'];peakcurrent=0.;lastcontact=None
 while p.t<c['maxTimeS']:
  since=None if released is None else p.t-released
  fresh=not(scenario=='sensor-loss' and since is not None and .03<=since<.08)
  load=300. if scenario=='load-step' and since is not None and .03<=since<.08 else c.get('otherRobotCurrentA',0.)
  state=policy.update(p.t,last_bus,fresh,rearm=released is None)
  if released is not None and not state['outputsEnabled']:latched=True
  enabled=state['outputsEnabled'] and not latched
  if scenario=='power-off' and since is not None and since>=.03:enabled=False
  release=released is None and enabled and min(p.w[2:])>=c['readyRpm']*2*math.pi/60 and (scenario=='zero-pinch' or min(p.w[:2])*c['pinchRadiusM']>=c['pinchReadySurfaceMps'])
  if release:released=p.t
  pinch=c['pinchDuty'] if enabled and scenario!='zero-pinch' else 0.
  duties=[pinch]*2+[c['duty'] if enabled else 0.]*4
  row=p.step(duties,release=release,other_current=load);last_bus=row['busV'];row['controller']=state;row['duties']=duties
  maxres=max(maxres,abs(row['energyJ']['residual']));minbus=min(minbus,row['busV']);peakcurrent=max(peakcurrent,row['busCurrentA'])
  if released is not None and any(abs(f)>1e-10 for f in row['forcesN']):lastcontact=dict(timeS=p.t,advanceM=p.x,speedMps=p.v)
  key=(state['stage'],enabled,p.held)
  if key!=previous:events.append(dict(timeS=p.t,stage=state['stage'],enabled=enabled,held=p.held,advanceM=p.x,speedMps=p.v));previous=key
  if tick%100==0:trace.append(row)
  if released is not None and p.v<0:outcome='REVERSED';break
  if p.x>=rows[-1]['advanceMm']/1000:outcome='PASSED_TABLE_BOUNDARY';break
  tick+=1
 if not trace or trace[-1]['timeS']!=row['timeS']:trace.append(row)
 return dict(scenario=scenario,outcome=outcome,releaseTimeS=released,events=events,trace=trace,final=row,maxAbsEnergyResidualJ=maxres,minimumBusV=minbus,peakBusCurrentA=peakcurrent,lastModeledDriveContact=lastcontact,acceptance=False)

def run(transfer,native,output):
 transfer=transfer.resolve();native=native.resolve();output=output.resolve()
 if output.exists():raise ValueError('Fresh output directory required')
 t,n=validate_reports(transfer,native)
 config=json.loads((H/'six-drive-config.json').read_text())
 intake=json.loads((R/'cad/assembly/intake_components.json').read_text());tread=json.loads((R/'cad/components/a2-transmission.json').read_text())['serviceInterfaces']['tread']
 derived={'pitchDegrees':intake['pitchDeg'],'radiusM':(tread['nativeSeatRadiusMm']+tread['thicknessMm'])/1000,'pinchRadiusM':intake['pinch']['pitch']*intake['pinch']['teeth']/(2000*math.pi)}
 if any(not math.isclose(config[k],v,rel_tol=0,abs_tol=1e-12) for k,v in derived.items()):raise ValueError('Plant/CAD geometry mismatch')
 files=[pathlib.Path(__file__),H/'transfer_plant.py',H/'power_state.py',H/'six-drive-config.json',transfer,native]
 pins={p.relative_to(R).as_posix():sha(p) for p in files}
 cases=[simulate(config,t['rows'],s) for s in ['nominal','zero-pinch','low-friction','sensor-loss','load-step','power-off','low-preload','nine-volt','half-step']]
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('Inputs changed')
 output.mkdir(parents=True)
 report=dict(schema='frc-six-drive-transfer-experiment/1',inputs=pins,cadInputs=t['inputs'],cases=cases,acceptance=False,limits=config['assumptions'])
 (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
 return report
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--transfer',type=pathlib.Path,required=True);a.add_argument('--native',type=pathlib.Path,required=True);a.add_argument('--output',type=pathlib.Path,required=True);v=a.parse_args();print([(r['scenario'],r['outcome']) for r in run(v.transfer,v.native,v.output)['cases']])
