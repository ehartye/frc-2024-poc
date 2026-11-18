"""Fresh corrected-geometry launcher slice; not a full acquisition/robot simulation."""
import pathlib,json,hashlib,math,importlib.util,csv,argparse,uuid
import plant
R=pathlib.Path(__file__).resolve().parents[2];HERE=pathlib.Path(__file__).parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def can_register(result):
 return result['outcome']=='passed contact-table boundary' and result['lastPositiveEngagementSample'] is not None and result['lastPositiveEngagementSample']['speedMps']>0

def registered(release,g):
 s=g['shooter'];angle=math.radians(s['releasePitchDeg']);along=g['wheelStationMm']/1000+release['xFromStationM'];p=s['pivot'];local=[p[0]+along*math.cos(angle),p[1],p[2]+along*math.sin(angle)];pose=s['shotPose'];yaw=math.radians(pose['worldHeadingDeg']);center=pose['worldRobotCenter'];speed=release['speedMps']
 return {'pieceXWorld':center[0]+math.cos(yaw)*local[0]-math.sin(yaw)*local[1],'pieceYWorld':center[1]+math.sin(yaw)*local[0]+math.cos(yaw)*local[1],'pieceZWorld':center[2]+local[2],'exitVx':speed*math.cos(angle)*math.cos(yaw),'exitVy':speed*math.cos(angle)*math.sin(yaw),'exitVz':speed*math.sin(angle),'bodyPitchRad':angle},local

def run(out):
 out=pathlib.Path(out)
 if out.exists():raise ValueError("Fresh output directory required")
 g=json.loads((HERE/'current-geometry.json').read_text());c=json.loads((HERE/'config.json').read_text())
 for path,h in g['inputs'].items():
  if sha(R/path)!=h:raise ValueError('Stale CAD/contact input:'+path)
 if not math.isclose(c['radiusM']*1000,g['wheelRadiusMm'],abs_tol=1e-9) or not math.isclose(c['startMm'],g['handoffFromStationMm'],abs_tol=1e-9):raise ValueError('Config/current geometry mismatch')
 files=['config.json','current-geometry.json','plant.py','power_state.py','stateful_brownout.py','run_current.py','UPSTREAM.json'];pins={f:sha(HERE/f) for f in files};spec=importlib.util.spec_from_file_location('aperture',R/'scripts/a2-exit-screen.py');ap=importlib.util.module_from_spec(spec);spec.loader.exec_module(ap);pins['scripts/a2-exit-screen.py']=sha(R/'scripts/a2-exit-screen.py')
 out.mkdir(parents=True,exist_ok=False);results=[]
 for name,overrides in [('nominal',{}),('slower handoff',{'feedEntrySpeedMps':.5}),('faster handoff',{'feedEntrySpeedMps':2.}),('low friction',{'friction':.3}),('zero traction',{'friction':0}),('9V plus load',{'busOpenCircuitV':9,'otherRobotCurrentA':60}),('half timestep',{'stepS':c['stepS']/2})]:
  result,trace=plant.simulate(dict(c,**overrides),g);entry={'name':name,'overrides':overrides,'plant':result};release=result['lastPositiveEngagementSample']
  if can_register(result):
   row,local=registered(release,g);entry.update(registeredRelease=row,releaseRobotM=local,thinApertureScreen=ap.screen(row,256,64))
  else:entry['releaseNotRegistered']='No established forward departure; last contact is not a release when the piece reverses'
  trace_name=name.replace(' ','-')+'.csv'
  with (out/trace_name).open('w',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(trace[0]));w.writeheader();w.writerows(trace)
  entry['trace']={'file':trace_name,'sha256':sha(out/trace_name)}
  results.append(entry);print(name,result['outcome'],release,flush=True)
 report={'schema':'frc-current-launcher-run/1','runId':str(uuid.uuid4()),'inputs':pins,'cadInputs':g['inputs'],'cases':results,'traceSha256':sha(out/'nominal.csv'),'wholeRobotSimulation':False,'scoreAccepted':False,'limits':g['limits']+c['assumptions']+['Previous shot pose remains an explicit provisional dock assumption; shooter translation is current, full field structures are absent','Base cases exclude stateful brownout: low-bus outcomes are conditional, not hardware operability','Last positive sampled engagement is not measured release; no fresh scoring success inferred']}
 for path,h in g['inputs'].items():assert sha(R/path)==h
 for path,h in pins.items():assert sha(R/path if path.startswith('scripts/') else HERE/path)==h
 (out/'report.json').write_bytes((json.dumps(report,indent=2,allow_nan=False)+'\n').encode())
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=pathlib.Path);args=parser.parse_args();run(args.output)
