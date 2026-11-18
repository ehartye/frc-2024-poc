"""Check actual continuation, clocked inputs and three-level trajectory evidence."""
import argparse,hashlib,json,math,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def verify(a):
 load=lambda p:json.loads(p.read_bytes());c=load(a.contract);s=load(a.source)
 names=['native-pressure-lift-clocked-'+n for n in ['half','quarter','eighth']]+['native-pressure-lift-100ms-'+n for n in ['coast','coast-quarter']];runs={};summaries=[]
 for name in names:
  path=a.packet/name/'receipt.json';r=load(path);runs[name]=r
  for k,v in s['final'].items():assert r['initial'][k]==v if k!='ledger' else all(r['initial']['ledger'][j]==x for j,x in v.items())
  assert r['inputs'][a.source.name]==sha(a.source) and r['inputs'][a.contract.name]==sha(a.contract)
  producer='run_native_pressure_lift_clocked.py' if 'clocked' in name else 'run_native_pressure_lift.py'
  for p in ['native_pressure_lift.py',producer]:assert r['inputs'][p]==sha(a.packet/p)
  assert abs(r['initialMechanicalDifferenceJ'])<1e-12
  for i,row in enumerate(r['trace']):
   assert abs(row['timeS']-r['initial']['timeS']-(i+1)*r['stepS'])<1e-9
   for k in ['pressureWorkResidualW','sourcePowerResidualW','portMidpointPowerDifferenceW']:assert abs(row[k])<1e-8
  if 'clocked' in name:
   assert len(r['events'])==80
   for i,e in enumerate(r['events']):
    assert abs(e['timeS']-r['initial']['timeS']-i*.01)<1e-9
    t=i*.01;z=.0254+(.05*t*t if i<20 else .002+.02*(t-.2) if i<50 else .008);v=.1*t if i<20 else .02 if i<50 else 0
    assert abs(e['command']['requestedPivotZM']-z)<1e-15 and abs(e['command']['requestedPivotVelocityMps']-v)<1e-15
  rows=r['trace'];peak=max(rows,key=lambda x:x['state']['noteZ']);air=[x for x in rows if x['floorForceN'][2]==0 and x['state']['noteZ']>.025401];returned=[x for x in rows if x['timeS']>peak['timeS'] and x['floorForceN'][2]>0] if air else []
  summaries.append(dict(name=name,outcome=r['outcome'],reason=r['reason'],stepS=r['stepS'],acceptedSteps=len(rows),durationS=r['final']['timeS']-r['initial']['timeS'],firstAirborneElapsedS=air[0]['timeS']-r['initial']['timeS'] if air else None,maximumNoteRiseM=peak['state']['noteZ']-.0254,peakElapsedS=peak['timeS']-r['initial']['timeS'],floorReturnElapsedS=returned[0]['timeS']-r['initial']['timeS'] if returned else None,finalNoteRiseM=r['final']['noteZ']-.0254,finalPivotRiseM=r['final']['zPivot']-.0254,finalRelativeDropM=r['final']['zPivot']-r['final']['noteZ'],finalNotePitchRad=r['final']['notePitch'],finalCradlePitchRad=r['final']['theta'],maximumIncrementalResidualJ=max(abs(x['energyJ']['residual']-r['inheritedResidualJ']) for x in rows),maximumCumulativeResidualJ=max(abs(x['energyJ']['residual']) for x in rows),receiptSha256=sha(path),runtimeSeconds=r['runtimeSeconds']))
 comparisons=[]
 pairs=[(names[0],names[1]),(names[1],names[2]),(names[3],names[4])]
 for pair in pairs:
  one,two=[runs[n] for n in pair];fine={round(x['timeS'],8):x for x in two['trace']};dp=dv=da=dw=0.;count=0
  for x in one['trace']:
   y=fine.get(round(x['timeS'],8))
   if y is None:continue
   count+=1;u,v=x['state'],y['state']
   dp=max(dp,*(abs(u[k]-v[k]) for k in ['noteZ','zPivot']),abs(u['chassis']['x']-v['chassis']['x']),abs(u['note']['p'][0]-v['note']['p'][0]))
   dv=max(dv,*(abs(u[k]-v[k]) for k in ['noteVz','vPivot']),abs(u['chassis']['vx']-v['chassis']['vx']),abs(u['note']['v'][0]-v['note']['v'][0]))
   da=max(da,*(abs(u[k]-v[k]) for k in ['theta','notePitch']));dw=max(dw,*(abs(u[k]-v[k]) for k in ['thetaRate','notePitchRate']))
  comparisons.append(dict(runs=list(pair),sharedStates=count,maximumPositionDifferenceM=dp,maximumVelocityDifferenceMps=dv,maximumPitchDifferenceRad=da,maximumPitchRateDifferenceRadps=dw,sameOutcome=one['outcome']==two['outcome'],withinDeclaredTranslationalBounds=one['outcome']==two['outcome'] and dp<=c['comparisonBounds']['positionM'] and dv<=c['comparisonBounds']['velocityMps']))
 pins=load(a.packet/'native-lift-dependency-pins.json')
 for p,h in pins['files'].items():assert sha(a.project/p)==h
 return dict(schema='native-pressure-lift-clocked-verification/1',inputs={p.name:sha(p) for p in [pathlib.Path(__file__),a.source,a.contract,a.packet/'native-lift-dependency-pins.json']},integrityChecksPassed=True,sourceTimeS=s['final']['timeS'],initialMechanicalDifferenceJ=r['initialMechanicalDifferenceJ'],clockedControllerEventsVerified=80,dependenciesUnchanged=True,comparisons=comparisons,runs=summaries,continuousGroundToTransientLift=True,retainedLiftSucceeded=False,continuousAcquisitionToLaunch=False,nativeMovingMassBound=False,fullAssemblyCollisionChecked=False,acceptance=False,hardwareValidated=False,limits=['All lift refinements start from the same attained half-millisecond ground-stop state, not independently refined full acquisition','Three timestep levels share exact integer controller ticks; initial floating-branch runs remain diagnostic only','Position/velocity comparison bounds do not certify material calibration or native clearance','Local spatial probes do not establish full spatial trajectory convergence','No static sticking state; floor, moving mass and transmission remain declared hypotheses; no feed or launch'])
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for n in ['packet','project','contract','source','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
 a=p.parse_args();assert not a.output.exists();r=verify(a);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r,indent=2))
