"""Verify finite-effort retained-lift evidence without promoting calibration."""
import argparse,copy,hashlib,json,pathlib,sys
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def difference(a,b):
 if isinstance(a,dict):assert a.keys()==b.keys();return max((difference(v,b[k]) for k,v in a.items()),default=0.)
 if isinstance(a,list):assert len(a)==len(b);return max((difference(x,y) for x,y in zip(a,b)),default=0.)
 if isinstance(a,(int,float)):return abs(a-b)
 assert a==b;return 0.
def verify(a):
 load=lambda p:json.loads(p.read_bytes());source=load(a.source);contract=load(a.packet/'native-static-lift-contract.json');base=load(a.packet/'native-pressure-lift-contract.json');binding=load(a.packet/'native-pressure-binding.json');pins=load(a.packet/'native-static-lift-dependency-pins.json')
 for p,h in pins['files'].items():assert sha(a.project/p)==h
 sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
 from native_static_lift import NativeStaticLift
 dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json')
 names=['native-static-lift-'+x for x in ['half','quarter','eighth','zero','low','coast','coast-resolved','hold-half']];runs={};summaries=[];replays=[]
 for name in names:
  path=a.packet/name/'receipt.json';r=load(path);runs[name]=r;cname='native-static-lift-hold-contract.json' if name.endswith('hold-half') else 'native-static-lift-coast-solver-contract.json' if name.endswith('coast-resolved') else 'native-static-lift-contract.json';c=load(a.packet/cname)
  for key,p in [('runner',a.packet/'run_native_static_lift.py'),('model',a.packet/'native_static_lift.py'),('friction',a.packet/'native_dry_friction.py'),('sourceReceipt',a.source),('binding',a.packet/'native-pressure-binding.json'),('contract',a.packet/cname),('baseContract',a.packet/'native-pressure-lift-contract.json')]:assert r['inputs'][key]==sha(p),(name,key)
  for k,v in source['final'].items():
   if k=='ledger':
    assert all(r['initial']['ledger'][j]==x for j,x in v.items());assert all(x==0 for j,x in r['initial']['ledger'].items() if j not in v)
   else:assert r['initial'][k]==v
  assert abs(r['initialMechanicalDifferenceJ'])<1e-12
  m=NativeStaticLift(source['final'],dc,nc,cad,binding,ac,base,friction_mu=r['mu'],solver_tolerance=c['solverGapToleranceJ'],solver_iterations=c['solverMaxIterations']);rows=r['trace'];peak_current=peak_duty=0.;previous=r['initial'];before=[]
  for i,row in enumerate(rows):
   assert abs(row['timeS']-r['initial']['timeS']-(i+1)*r['stepS'])<1e-9;event=r['events'][i//round(.01/r['stepS'])];cmd=event['command'];tick=i//round(.01/r['stepS']);t=tick*.01;desired=.0254+(.05*t*t if tick<20 else .002+.02*(t-.2) if tick<50 else .008)
   assert abs(cmd['requestedPivotZM']-desired)<1e-15 and cmd['pinch']==[0.,0.];de,pe,ae=m.electrical(previous,row['busV'],cmd);peak_current=max(peak_current,*(abs(x['currentA']) for x in ae['motors']));peak_duty=max(peak_duty,*map(abs,cmd['actuator']));assert all(x['currentA']==0 for x in pe);assert not ae['infeasibleCurrentLimitVoltage']
   assert abs(row['sourcePowerResidualW'])<1e-8 and abs(row['pressureWorkResidualW'])<1e-8;f=row['friction'];assert f['gapJ']<=c['solverGapToleranceJ'] and f['maximumConeExcessNs']<1e-12 and abs(f['projectionIdentityJ'])<1e-9 and f['heatJ']>=-c['solverGapToleranceJ'];before.append(previous);previous=row['state']
  for index in sorted(set([0,len(rows)//4,len(rows)//2,3*len(rows)//4,len(rows)-1])):
   if index<0:continue
   m.s=copy.deepcopy(before[index]);m.ledger=copy.deepcopy(m.s['ledger']);cmd=r['events'][index//round(.01/r['stepS'])]['command'];replayed=m.step(r['stepS'],cmd);error=difference(replayed,rows[index]);assert error<1e-12,(name,index,error);replays.append(dict(run=name,step=index+1,maximumRecordedDifference=error))
  last=r['final'];gate=contract['retentionGate'];retained=r['outcome']=='HORIZON' and r['requestedDurationS']>=.8 and last['noteZ']-.0254>=gate['minimumFinalRiseM'] and abs(last['noteZ']-last['zPivot'])<=gate['maximumRelativeVerticalDriftM'] and abs(last['noteVz']-last['vPivot'])<=gate['maximumFinalRelativeVerticalSpeedMps'];assert retained==r['retentionGatePassed']
  summaries.append(dict(name=name,stepS=r['stepS'],mu=r['mu'],outcome=r['outcome'],reason=r['reason'],acceptedSteps=len(rows),durationS=last['timeS']-r['initial']['timeS'],retentionGatePassed=retained,finalNoteRiseM=last['noteZ']-.0254,finalPivotRiseM=last['zPivot']-.0254,maximumRelativeVerticalDriftM=max(abs(x['state']['noteZ']-x['state']['zPivot']) for x in rows),finalRelativeVerticalSpeedMps=last['noteVz']-last['vPivot'],finalNoteVerticalSpeedMps=last['noteVz'],maximumIncrementalEnergyResidualJ=max(abs(x['energyJ']['residual']-r['inheritedResidualJ']) for x in rows),maximumSolverGapJ=max(x['friction']['gapJ'] for x in rows),maximumConeExcessNs=max(x['friction']['maximumConeExcessNs'] for x in rows),maximumIterations=max(x['friction']['iterations'] for x in rows),peakActuatorCurrentA=peak_current,peakActuatorDuty=peak_duty,minimumBusV=min(x['busV'] for x in rows),finalPinchRotorOmegaRadps=last['note']['w'],receiptSha256=sha(path),runtimeSeconds=r['runtimeSeconds']))
 comparisons=[]
 for one,two in [('half','quarter'),('quarter','eighth')]:
  first=runs['native-static-lift-'+one];second=runs['native-static-lift-'+two];fine={round(x['timeS'],8):x for x in second['trace']};dp=dv=da=dw=0.
  for x in first['trace']:
   y=fine[round(x['timeS'],8)];u,v=x['state'],y['state'];dp=max(dp,*(abs(u[k]-v[k]) for k in ['noteZ','zPivot']),abs(u['chassis']['x']-v['chassis']['x']),abs(u['note']['p'][0]-v['note']['p'][0]));dv=max(dv,*(abs(u[k]-v[k]) for k in ['noteVz','vPivot']),abs(u['chassis']['vx']-v['chassis']['vx']),abs(u['note']['v'][0]-v['note']['v'][0]));da=max(da,*(abs(u[k]-v[k]) for k in ['theta','notePitch']));dw=max(dw,*(abs(u[k]-v[k]) for k in ['thetaRate','notePitchRate']))
  comparisons.append(dict(coarse=one,fine=two,sharedStates=len(first['trace']),maximumPositionDifferenceM=dp,maximumVelocityDifferenceMps=dv,maximumPitchDifferenceRad=da,maximumPitchRateDifferenceRadps=dw,withinDeclaredTranslationalBounds=dp<=contract['comparisonBounds']['positionM'] and dv<=contract['comparisonBounds']['velocityMps']))
 long=runs['native-static-lift-hold-half'];hc=load(a.packet/'native-static-lift-hold-contract.json')['holdCheck'];window=[x for x in long['trace'] if x['timeS']>=long['final']['timeS']-hc['windowS']-1e-10];hold=dict(windowS=hc['windowS'],minimumNoteRiseM=min(x['state']['noteZ']-.0254 for x in window),maximumAbsoluteNoteVerticalSpeedMps=max(abs(x['state']['noteVz']) for x in window),maximumRelativeVerticalSpeedMps=max(abs(x['state']['noteVz']-x['state']['vPivot']) for x in window),maximumRelativeVerticalDriftM=max(abs(x['state']['noteZ']-x['state']['zPivot']) for x in window),maximumFloorForceN=max(abs(x['floorForceN'][2]) for x in window));hold['passed']=hold['minimumNoteRiseM']>=hc['minimumRiseM'] and hold['maximumAbsoluteNoteVerticalSpeedMps']<=hc['maximumAbsoluteNoteVerticalSpeedMps'] and hold['maximumRelativeVerticalSpeedMps']<=hc['maximumRelativeVerticalSpeedMps'] and hold['maximumRelativeVerticalDriftM']<=hc['maximumRelativeVerticalDriftM'] and hold['maximumFloorForceN']==0
 assert all(x['retentionGatePassed'] for x in summaries[:3]) and not summaries[3]['retentionGatePassed'] and not summaries[4]['retentionGatePassed']
 return dict(schema='native-static-lift-verification/1',producerSha256=sha(pathlib.Path(__file__)),integrityChecksPassed=True,dependencyPinsVerified=True,sourceReceiptSha256=sha(a.source),initialMechanicalDifferenceJ=long['initialMechanicalDifferenceJ'],runs=summaries,comparisons=comparisons,longHold=hold,selectedStepReplays=replays,retainedLiftModelEvidence=True,acceptance=False,hardwareValidated=False,continuousFeed=False,sameLawFreshAcquisition=False,limits=['Separate constitutive/integration and pinch-policy switch at attained stop; state/energy continuity is not same-law full-cycle verification','Coast controls are preserved numerical solver/energy rejections, not validated physical drops','Timestep comparisons are local from one attained source; no full spatial trajectory refinement','Native mass/transmission and whole-assembly motion clearance remain unbound; friction/compliance/floor assumptions uncalibrated'])
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for name in ['packet','project','source','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
 a=p.parse_args();assert not a.output.exists();r=verify(a);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:v for k,v in r.items() if k!='selectedStepReplays'},indent=2))
