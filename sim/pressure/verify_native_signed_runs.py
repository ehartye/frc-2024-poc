"""Check source-bound fresh trajectories and finite finest-pair differences."""
import argparse,hashlib,json,math,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def verify(folder,contract):
 c=json.loads(contract.read_bytes());runs=[];summaries=[]
 for name in ['native-signed-1ms','native-signed-half-ms','native-signed-quarter-ms']:
  file=folder/name/'receipt.json';r=json.loads(file.read_bytes());runs.append(r)
  for name,h in r['inputs'].items():assert sha(folder/name)==h,(name,'source mismatch')
  assert r['initial']['timeS']==0 and r['initial']['initialEnergyJ']==0 and all(x==0 for x in r['initial']['ledger'].values())
  assert r['initial']['note']==dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.])
  dt=r['stepS'];trace=r['trace'];assert abs(r['final']['timeS']-len(trace)*dt)<1e-9
  for i,row in enumerate(trace):
   assert abs(row['timeS']-(i+1)*dt)<1e-9
   assert abs(row['pressureWorkResidualW'])<1e-9
   assert abs(row['sourcePowerResidualW'])<1e-8
   for e in row['pinchElectrical']:assert abs(e['currentA'])<=40 and e['terminalPowerW']>=-1e-10 and abs(e['powerResidualW'])<1e-9
  if r['index']:
   i=round(r['index']['timeS']/dt)-1;row=trace[i]
   for k,field in [('chassis','chassis'),('note','note')]:assert r['index'][k]==row[field]
   assert r['index']['ledger']=={k:row['energyJ'][k] for k in r['index']['ledger']}
   assert row['phase']=='acquisition' and trace[i+1]['phase']=='transport'
  transport=[x for x in trace if x['phase']=='transport'];start=r['index'];end=transport[-1] if transport else None
  chassis_travel=end['chassis']['x']-start['chassis']['x'] if end else None;note_travel=end['note']['p'][0]-start['note']['p'][0] if end else None
  stopped=[x for x in trace if x['phase']=='stop' and x['timeS']>=r['final']['timeS']-.1-1e-10]
  stop_ok=len(stopped)>=round(.1/dt) and all(abs(x['chassis']['vx'])<=.003 and abs(x['note']['v'][0]-x['chassis']['vx'])<=.003 for x in stopped)
  ground_pass=r['outcome']=='BOUNDED_GROUND_SEQUENCE_COMPLETE' and chassis_travel>=.2 and note_travel>=.19 and stop_ok
  summaries.append(dict(outcome=r['outcome'],reason=r['reason'],stepS=dt,endTimeS=r['final']['timeS'],indexTimeS=start['timeS'] if start else None,transportDurationS=end['timeS']-start['timeS'] if end else None,chassisTravelM=chassis_travel,noteTravelM=note_travel,stoppedLast100ms=stop_ok,boundedGroundMetricsPassed=ground_pass,maximumResidualJ=max(abs(x['energyJ']['residual']) for x in trace),minimumPinchCurrentA=min(e['currentA'] for x in trace for e in x['pinchElectrical']),traceRows=len(trace),receiptSha256=sha(file),runtimeSeconds=r['runtimeSeconds']))
 pairs=[]
 for one,two in zip(runs,runs[1:]):
  limit=min(one['index']['timeS'] if one['index'] else one['final']['timeS'],two['index']['timeS'] if two['index'] else two['final']['timeS']);fine={round(x['timeS'],8):x for x in two['trace']};dp=dv=0.;n=0
  for x in one['trace']:
   if x['timeS']>limit+1e-9:break
   y=fine[round(x['timeS'],8)];n+=1;dp=max(dp,abs(x['relativeRobotXY'][0]-y['relativeRobotXY'][0]));dv=max(dv,abs((x['note']['v'][0]-x['chassis']['vx'])-(y['note']['v'][0]-y['chassis']['vx'])))
  phase_n=0
  if one['index'] and two['index']:
   phase_fine={round(x['timeS']-two['index']['timeS'],8):x for x in two['trace'] if x['timeS']>two['index']['timeS']+1e-9}
   for x in one['trace']:
    key=round(x['timeS']-one['index']['timeS'],8)
    if key not in phase_fine:continue
    y=phase_fine[key];phase_n+=1;dp=max(dp,abs(x['relativeRobotXY'][0]-y['relativeRobotXY'][0]));dv=max(dv,abs((x['note']['v'][0]-x['chassis']['vx'])-(y['note']['v'][0]-y['chassis']['vx'])))
  indexdelta=abs(one['index']['timeS']-two['index']['timeS']) if one['index'] and two['index'] else None
  durationdelta=abs((one['final']['timeS']-one['index']['timeS'])-(two['final']['timeS']-two['index']['timeS'])) if one['index'] and two['index'] else None
  bounds=c['finestPairBounds'];passed=one['outcome']==two['outcome'] and indexdelta is not None and durationdelta is not None and dp<=bounds['relativePositionM'] and dv<=bounds['relativeVelocityMps'] and indexdelta<=bounds['indexEventTimeS'] and durationdelta<=bounds['transportEventTimeS']
  pairs.append(dict(stepsS=[one['stepS'],two['stepS']],sharedAcquisitionStates=n,sharedPostIndexStates=phase_n,maximumRelativePositionDifferenceM=dp,maximumRelativeVelocityDifferenceMps=dv,indexEventDifferenceS=indexdelta,postIndexDurationDifferenceS=durationdelta,withinDeclaredBounds=passed))
 return dict(schema='native-signed-fresh-verification/1',integrityChecksPassed=True,numericalGatePassed=pairs[-1]['withinDeclaredBounds'],acceptance=False,hardwareValidated=False,continuousCycle=False,runs=summaries,comparisons=pairs,contractSha256=sha(contract),limits=['Fresh ground acquisition only until actual phase gate; no imported capture','No complete native collision model, lifting, pitch or feed qualification','Finite refinement and uncalibrated constitutive assumptions; not hardware validation'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--packet',type=pathlib.Path,required=True);p.add_argument('--contract',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists();r=verify(a.packet,a.contract);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r,indent=2))
