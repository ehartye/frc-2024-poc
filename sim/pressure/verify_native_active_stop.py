"""Validate exact transport-to-stop composition and local stop refinement."""
import argparse,hashlib,json,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for n in ['trajectory','coarse','fine','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();load=lambda p:json.loads(p.read_bytes());source=load(a.trajectory);one=load(a.coarse);two=load(a.fine);c=load(a.contract);summaries=[]
for path,r in [(a.coarse,one),(a.fine,two)]:
 assert r['sourceTrajectorySha256']==sha(a.trajectory) and r['initial']==source['transportEnd']==r['sourceTransportCheckpoint'];assert r['contractSha256']==sha(a.contract)
 assert r['outcome']=='ACTIVE_STOP_HORIZON' and r['settledLast100ms']
 assert abs(r['final']['timeS']-r['initial']['timeS']-len(r['trace'])*r['stepS'])<1e-9
 for x in r['trace']:
  assert .180<=x['relativeRobotXY'][0]<=.190 and abs(x['note']['v'][0]-x['chassis']['vx'])<=.01
  assert abs(x['pressureWorkResidualW'])<1e-9 and abs(x['sourcePowerResidualW'])<1e-8
  assert abs(x['energyJ']['residual'])<=.01+.01*x['energyJ']['sourceWork']
  for e in x['pinchElectrical']:assert abs(e['currentA'])<=40 and e['terminalPowerW']>=-1e-10
 final=r['trace'][-1];summaries.append(dict(stepS=r['stepS'],traceRows=len(r['trace']),outcome=r['outcome'],endTimeS=r['final']['timeS'],finalRelativeXM=final['relativeRobotXY'][0],finalChassisSpeedMps=final['chassis']['vx'],finalRelativeSpeedMps=final['note']['v'][0]-final['chassis']['vx'],maximumResidualJ=r['maximumResidualJ'],receiptSha256=sha(path)))
fine={round(x['timeS'],8):x for x in two['trace']};dp=dv=0.
for x in one['trace']:
 y=fine[round(x['timeS'],8)];dp=max(dp,abs(x['relativeRobotXY'][0]-y['relativeRobotXY'][0]));dv=max(dv,abs((x['note']['v'][0]-x['chassis']['vx'])-(y['note']['v'][0]-y['chassis']['vx'])))
passed=dp<=c['finestPairBounds']['relativePositionM'] and dv<=c['finestPairBounds']['relativeVelocityMps']
transport=source['transportEnd'];index=source['index'];chassis_travel=transport['chassis']['x']-index['chassis']['x'];note_travel=transport['note']['p'][0]-index['note']['p'][0];assert chassis_travel>=.2 and note_travel>=.19
out=dict(schema='native-signed-active-stop-verification/1',integrityChecksPassed=True,localStopNumericalBoundsPassed=passed,attainedGroundSequenceMetricsPassed=passed,sourceFreshStepS=source['stepS'],exactTransportJoin=True,chassisTransportTravelM=chassis_travel,noteTransportTravelM=note_travel,joinedUniformHalfStepCount=round(transport['timeS']/one['stepS'])+len(one['trace']),runs=summaries,maximumRelativePositionDifferenceM=dp,maximumRelativeVelocityDifferenceMps=dv,acceptance=False,hardwareValidated=False,continuousAcquisitionToLaunch=False,limits=['Both stop refinements use the same attained half-millisecond fresh trajectory checkpoint; full independent fine-start active-stop replay is not included','Ground-supported finite uncalibrated model only; no lift, feed or hardware qualification'])
a.output.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(out,indent=2))
