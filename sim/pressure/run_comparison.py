"""Fresh standalone constitutive comparison, not a physical checkpoint handoff."""
import json,pathlib,sys,hashlib,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/integrated')]
from finite_area import evaluate,DomainError
from patch_registration import rotation
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/pressure/finite_area.py','sim/pressure/run_comparison.py','tests/test_finite_area_pressure.py','cad/assembly/belt_contact_geometry.py','cad/assembly/intake_components.json','sim/integrated/patch_registration.py','sim/pressure/source/cassette-travel-contract.json','sim/pressure/source/provenance.json']
def run(output):
 if output.exists():raise ValueError('fresh output required')
 pins={f:sha(R/f) for f in FILES};source=json.loads((R/'sim/integrated/evidence/fast-regulated-index.json').read_text());events=json.loads((R/'sim/integrated/evidence/lift-duration-domain/100ms.json').read_text())
 for path,report in [('sim/integrated/evidence/fast-regulated-index.json',source),('sim/integrated/evidence/lift-duration-domain/100ms.json',events)]:
  pins[path]=sha(R/path)
  for f,h in report['inputs'].items():
   if sha(R/f)!=h:raise ValueError('stale source '+f)
   if f in pins and pins[f]!=h:raise ValueError('conflicting source lineage')
   pins[f]=h
 frozen=json.loads((R/FILES[-2]).read_text());provenance=json.loads((R/FILES[-1]).read_text())
 if provenance['sha256']!=sha(R/FILES[-2]) or frozen['coordinate']['limitsM']!=[0,.0015]:raise ValueError('cassette source convention changed')
 c=json.loads((R/'cad/assembly/intake_components.json').read_text())
 if c['pinch']['innerFaceYAbs']/1000!=frozen['registeredGeometry']['baselineBeltInnerFaceYmetres']:raise ValueError('belt source registration differs')
 checkpoint=events['cases'][0]['initialState'];fixtures=[('indexed-diagnostic',checkpoint)]
 for case in events['cases']:
  if case['rejectedEvaluation']:fixtures.append((case['name']+'-rejected-trial',case['rejectedEvaluation']['state']))
 rows=[]
 for name,s in fixtures:
  d=s['drive'];n=s['note'];hd=d['heading'];origin=[d['x']+.178*math.cos(hd),d['y']+.178*math.sin(hd),s['zPivot']]
  for resolution in [32,64,128,256,512,1024]:
   args=dict(note_center=[*n['p'],s['noteZ']],note_basis=rotation(n['yaw'],s['notePitch']),cradle_origin=origin,cradle_basis=rotation(hd,s['theta']),resolution=(resolution,resolution),u_positive_m=0.,k_area=1e7)
   try:
    value=evaluate(c,**args);row=dict(fixture=name,sourceState=s,arguments=args,result=value,outcome='SAMPLED_CONSTITUTIVE_RESULT',physicalHandoff=False)
    if name=='indexed-diagnostic':row['modelChangePotentialDeltaJ']=value['potentialJ']-source['final']['energyJ']['spring']
    print(name,resolution,value['potentialJ'],value['sides'][1]['normalMagnitudeN'],value['features'],flush=True)
   except DomainError as e:row=dict(fixture=name,sourceState=s,arguments=args,outcome='UNKNOWN_DOMAIN',reason=str(e),physicalHandoff=False);print(name,resolution,str(e),flush=True)
   rows.append(row)
 # Stiffness sensitivity is linear for this fixed geometry and law, not a fitted foam model.
 initial=next(row for row in rows if row['fixture']=='indexed-diagnostic' and row['arguments']['resolution']==(1024,1024))
 sensitivity=[]
 for k in [1e6,1e7,1e8]:
  args=dict(initial['arguments'],k_area=k);v=evaluate(c,**args);sensitivity.append(dict(kAreaNpm3=k,result=v,modelChangePotentialDeltaJ=v['potentialJ']-source['final']['energyJ']['spring']))
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 report=dict(schema='finite-area-pressure-standalone/1',inputs=pins,fixtures=rows,stiffnessSensitivity=sensitivity,legacyNormalNPerSide=[x['normalN'] for x in source['final']['contacts']],legacyNormalSpringJ=source['final']['energyJ']['spring'],physicalHandoff=False,acceptance=False,limits=['Penalty density1e7 N/m3 is hypothetical, not measured or fitted material.','Material torus surface quadrature has no active-cell force renormalization; coarse samples can miss contact entirely.','Finite straight sidewall and top/bottom caps use source SDF gradients; entry wraps, deep/cavity and gradient-tie contacts stop as unknown.','No friction, belt motor, cassette dynamics or spring equilibrium is implemented in this standalone law.','Actual positive-side translation convention is sourced, but all comparison fixtures keepu=0. No negative-side moving-cassette qualification.','Indexed state is a diagnostic fixture: new potential minus inherited reservoir is exposed, not hidden by a reference shift.','A fresh acquisition from no contact under the same new law is required before claiming continuous handoff.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
