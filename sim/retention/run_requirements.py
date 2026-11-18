"""Required wrench screens on verified finite footprint witnesses; no ideal attachment."""
import pathlib,json,hashlib,sys,math
from collections import Counter
import wrench
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'cad/assembly'));import extract_retention_patch as source
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output):
 if output.exists():raise ValueError('Fresh output required')
 gp=R/'cad/assembly/evidence/retention-patch.json';g=json.loads(gp.read_text())
 if g['schema']!='finite-sidebelt-footprint-witness/1' or g['inputs']!=source.pins():raise ValueError('Current complete patch source required')
 c=json.loads((R/'cad/assembly/intake_components.json').read_text())
 ground_path=R/'sim/acquisition/evidence.json';ground=json.loads(ground_path.read_text());final=next(x for x in ground['cases'] if x['case']=='braked-hypothesis')['final'];recorded_offset=final['relativeRobotX']*1000-c['stageCenterX']
 if abs(final['pieceWorldSpeedMps'])>1e-9 or final['robotSpeedMps']!=0:raise ValueError('Rest-start demand is not supported by ground observation')
 offsets=[0,9.88835038313412,10,20]
 if not math.isclose(recorded_offset,offsets[1],abs_tol=1e-10):raise ValueError('Ground state/patch offset mismatch')
 if [(x['state'],x['noteLongitudinalOffsetMm']) for x in g['cases']]!=[(s,o) for s in ['floor','rotated','handoff'] for o in offsets]:raise ValueError('Complete12-case patch inventory required')
 expected={(side,dx,side*c['pinch']['innerFaceYAbs'],dz) for side in [-1,1] for dx in [-12,-6,0,6,12] for dz in [-3,0,3]}
 for case in g['cases']:
  if len(case['points'])!=30 or {(p['side'],*p['noteRelativePointMm']) for p in case['points']}!=expected:raise ValueError('Complete expected material witnesses required')
 mass=.235;major=c['note']['majorRadius']/1000;minor=c['note']['tubeRadius']/1000;inertia=mass*(major**2/2+5*minor**2/8);pitch=math.radians(c['pitchDeg']);lift=(c['handoffCenterZ']-c['floorCenterZ'])/1000-major*math.sin(pitch)
 files=[pathlib.Path(__file__),R/'sim/retention/wrench.py',gp,ground_path];pins=dict(source.pins(),**{p.relative_to(R).as_posix():sha(p) for p in files});cases=[]
 settings=[('nominal',.5,.6,10.,.6),('slower',1.,1.2,10.,.6),('fast',.15,.2,10.,.6),('low-preload',.5,.6,2.,.6),('low-friction',.5,.6,10.,.2),('point-contact-control',.5,.6,10.,.6),('one-side-loss',.5,.6,10.,.6)]
 for geo in [x for x in g['cases'] if x['state']=='floor']:
  offset=geo['noteLongitudinalOffsetMm']/1000
  for label,tr,tl,normal,mu in settings:
   points=[dict(side=p['side'],positionM=[v/1000 for v in p['noteRelativePointMm']]) for p in geo['points']]
   if len(points)!=30 or len({(p['side'],*p['positionM']) for p in points})!=30:raise ValueError('30 distinct patch witnesses required')
   if any(p['inwardNormalCradle']!=[0,-p['side'],0] for p in geo['points']):raise ValueError('Expected bilateral normal directions')
   if label=='point-contact-control':points=[dict(side=p['side'],positionM=[0.,p['positionM'][1],0.]) for p in points]
   normal_by_side={-1:0. if label=='one-side-loss' else normal,1:normal};rows=[];count=max(2,math.ceil((tr+tl)/.005))
   for k in range(count+1):
    t=(tr+tl)*k/count;d=wrench.demand(t,tr,tl,pitch,major,mass,inertia,offset,lift);a=wrench.allocate(points,d['forceLocalXZ'],d['torqueNm'],mu,normal_by_side)
    radius=c['pinch']['teeth']*c['pinch']['pitch']/(2000*math.pi);phase=[]
    for side in [-1,1]:phase.append(abs(sum(f[0] for p,f in zip(points,a['pointForcesXZ']) if p['side']==side))*radius*50.39638215133627)
    a.pop('pointForcesXZ');a.pop('pointCapacitiesN')
    rows.append(dict(timeS=t,**d,**a,minimumIdealActiveHoldPhaseA=phase,powerOffStaticHoldSupported=abs(d['forceLocalXZ'][0])<=1e-9))
   cases.append(dict(case=label,noteOffsetMm=offset*1000,normalPerSideN=normal_by_side,friction=mu,rotationTimeS=tr,liftTimeS=tl,statusCounts=dict(Counter(x['status'] for x in rows)),maximumUtilization=max(x['maxUtilization'] for x in rows),rows=rows,acceptance=False))
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('Inputs changed')
 report=dict(schema='frc-finite-patch-wrench-requirements/1',inputs=pins,massKg=mass,uniformSolidTorusPitchInertiaHypothesisKgM2=inertia,trajectoryRequirementsOnly=True,pythonVersion=sys.version,numpyVersion=wrench.np.__version__,cases=cases,acceptance=False,limits=['No attached NOTE trajectory is executed; this calculates forces/torque a desired motion would require.','One total normal budget per side is uniformly allocated among15 witnesses; preload,friction,pressuredistribution and persistence unmeasured.','Constructive allocation inside friction circles is conditional; a failed construction without a necessary bound violation is UNKNOWN, not impossible.','Pitch inertia assumes a uniform solid torus; actual NOTE mass distribution/wear unmeasured.','Intermediate pitch/contact persistence not revalidated; endpoint material witnesses are not continuous deformable contact proof.','Active hold current is an ideal1:1 reciprocal-DC lower-bound requirement, not verified controller/motor holding ability.','Power-off static support along belt travel is not assumed; lack of static hold is not a safety conclusion or forced retainer requirement.','Lateral/yaw/roll imbalances stay unresolved; +/-5mm entry counterexamples remain separate.'])
 output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return report
if __name__=='__main__':
 import argparse;a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);v=a.parse_args();print([(c['case'],c['noteOffsetMm'],c['statusCounts']) for c in run(v.output)['cases']])
