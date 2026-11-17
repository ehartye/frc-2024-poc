"""Geometric retention candidate at exact coupled-drive end state; no attached-body model."""
import json,math,pathlib,hashlib,argparse
import build_connected_candidate as a
from extract_retention_patch import world,classify
from audit_intake_motion import poses
from audit_cartridge_interfaces import common
from OCP.TopAbs import TopAbs_IN,TopAbs_ON,TopAbs_OUT
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=a.input_pins()
 for f in ['cad/assembly/check_coupled_retention_geometry.py','cad/assembly/extract_retention_patch.py','cad/assembly/audit_cartridge_interfaces.py']:p[f]=sha(a.ROOT/f)
 return p

def run(runtime):
 before=pins();files=['sim/integrated/evidence/speed-approach-nominal.json','sim/integrated/evidence/retention-entry-bound.json'];external={f:sha(runtime/f) for f in files};reports=[json.loads((runtime/f).read_text()) for f in files]
 def verify():
  for f,report in zip(files,reports):
   if sha(runtime/f)!=external[f] or any(sha(runtime/p)!=v for p,v in report['inputs'].items()):raise ValueError('stale runtime input')
 verify();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());state=reports[0]['final'];offset=state['relativeRobotXY'][0]*1000-c['stageCenterX'];dy=state['relativeRobotXY'][1]*1000
 if abs(dy)>1e-8:raise ValueError('noncentral state')
 for f,v in a.input_pins().items():
  if reports[0]['inputs'].get(f)!=v:raise ValueError('CAD lineage mismatch '+f)
 R=c['note']['majorRadius'];r=c['note']['tubeRadius'];p=c['pinch'];face=p['innerFaceYAbs'];span=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;rear=p['beltCenterX']-span/2;front=p['beltCenterX']+span/2;half=math.sqrt((R+r)**2-face**2);base=[(n,s,k) for n,s,k in a.assembly.build('floor') if not n.startswith('intake candidate')];rows=[]
 for phase,deg,z in poses(c):
  angle=math.radians(deg);pivot=[c['stageCenterX'],0,z];center=world([offset,dy,0],angle,pivot);note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(*center),a.h.gp_Dir(-math.sin(angle),0,math.cos(angle))),R,r).Shape();parts=base+a.assembly.intake_components.build(c=c,pose=(deg,z));parts,_=a.upper.lower.apply(parts,c);parts,_=a.upper.apply(parts,z,c);parts,_=a.pads.apply(parts,z,c);bounds=a.assembly.robot.bounds(note);hits=[];points=[]
  for n,s,k in parts:
   if k=='service':continue
   q=a.assembly.robot.bounds(s)
   if min(min(bounds[j+3],q[j+3])-max(bounds[j],q[j]) for j in range(3))<=1e-6:continue
   v=common(note,s)
   if v>1e-5:hits.append({'part':n,'commonMm3':v,'intendedContactHypothesis':'side pinch belt' in n or 'nominal tread annulus' in n})
  belts={n:s for n,s,k in parts if 'side pinch belt' in n}
  for side in [-1,1]:
   belt=belts['intake candidate side pinch belt '+str(side)]
   for dx in [-12,-6,0,6,12]:
    for dz in [-3,0,3]:
     pt=[offset+dx,side*face,dz];w=world(pt,angle,pivot)
     if not rear<=pt[0]<=front or classify(belt,w)!=TopAbs_ON or classify(note,w)!=TopAbs_IN:raise ValueError('patch witness')
     if classify(belt,world([pt[0],pt[1]+side*.001,dz],angle,pivot))!=TopAbs_IN or classify(belt,world([pt[0],pt[1]-side*.001,dz],angle,pivot))!=TopAbs_OUT:raise ValueError('normal')
     points.append({'side':side,'cradlePointMm':pt,'noteRelativePointMm':[dx,side*face,dz],'worldPointMm':w,'inwardNormalWorld':[0,-side,0]})
  rows.append({'phase':phase,'pitchDeg':deg,'centerRobotMm':center,'partCount':len(parts),'points':points,'hits':hits});print(phase,deg,len([h for h in hits if not h['intendedContactHypothesis']]),flush=True)
 verify()
 if before!=pins():raise ValueError('CAD changed')
 result={'schema':'coupled-retention-geometry/1','inputs':before,'runtimeReceipts':external,'runtimeInputs':{f:report['inputs'] for f,report in zip(files,reports)},'preservedEntryState':state,'longitudinalOffsetMm':offset,'straightFaceXmm':[rear,front],'footprintXmm':[offset-half,offset+half],'forceBound':reports[1]['bound'],'samples':rows,'adopted':False,'acceptance':False,'limits':['Conditional rigid carried-pose geometry test, NOT integrated retained-body dynamics or an attachment assumption','Actual end state preserved as input; nonzero velocity/rotor states not consumed or reset by this study','30 patch points share one normal budget per side; no pressure or stiffness calibration','Current end-state force budget fails necessary static lift condition before motion','Canonical source-sized assembly retains known bearing underbound; separate94ab67d candidate not silently adopted','Finite15poses, no continuous sweep or physical capture qualification']}
 out=a.ROOT/'artifacts/coupled-retention-geometry';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--runtime',type=pathlib.Path,required=True);args=p.parse_args();run(args.runtime)
