"""Finite nominal side-belt footprint witnesses; not a calibrated pressure model."""
import json,math,pathlib,hashlib
import build_connected_candidate as a
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN,TopAbs_ON,TopAbs_OUT
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus

def pins():
 p=a.input_pins();n='cad/assembly/extract_retention_patch.py';p[n]=hashlib.sha256((a.ROOT/n).read_bytes()).hexdigest();return p

def world(point,angle,pivot):
 x,y,z=point;return [pivot[0]+x*math.cos(angle)-z*math.sin(angle),y,pivot[2]+x*math.sin(angle)+z*math.cos(angle)]
def classify(shape,point):return BRepClass3d_SolidClassifier(shape,a.h.gp_Pnt(*point),1e-7).State()
def run():
 before=pins();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());R=c['note']['majorRadius'];r=c['note']['tubeRadius'];face=c['pinch']['innerFaceYAbs'];halfwidth=c['pinch']['beltWidth']/2;center=(c['pinch']['beltTeethHypothesis']-c['pinch']['teeth'])*c['pinch']['pitch']/2;rear=c['pinch']['beltCenterX']-center/2;front=c['pinch']['beltCenterX']+center/2;cases=[]
 for state in ['floor','rotated','handoff']:
  angle,z=a.pose(state,c);pivot=[c['stageCenterX'],0,z];belts={n:s for n,s,k in a.assembly.intake_components.build(state) if 'side pinch belt' in n}
  for offset in [0,9.88835038313412,10,20]:
   note_center=world([offset,0,0],angle,pivot);note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(*note_center),a.h.gp_Dir(-math.sin(angle),0,math.cos(angle))),R,r).Shape();points=[]
   for side in [-1,1]:
    belt=belts['intake candidate side pinch belt '+str(side)]
    for dx in [-12,-6,0,6,12]:
     for dz in [-3,0,3]:
      point=[offset+dx,side*face,dz];w=world(point,angle,pivot)
      if not rear<=point[0]<=front or abs(dz)>=halfwidth:raise RuntimeError('point outside straight belt face')
      if classify(belt,w)!=TopAbs_ON or classify(note,w)!=TopAbs_IN:raise RuntimeError('footprint witness failed')
      outward=world([point[0],point[1]+side*.001,dz],angle,pivot);inward=world([point[0],point[1]-side*.001,dz],angle,pivot)
      if classify(belt,outward)!=TopAbs_IN or classify(belt,inward)!=TopAbs_OUT:raise RuntimeError('normal witness failed')
      points.append({'side':side,'cradlePointMm':point,'noteRelativePointMm':[dx,side*face,dz],'worldPointMm':w,'inwardNormalCradle':[0,-side,0],'inwardNormalWorld':[0,-side,0]})
   cases.append({'state':state,'noteLongitudinalOffsetMm':offset,'noteCenterWorldMm':note_center,'pitchDeg':math.degrees(angle),'points':points})
 if before!=pins():raise RuntimeError('inputs changed')
 report={'schema':'finite-sidebelt-footprint-witness/1','inputs':before,'cases':cases,'analyticUnclippedFootprintHalfXAtMidheightMm':math.sqrt((R+r)**2-face**2),'analyticFootprintHalfZAtCenterMm':min(halfwidth,math.sqrt(r*r-(face-R)**2)),'selectedWitnessHalfSpanMm':{'x':12,'z':3},'nominalIndentationPerSideMm':R+r-face,'acceptance':False,'limits':['Points lie on native authored belt planar face and inside undeformed nominal NOTE, representing a possible compressed footprint, not measured contact pressures','Inward normals verified by inside/outside material samples; no lateral recentering model','15 witnesses per side are not15 independent full-normal-force contacts; allocate one side total preload across them','Tangential forces need friction limits and finite motor/belt response; no perfect attachment or retention inferred','Four longitudinal offsets (including recorded ground-dwell offset) and three prescribed CAD poses only; no dynamic path or contact persistence proof','Yaw, wear, actual foam stiffness, preload distribution and contact damage unknown']}
 out=a.ROOT/'artifacts/retention-patch';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'cases':len(cases),'pointsPerCase':len(cases[0]['points']),'halfX':report['analyticUnclippedFootprintHalfXAtMidheightMm']}))
if __name__=='__main__':run()
