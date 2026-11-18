"""Export conservative per-solid obstacle domains; no clearance acceptance."""
import json,pathlib,sys,hashlib,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'cad'),str(R/'cad/assembly')]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def verify_cover(shape,boxes,h):
 from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
 from OCP.TopExp import TopExp_Explorer
 from OCP.TopAbs import TopAbs_SOLID
 remaining=shape
 for b in boxes:
  tool=h.box(*b[:3],*[b[i+3]-b[i] for i in range(3)]);op=BRepAlgoAPI_Cut(remaining,tool)
  if not op.IsDone() or not h.BRepCheck_Analyzer(op.Shape()).IsValid():raise ValueError('invalid coverage Boolean')
  remaining=op.Shape()
  if not TopExp_Explorer(remaining,TopAbs_SOLID).More():return
 raise ValueError('uncovered weldment material remains')

def export(output):
 import build_connected_candidate as a
 import belt_contact_geometry as belt
 from OCP.TopExp import TopExp_Explorer
 from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
 before=a.input_pins();extra=['cad/components/a2-wheel-cartridge.json','cad/assembly/belt_contact_geometry.py','sim/acquisition/export_obstacles.py']
 before.update({p:sha(R/p) for p in extra});c=json.loads((R/'cad/assembly/intake_components.json').read_text());g=belt.geometry(c);parts,_=a.build('floor');obstacles=[];excluded=[];coverage=[]
 if len(parts)!=711:raise ValueError('711-part floor source required')
 for n,s,k in parts:
  if n in ['intake candidate side pinch belt -1','intake candidate side pinch belt 1']:excluded.append(n);continue
  if not a.h.BRepCheck_Analyzer(s).IsValid() or TopExp_Explorer(s,TopAbs_FACE,TopAbs_SOLID).More():raise ValueError('invalid or loose material '+n)
  b=a.assembly.robot.bounds(s)
  if n.startswith('intake candidate original cradle side weldment'):
   side=int(n.rsplit(' ',1)[1]);y=side*g['axisYAbs'];pp=c['pinch'];boxes=[]
   def add_box(x,y,z,dx,dy,dz):boxes.append([x+g['floorOrigin'][0],y,z+g['floorOrigin'][2],x+dx+g['floorOrigin'][0],y+dy,z+dz+g['floorOrigin'][2]])
   start=min(-40,g['rearX']-20);add_box(start,side*c['cradleSideYAbs']-3.175,-12.7,g['frontX']+20-start,6.35,25.4)
   for x in [g['rearX'],g['frontX']]:
    for zz in [pp['lowerBearingZ'],pp['upperBearingZ']]:
     add_box(x-17,y-17,zz-4.7625,34,34,3.175)
     if x<0:
      wx=max(x,-95);add_box(x-10,min(side*185.3,side*205.3),zz-4.7625,wx-(x-10),20,3.175)
     else:
      wx=x;add_box(x-10,min(side*185.3,side*207),zz-4.7625,20,21.7,3.175)
     low=min(-12.7,zz-4.7625);high=max(12.7,zz-1.5875);add_box(wx-10,side*c['cradleSideYAbs']-3.175,low,20,6.35,high-low)
   boxes=[[v-1e-6 if j<3 else v+1e-6 for j,v in enumerate(b)] for b in boxes]
   verify_cover(s,boxes,a.h);coverage.append(dict(name=n,remainingSolidCount=0,boxCount=len(boxes)))
   for i,box in enumerate(boxes):obstacles.append(dict(name=n,solid=i,role=k,boundsMm=box,basis='conservative union of exact authored stock/carrier/arm/web constituent bounds before cuts'))
  elif n.startswith('intake candidate 12T track pulley outer-profile representation'):
   x=(b[0]+b[3])/2;y=(b[1]+b[4])/2
   for i,band in enumerate(c['pinch']['outerProfileBands']):
    radius=band['proposedOuterRadiusMm'];obstacles.append(dict(name=n,solid=i,role=k,boundsMm=[x-radius,y-radius,g['floorOrigin'][2]+band['zMinMm'],x+radius,y+radius,g['floorOrigin'][2]+band['zMaxMm']],basis='configured source profile band conservative box'))
  elif n.startswith('intake candidate bearing representation'):
   x,y,z=[(b[i]+b[i+3])/2 for i in range(3)];radius=15.5575001;half=3.9687501
   obstacles.append(dict(name=n,solid=0,role=k,boundsMm=[x-radius,y-radius,z-half,x+radius,y+radius,z+half],basis='full native bearing outer-bounds correction; existing unseated center, both flange directions'))
  else:
   e=TopExp_Explorer(s,TopAbs_SOLID);i=0
   while e.More():
    solid=e.Current()
    if not a.h.BRepCheck_Analyzer(solid).IsValid():raise ValueError('invalid solid '+n)
    obstacles.append(dict(name=n,solid=i,role=k,boundsMm=a.assembly.robot.bounds(solid),basis='per-solid conservative AABB of original source-sized CAD'));i+=1;e.Next()
   if not i:raise ValueError('no solid '+n)
 if any(sha(R/p)!=v for p,v in before.items()):raise ValueError('source changed')
 bearing=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['bearing']
 result=dict(schema='conservative-floor-obstacles/1',inputs=before,assemblyPartCount=711,excludedContactParts=excluded,weldmentCoverage=coverage,obstacles=obstacles,bearingSource=bearing,acceptance=False,limits=['Boxes are possible-contact domains, never material contact or clearance certification.','Per-solid boxes preserve disconnected source members but may overbound holes, notches and curved surfaces.','Track bearing outer bounds corrected conservatively at current unseated placements; not hardware fit.','Only designated side belts excluded; service reserve candidates remain explicit.','Flat rigid torus only; no tilted NOTE or deforming clearance proof.'])
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'parts':711,'boxes':len(obstacles),'excluded':excluded}))
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=pathlib.Path);export(p.parse_args().output)
