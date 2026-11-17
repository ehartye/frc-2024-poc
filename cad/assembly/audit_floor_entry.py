"""Loose-floor entry geometry into the current fixed pickup pose; not capture dynamics."""
import json,pathlib,hashlib
import build_connected_candidate as a
from audit_cartridge_interfaces import common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE

def pins():
 p=a.input_pins()
 for n in ['cad/assembly/audit_floor_entry.py','cad/assembly/audit_cartridge_interfaces.py']:p[n]=hashlib.sha256((a.ROOT/n).read_bytes()).hexdigest()
 return p

def run():
 before=pins();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());assembly,joints=a.build('floor')
 if len(assembly)!=711 or len({n for n,s,k in assembly})!=711:raise RuntimeError('wrong connected inventory')
 parts=[]
 for n,s,k in assembly:
  if k=='service':continue
  if not a.h.BRepCheck_Analyzer(s).IsValid() or a.h.TopExp_Explorer(s,TopAbs_FACE,TopAbs_SOLID).More():raise RuntimeError('invalid or loose-face input '+n)
  ex=a.h.TopExp_Explorer(s,TopAbs_SOLID);solids=[]
  while ex.More():
   solid=ex.Current()
   if not a.h.BRepCheck_Analyzer(solid).IsValid():raise RuntimeError('invalid solid '+n)
   solids.append((solid,a.assembly.robot.bounds(solid)));ex.Next()
  if not solids:raise RuntimeError('non-solid input '+n)
  parts.append((n,solids))
 rows=[];radius=c['note']['majorRadius']+c['note']['tubeRadius'];tube=c['note']['tubeRadius']
 for offset in [0,-5,5]:
  for advance in range(0,481,10):
   x=c['stageCenterX']+480-advance;z=c['floorCenterZ'];note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(x,offset,z),a.h.gp_Dir(0,0,1)),c['note']['majorRadius'],tube).Shape();bounds=[x-radius,offset-radius,z-tube,x+radius,offset+radius,z+tube];hits=[]
   for n,solids in parts:
    v=0
    for s,q in solids:
     if min(min(bounds[j+3],q[j+3])-max(bounds[j],q[j]) for j in range(3))<=1e-7:continue
     v+=common(s,note)
    if v>1e-5:hits.append({'name':n,'overlapMm3':v,'contactGroup':'pinch' if 'side pinch belt' in n else 'unresolved'})
   rows.append({'lateralOffsetMm':offset,'advanceMm':advance,'centerRobotMm':[x,offset,z],'hits':hits});print(offset,advance,[h['name'] for h in hits if h['contactGroup']=='unresolved'],flush=True)
 if before!=pins():raise RuntimeError('inputs changed during floor study')
 r={'schema':'connected-floor-entry/1','inputs':before,'assemblyPartCount':len(assembly),'rows':rows,'acceptance':False,'limits':['Fixed floor-pose robot and rigid flat ground-tangent NOTE translated inward; not free-body capture simulation','10mm steps and three lateral offsets only, not continuous or manufacturing tolerance clearance','Side-belt occupancy is intended contact hypothesis, not traction, friction, force or retention validation','Other source-sized envelopes may overbound native material; unresolved hits retained','No vehicle motion, ground friction, yaw, pitch, impact, retention during rotation/lift or sensor acquisition modeled']}
 out=a.ROOT/'artifacts/floor-entry';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');return r
if __name__=='__main__':run()
