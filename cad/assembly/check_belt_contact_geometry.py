"""Deterministic native parity witnesses for the analytical smooth belt material API."""
import pathlib,json,hashlib,random
import build_connected_candidate as a
import belt_contact_geometry as q
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN,TopAbs_OUT

def pins():
 p=a.input_pins()
 for n in ['cad/assembly/belt_contact_geometry.py','cad/assembly/check_belt_contact_geometry.py']:p[n]=hashlib.sha256((a.ROOT/n).read_bytes()).hexdigest()
 return p

def run():
 before=pins();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());g=q.geometry(c);parts={n:s for n,s,k in a.assembly.intake_components.build('floor')};rng=random.Random(2024);checked=0
 for side in [-1,1]:
  belt=parts['intake candidate side pinch belt '+str(side)]
  for _ in range(2000):
   p=[rng.uniform(g['rearX']-g['outerRadius']-5,g['frontX']+g['outerRadius']+5),side*rng.uniform(g['axisYAbs']-g['outerRadius']-5,g['axisYAbs']+g['outerRadius']+5),rng.uniform(-10,10)];d=q.query(p,side,g)['signedDistanceToMaterialMm']
   if abs(d)<1e-6:continue
   w=[p[0]+c['stageCenterX'],p[1],p[2]+c['floorCenterZ']];actual=BRepClass3d_SolidClassifier(belt,a.h.gp_Pnt(*w),1e-7).State();expected=TopAbs_IN if d<0 else TopAbs_OUT
   if actual!=expected:raise RuntimeError('native/material sign mismatch')
   checked+=1
 if before!=pins():raise RuntimeError('inputs changed')
 report={'schema':'analytical-belt-native-parity/1','inputs':before,'seed':2024,'checkedPoints':checked,'mismatches':0,'geometry':g,'limits':['Parity for authored smooth capsule-annulus geometry, not physical teeth or friction','Point distance is not a NOTE force law; cap/edge normals require separate handling','Random finite witnesses supplement explicit endpoint/height/cavity controls']};out=a.ROOT/'artifacts/belt-contact-geometry';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n');print(checked)
if __name__=='__main__':run()
