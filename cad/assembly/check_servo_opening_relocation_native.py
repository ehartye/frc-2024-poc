"""Pinned native-interface checks, including unresolved spline material discrepancy."""
import pathlib,json,hashlib,itertools
import servo_opening_relocation as m
from audit_cartridge_interfaces import common
from OCP.TopAbs import TopAbs_SOLID
SOURCES={'REV-41-1097.STEP':'2ae8de8a60ea26b95021ca04b3a71c2f0a68ec0f3b7cafd27d5b0353f536afb2','REV-41-1363.STEP':'ee4a5cd9bc8cd4f2d926df96f9dd855dd69da0b343d69a2cc7d9a8a6bfee9d2b'}
def run():
 pins=m.s.pins()
 for n in ['servo_opening_mount.py','servo_opening_relocation.py','cassette_opening_cam.py','cassette-opening-source-candidate.json','audit_cartridge_interfaces.py','check_servo_opening_relocation_native.py']:
  p=m.s.a.ROOT/'cad/assembly'/n;pins['cad/assembly/'+n]=hashlib.sha256(p.read_bytes()).hexdigest()
 cache=pathlib.Path.home()/'.cache/frc-component-cartridge';shapes={};solids={};valid={}
 for file,reg in [('REV-41-1097.STEP',m.register),('REV-41-1363.STEP',m.register_horn)]:
  p=cache/file;sha=hashlib.sha256(p.read_bytes()).hexdigest()
  if sha!=SOURCES[file]:raise ValueError('source hash')
  r=m.h.STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();shape=r.OneShape()
  if not m.h.BRepCheck_Analyzer(shape).IsValid():raise ValueError('invalid native')
  shapes[file]=reg(shape);solids[file]=[];ex=m.h.TopExp_Explorer(shape,TopAbs_SOLID)
  while ex.More():
   sh=ex.Current()
   if not m.h.BRepCheck_Analyzer(sh).IsValid():raise ValueError('invalid solid')
   solids[file].append(reg(sh));ex.Next()
  valid[file]={'sha256':sha,'url':'https://www.revrobotics.com/content/cad/'+file,'roots':r.NbRootsForTransfer(),'solids':len(solids[file]),'valid':True}
 rows=[];parts=m.local_parts(0)
 for file,ss in solids.items():
  for n,p,k in parts:
   if k!='source-envelope':rows.append({'native':file,'part':n,'commonMm3PerSolid':[common(p,q) for q in ss]})
 mutual=[]
 for (n,p,k),(nn,q,kk) in itertools.combinations(parts,2):
  if 'source-envelope' in [k,kk]:continue
  v=common(p,q)
  if v>1e-6:mutual.append({'a':n,'b':nn,'commonMm3':v})
 native=[common(p,shapes['REV-41-1363.STEP']) for p in solids['REV-41-1097.STEP']]
 for p,sha in pins.items():
  if hashlib.sha256((m.s.a.ROOT/p).read_bytes()).hexdigest()!=sha:raise ValueError('source changed')
 return {'inputs':pins,'nativeSources':valid,'rows':rows,'nativeHornServoCommonMm3PerSolid':native,'authoredMutualHits':mutual,'accepted':False,'limits':['PositiveY only; theta0 native interface sample','Supplier horn spline bore simplified: native shaft overlap is UNKNOWN, not accepted engagement','Mounting hardware heads/nuts and weld strength remain unqualified','No source CAD redistributed']}
if __name__=='__main__':
 r=run();p=m.s.a.ROOT/'cad/assembly/evidence/servo-opening-relocation-native.json';p.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(r['nativeHornServoCommonMm3PerSolid'],r['authoredMutualHits'],flush=True)
