"""Private exact pulley/bearing floor-entry witnesses, including flange orientation uncertainty."""
import json,math,pathlib,hashlib,argparse
import build_connected_candidate as a
from audit_cartridge_interfaces import common
from audit_intake_feed import possible_torus_box
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=a.input_pins()
 for n in ['cad/assembly/check_native_floor_entry.py','cad/assembly/audit_cartridge_interfaces.py','cad/assembly/evidence/floor-entry.json','cad/components/a2-wheel-cartridge.json','cad/assembly/audit_intake_feed.py']:p[n]=sha(a.ROOT/n)
 return p

def run(cache):
 before=pins();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());source_bearing=json.loads((a.ROOT/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['bearing'];sources={'pulley':{'file':'REV-21-2201.step','sha256':'c2825524b2293af12647d92ad57e9162379ad64105b86f867be2223037059c68','expectedSolids':3},'bearing':dict(source_bearing,file='REV-21-1916-PK4.step',expectedSolids=1)};native={}
 for name,record in sources.items():
  path=cache/record['file']
  if sha(path)!=record['sha256']:raise RuntimeError('source hash mismatch')
  rd=a.h.STEPControl_Reader()
  if rd.ReadFile(str(path))!=a.h.IFSelect_RetDone:raise RuntimeError('import failed')
  rd.TransferRoots();shape=rd.OneShape();ex=a.h.TopExp_Explorer(shape,TopAbs_SOLID);solids=[]
  while ex.More():
   s=ex.Current()
   if not a.h.BRepCheck_Analyzer(s).IsValid():raise RuntimeError('invalid solid')
   solids.append(s);ex.Next()
  if len(solids)!=record['expectedSolids'] or a.h.TopExp_Explorer(shape,TopAbs_FACE,TopAbs_SOLID).More():raise RuntimeError('source topology mismatch')
  native[name]=solids
 p=c['pinch'];radius=p['teeth']*p['pitch']/(2*math.pi);yabs=p['innerFaceYAbs']+radius+1;center=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;placed=[]
 for side in [-1,1]:
  for x in [p['beltCenterX']-center/2,p['beltCenterX']+center/2]:
   for idx,s in enumerate(native['pulley']):
    w=a.h.move(s,c['stageCenterX']+x,side*yabs,c['floorCenterZ']);placed.append((f'pulley/{side}/{x}/{idx}',w,a.assembly.robot.bounds(w)))
   for height in [p['lowerBearingZ'],p['upperBearingZ']]:
    for flip in [False,True]:
     s=native['bearing'][0]
     if flip:
      tr=a.h.gp_Trsf();tr.SetRotation(a.h.gp_Ax1(a.h.gp_Pnt(0,0,0),a.h.gp_Dir(1,0,0)),math.pi);s=a.h.BRepBuilderAPI_Transform(s,tr,True).Shape()
     w=a.h.move(s,c['stageCenterX']+x,side*yabs,c['floorCenterZ']+height);placed.append((f'bearing/{side}/{x}/{height}/flip{flip}',w,a.assembly.robot.bounds(w)))
 rows=[];R=c['note']['majorRadius'];r=c['note']['tubeRadius'];outer=R+r
 floor=json.loads((a.ROOT/'cad/assembly/evidence/floor-entry.json').read_text())
 if any(sha(a.ROOT/name)!=digest for name,digest in floor['inputs'].items()):raise RuntimeError('stale floor entry receipt')
 if [(r['lateralOffsetMm'],r['advanceMm']) for r in floor['rows']]!=[(y,x) for y in [0,-5,5] for x in range(0,481,10)]:raise RuntimeError('wrong domain')
 for row in floor['rows']:
  x,y,z=row['centerRobotMm'];note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(x,y,z),a.h.gp_Dir(0,0,1)),R,r).Shape();b=[x-outer,y-outer,z-r,x+outer,y+outer,z+r];hits=[]
  for n,s,q in placed:
   relative=[q[0]-x,q[1]-y,q[2]-z,q[3]-x,q[4]-y,q[5]-z]
   if not possible_torus_box(relative,0,R,r):continue
   if min(min(b[j+3],q[j+3])-max(b[j],q[j]) for j in range(3))<=1e-7:continue
   v=common(s,note)
   if v>1e-5:hits.append({'part':n,'commonMm3':v})
  rows.append({'lateralOffsetMm':row['lateralOffsetMm'],'advanceMm':row['advanceMm'],'centerRobotMm':row['centerRobotMm'],'equivalentCradleLocalGuideAdvanceMm':x-c['stageCenterX'],'hits':hits});print(row['lateralOffsetMm'],row['advanceMm'],len(hits),flush=True)
 if before!=pins() or any(sha(cache/r['file'])!=r['sha256'] for r in sources.values()):raise RuntimeError('inputs changed')
 report={'schema':'native-floor-entry-witness/1','inputs':before,'sources':sources,'rows':rows,'acceptance':False,'limits':['Private native source material, scalar results only; supplier geometry not redistributed','Both bearing flange orientations screened because current intake envelope does not select one','Native flanged bearing is larger than current28.575mm by6.35mm cylindrical envelope; interface adoption requires separate carrier/mount checks','Rigid prescribed paths do not model free lateral recentering or foam deformation','Finite147positions and nominal source clocking; no tolerance, force or pickup acceptance','Centered local floor480-to0 trajectory also covers reversed local guide0-to480 contacts for these cradle-mounted parts under rigid pitch transform; not whole chassis clearance']}
 out=a.ROOT/'artifacts/native-floor-entry';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'rows':len(rows),'hitRows':sum(bool(r['hits']) for r in rows)}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);run(p.parse_args().vendor_cache)
