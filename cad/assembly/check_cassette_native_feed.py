"""Native positive-side cassette metal screen over full centered relative feed."""
import json,math,pathlib,hashlib
import spring_cassette_candidate as candidate
from audit_cartridge_interfaces import common
from audit_intake_feed import possible_torus_box
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
A=candidate.a;h=A.h
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(cache):
 before=candidate.pins()
 for f in ['cad/assembly/check_cassette_native_feed.py','cad/assembly/audit_cartridge_interfaces.py','cad/assembly/audit_intake_feed.py']:before[f]=sha(A.ROOT/f)
 sources={'pulley':{'file':'REV-21-2201.step','sha256':'c2825524b2293af12647d92ad57e9162379ad64105b86f867be2223037059c68','solids':3},'bearing':{'file':'REV-21-1916-PK4.step','sha256':'3e16cb3f5f215288b8706eb8228d9580fea78c425eee536ef95195b5b98f02e6','solids':1}};native={}
 for name,record in sources.items():
  p=cache/record['file']
  if sha(p)!=record['sha256']:raise ValueError('source hash')
  rd=h.STEPControl_Reader()
  if rd.ReadFile(str(p))!=h.IFSelect_RetDone:raise ValueError('read')
  rd.TransferRoots();s=rd.OneShape();ex=h.TopExp_Explorer(s,TopAbs_SOLID);ss=[]
  while ex.More():
   if not h.BRepCheck_Analyzer(ex.Current()).IsValid():raise ValueError('invalid native')
   ss.append(ex.Current());ex.Next()
  if len(ss)!=record['solids'] or h.TopExp_Explorer(s,TopAbs_FACE,TopAbs_SOLID).More():raise ValueError('topology')
  native[name]=ss
 c=json.loads((A.ROOT/'cad/assembly/intake_components.json').read_text());p=c['pinch'];span=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;track=p['innerFaceYAbs']+p['teeth']*p['pitch']/(2*math.pi)+1;R=c['note']['majorRadius'];r=c['note']['tubeRadius'];rows=[]
 for travel in [0,.75,1.5]:
  placed=[]
  for x in [p['beltCenterX']-span/2,p['beltCenterX']+span/2]:
   for j,s in enumerate(native['pulley']):
    w=h.move(s,c['stageCenterX']+x,track-travel,c['floorCenterZ']);placed.append((f'pulley/{x}/{j}',w,A.assembly.robot.bounds(w)))
   for z in [p['lowerBearingZ'],p['upperBearingZ']]:
    for flip in [False,True]:
     s=native['bearing'][0]
     if flip:
      t=h.gp_Trsf();t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(1,0,0)),math.pi);s=h.BRepBuilderAPI_Transform(s,t,True).Shape()
     w=h.move(s,c['stageCenterX']+x,track-travel,c['floorCenterZ']+z);placed.append((f'bearing/{x}/{z}/{flip}',w,A.assembly.robot.bounds(w)))
  for offset in sorted(set(list(range(0,481,10))+[10.5674400200612])):
   cx=c['stageCenterX']+offset;cz=c['floorCenterZ'];note=BRepPrimAPI_MakeTorus(h.gp_Ax2(h.gp_Pnt(cx,0,cz),h.gp_Dir(0,0,1)),R,r).Shape();hits=[]
   for name,s,b in placed:
    q=[b[0]-cx,b[1],b[2]-cz,b[3]-cx,b[4],b[5]-cz]
    if not possible_torus_box(q,0,R,r):continue
    v=common(s,note)
    if v>1e-5:hits.append({'part':name,'commonMm3':v})
   rows.append({'inwardTravelMm':travel,'noteCradleXmm':offset,'hits':hits})
  print(travel,sum(bool(q['hits']) for q in rows if q['inwardTravelMm']==travel),flush=True)
 if any(sha(A.ROOT/f)!=v for f,v in before.items()) or any(sha(cache/s['file'])!=s['sha256'] for s in sources.values()):raise ValueError('inputs changed')
 out=A.ROOT/'artifacts/spring-cassette';q={'schema':'cassette-native-relative-feed/1','inputs':before,'sources':sources,'rows':rows,'acceptance':False,'limits':['Positive-side cassette only, nominal source pulley clocking; full rotor-phase variation not screened','Full centered local0..480mm path at10mm samples plus indexed10.567440mm hold offset; not a continuous or lateral-tolerance guarantee','Native bearing at inherited unresolved centers, both flange orientations; seated94ab67d not adopted','Rigid foam/metal occupancy is geometric contact, not safety verdict or calibrated force','Travel cannot be treated usable through feed without staged opening/contact-force model','PriorÂ±5mm fixed-offset counterexamples retained separately, not erased']};(out/'native-feed.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':
 import argparse;p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);run(p.parse_args().vendor_cache)
