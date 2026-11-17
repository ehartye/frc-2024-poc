"""Whole rigid NOTE vs changed bearing/carrier material, including actual free final pose."""
import json,math,pathlib,hashlib,argparse
import seated_bearing_assembly as candidate
from bearing_seat_candidate import flip
from audit_cartridge_interfaces import common
from audit_intake_feed import possible_torus_box
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
A=candidate.a;h=A.h
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=candidate.pins()
 for f in ['cad/assembly/check_seated_bearing_note.py','cad/assembly/audit_cartridge_interfaces.py','cad/assembly/audit_intake_feed.py','cad/components/a2-wheel-cartridge.json']:p[f]=sha(A.ROOT/f)
 return p

def changed(c,deg,z,native):
 parts=candidate.replacements(c,deg,z);p=c['pinch'];r=p['teeth']*p['pitch']/(2*math.pi);track=p['innerFaceYAbs']+r+1;span=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2
 for side in [-1,1]:
  for x in [p['beltCenterX']-span/2,p['beltCenterX']+span/2]:
   for zz in [p['lowerBearingZ'],p['upperBearingZ']]:
    low=zz==p['lowerBearingZ'];s=h.move(flip(native) if low else native,x,side*track,zz+(-.38125 if low else -3.96875));name='intake candidate bearing representation '+str((side,x))+str(zz);parts[name]=h.pitch(s,math.radians(deg),[c['stageCenterX'],0,z])
 return parts

def run(cache,runtime):
 before=pins();source=json.loads((A.ROOT/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['bearing'];sp=cache/'REV-21-1916-PK4.step'
 if sha(sp)!=source['sha256']:raise ValueError('source hash')
 rd=h.STEPControl_Reader()
 if rd.ReadFile(str(sp))!=h.IFSelect_RetDone:raise ValueError('read')
 rd.TransferRoots();native=rd.OneShape();ex=h.TopExp_Explorer(native,TopAbs_SOLID);count=0
 while ex.More():
  if not h.BRepCheck_Analyzer(ex.Current()).IsValid():raise ValueError('invalid')
  count+=1;ex.Next()
 if count!=1 or h.TopExp_Explorer(native,TopAbs_FACE,TopAbs_SOLID).More():raise ValueError('topology')
 c=json.loads((A.ROOT/'cad/assembly/intake_components.json').read_text());R=c['note']['majorRadius'];r=c['note']['tubeRadius'];floor=changed(c,0,c['floorCenterZ'],native);bounds={n:A.assembly.robot.bounds(s) for n,s in floor.items()};rows=[]
 containment=[];proxies=candidate.replacements(c,0,c['floorCenterZ'])
 for name,shape in floor.items():
  if 'bearing representation' not in name:continue
  op=BRepAlgoAPI_Cut(shape,proxies[name])
  if not op.IsDone():raise ValueError('containment Boolean failed')
  ex=h.TopExp_Explorer(op.Shape(),TopAbs_SOLID);outside=0;solids=0
  while ex.More():
   if not h.BRepCheck_Analyzer(ex.Current()).IsValid():raise ValueError('invalid containment remainder')
   g=GProp_GProps();BRepGProp.VolumeProperties_s(ex.Current(),g);outside+=abs(g.Mass());solids+=1;ex.Next()
  containment.append({'part':name,'nativeOutsideProxyMm3':outside,'remainingSolids':solids})
 for dy in [0,-5,5]:
  for advance in range(0,481,10):
   x=c['stageCenterX']+480-advance;z=c['floorCenterZ'];note=BRepPrimAPI_MakeTorus(h.gp_Ax2(h.gp_Pnt(x,dy,z),h.gp_Dir(0,0,1)),R,r).Shape();hits=[]
   for n,s in floor.items():
    b=bounds[n];rel=[b[0]-x,b[1]-dy,b[2]-z,b[3]-x,b[4]-dy,b[5]-z]
    if not possible_torus_box(rel,0,R,r):continue
    v=common(note,s)
    if v>1e-5:hits.append({'part':n,'commonMm3':v})
   rows.append({'lateralOffsetMm':dy,'advanceMm':advance,'hits':hits})
  print('floor',dy,'hitRows',sum(bool(q['hits']) for q in rows if q['lateralOffsetMm']==dy),flush=True)
 rp=runtime/'sim/retention/free-handoff-evidence.json';runtimehash=sha(rp);report=json.loads(rp.read_text())
 if any(sha(runtime/k)!=v for k,v in report['inputs'].items()):raise ValueError('stale actual free pose')
 state=report['state'];center=report['centerRobotMm'];phi=state['notePitchRad'];note=BRepPrimAPI_MakeTorus(h.gp_Ax2(h.gp_Pnt(*center),h.gp_Dir(-math.sin(phi),0,math.cos(phi))),R,r).Shape();finalhits=[]
 for n,s in changed(c,c['pitchDeg'],c['handoffCenterZ'],native).items():
  v=common(note,s)
  if v>1e-5:finalhits.append({'part':n,'commonMm3':v})
 if before!=pins() or sha(sp)!=source['sha256'] or sha(rp)!=runtimehash or any(sha(runtime/k)!=v for k,v in report['inputs'].items()):raise ValueError('inputs changed')
 out=A.ROOT/'artifacts/seated-bearing-note';out.mkdir(parents=True,exist_ok=True);q={'schema':'seated-bearing-whole-note-delta/1','inputs':before,'nativeSource':source,'nativeMaterialContainment':containment,'rows':rows,'actualFreeFinal':{'evidenceSha256':runtimehash,'inputs':report['inputs'],'state':state,'centerRobotMm':center,'hits':finalhits},'adopted':False,'acceptance':False,'limits':['Full rigid torus tested against ten changed parts only; unchanged assembly still requires combined adoption gate','Native nominal bearing replaces original stepped proxy for these witnesses; carrier is authored original material','147 fixed-offset floor poses, not free lateral motion or tolerance evidence','Centered reverse local trajectory also represents aligned handoff feed against these cradle-attached parts under rigid transform','Actual final free pose preserved with no attitude/position snap, but no subsequent free transfer trajectory','No force, retention, shaft end hardware, carpet or fabrication qualification']};(out/'report.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');print('free final',finalhits,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);p.add_argument('--runtime',type=pathlib.Path,required=True);args=p.parse_args();run(args.vendor_cache,args.runtime)
