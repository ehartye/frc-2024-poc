"""Finite thread-target envelope audit; not thread strength or fit certification."""
import pathlib,hashlib,json,itertools
import servo_opening_fasteners as f
from audit_cartridge_interfaces import common
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
m=f.base;h=f.h

def run():
 old=json.loads((m.s.a.ROOT/'cad/assembly/evidence/servo-opening-relocation-native.json').read_text());pins=dict(old['inputs'])
 for n in ['servo_opening_fasteners.py','servo-opening-fastener-source.json','check_servo_opening_fasteners.py']:
  p=m.s.a.ROOT/'cad/assembly'/n;pins['cad/assembly/'+n]=hashlib.sha256(p.read_bytes()).hexdigest()
 natives={}
 for name,reg in [('REV-41-1097.STEP',m.register),('REV-41-1363.STEP',m.register_horn)]:
  p=pathlib.Path.home()/'.cache/frc-component-cartridge'/name
  if hashlib.sha256(p.read_bytes()).hexdigest()!=old['nativeSources'][name]['sha256']:raise ValueError('native source changed')
  r=h.STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();natives[name]=reg(r.OneShape())
 rows=[];parts=f.local_parts(0);mutual=[]
 for n,p,k in parts:
  if k=='source-envelope':continue
  for source,q in natives.items():
   v=common(p,q);rows.append({'part':n,'source':source,'rawCommonMm3':v})
 for (n,p,k),(nn,q,kk) in itertools.combinations(parts,2):
  if 'source-envelope' in [k,kk]:continue
  v=common(p,q)
  if v>1e-6:mutual.append({'a':n,'b':nn,'commonMm3':v})
 checks=[]
 for dy,dz in f.OFFSETS:
  target=h.cyl((130.5,196.25+dy,185+dz),(1,0,0),1.5,2.5)
  bolt=next(p for n,p,k in parts if n=='M3x8 horn screw major-envelope '+str(dy))
  op=BRepAlgoAPI_Common(bolt,natives['REV-41-1363.STEP']);op.Build();inter=op.Shape();outside=f.s.cut(inter,target)
  shifted=h.move(bolt,0,.75,0);op2=BRepAlgoAPI_Common(shifted,natives['REV-41-1363.STEP']);op2.Build();bad=f.s.cut(op2.Shape(),target)
  checks.append({'shiftedOutsideTargetMm3':common(bad,bad),'offset':[dy,dz],'rawMm3':common(bolt,natives['REV-41-1363.STEP']),'outsideSourceTargetMm3':common(outside,outside),'shiftedNegativeControlRawMm3':common(h.move(bolt,0,.75,0),natives['REV-41-1363.STEP'])})
 center=next(p for n,p,k in parts if n.startswith('candidate M3x8 center'))
 adapter=next(p for n,p,k in parts if n.startswith('original M3'))
 legacy=next(p for n,p,k in m.local_parts(0) if n=='original horn flange neck eccentric cam')
 controls={'oldAdapterCenterHeadCollisionMm3':common(center,legacy),'newAdapterCenterHeadCollisionMm3':common(center,adapter)}
 for p,sha in pins.items():
  if hashlib.sha256((m.s.a.ROOT/p).read_bytes()).hexdigest()!=sha:raise ValueError('input changed')
 return {'inputs':pins,'nativeSources':old['nativeSources'],'rows':rows,'authoredMutualHits':mutual,'finiteThreadTargets':checks,'negativeControls':controls,'shaftHornCommonMm3':common(*natives.values()),'accepted':False,'limits':['Raw major-envelope overlap in specified tapped region is categorized, NOT a load or thread-fit acceptance','Center screw insertion depth remains candidate; source exact8mm documentation uses V2 horn, not this V1 horn','Shaft/horn overlap remains UNKNOWN','Conservative head envelopes, not source-native fasteners; installation order removes adapter before accessing center screw']}
if __name__=='__main__':
 r=run();p=m.s.a.ROOT/'cad/assembly/evidence/servo-opening-fasteners.json';p.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(r['authoredMutualHits'],r['finiteThreadTargets'],r['negativeControls'],flush=True)
