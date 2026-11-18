"""Original dimension-only drivetrain envelopes. Does not import or distribute vendor CAD."""
import json,math,pathlib,sys,argparse,hashlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from a2_primitives import box,cyl,tube
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.STEPControl import STEPControl_Writer,STEPControl_Reader,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
HERE=pathlib.Path(__file__).resolve().parent

def validate(c):
 if c['units']!='mm' or c['motorCount']!=4:raise ValueError('wrong drivetrain contract')
 for p in c['parts']:
  if len(p['boundsMm'])!=6 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in p['boundsMm']):raise ValueError('nonfinite/non-numeric bounds')
  if any(p['boundsMm'][i+3]<=p['boundsMm'][i] for i in range(3)):raise ValueError('empty envelope')
  if len(p['sourceSha256'])!=64:raise ValueError('missing source binding')
 if len({p['id'] for p in c['parts']})!=len(c['parts']):raise ValueError('duplicate id')

def build(c,services=False):
 validate(c);parts=[]
 for p in c['parts']:
  b=p['boundsMm'];s=p['sizeMm'];center=p['worldCenterMm'];role=p['role']
  if role=='wheel':shape=cyl((center[0],b[1],center[2]),(0,1,0),76.2,s[1])
  elif role in ('shaft','dead-axle','bearing','hub','spacer','standoff'):
   # Cylinder is only a simplified envelope: hex bores/races and mating details omitted.
   shape=cyl((center[0],b[1],center[2]),(0,1,0),max(s[0],s[2])/2,s[1])
  else:shape=box(*b[:3],*s)
  parts.append((p['id'],shape,role))
 for x in c['interfaceProposal']['xCentersMm']:
  parts.append((f'original-crossmember-{x}',tube(x-12.7,-228.6,160,25.4,457.2,50.8,3.175,'y'),'original-interface'))
  for y in [-228.6,228.6]:parts.append((f'original-adapter-{x}-{y}',box(x-25,y-19.05,156,50,38.1,4),'original-interface'))
 if services:
  for p in c['parts']:
   if p['role'] not in ('motor','dead-axle'):continue
   b=p['boundsMm'];sign=1 if p['worldCenterMm'][1]>0 else -1
   delta=(-sign*60) if p['role']=='motor' else sign*120
   lo=min(b[1],b[1]+delta);hi=max(b[4],b[4]+delta)
   parts.append((p['id']+'-service',box(b[0],lo,b[2],b[3]-b[0],hi-lo,b[5]-b[2]),'service-reserve'))
 return parts

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=pathlib.Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 c=json.loads((HERE/'candidate.json').read_text());parts=build(c);compound=TopoDS_Compound();builder=BRep_Builder();builder.MakeCompound(compound)
 for name,s,role in parts:
  assert BRepCheck_Analyzer(s).IsValid(),name;builder.Add(compound,s)
 w=STEPControl_Writer();w.Transfer(compound,STEPControl_AsIs);dest=a.out/'original-drivetrain-envelopes.step';assert w.Write(str(dest))==IFSelect_RetDone
 rd=STEPControl_Reader();assert rd.ReadFile(str(dest))==IFSelect_RetDone;rd.TransferRoots();ex=TopExp_Explorer(rd.OneShape(),TopAbs_SOLID);n=0
 while ex.More():n+=1;ex.Next()
 assert n==len(parts),(n,len(parts))
 receipt=dict(parts=len(parts),reopenedSolids=n,sourcePins={k:v['sha256'] for k,v in c['sources'].items()},inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'candidate.json',pathlib.Path(__file__),HERE.parent/'a2_primitives.py']},outputSha256=hashlib.sha256(dest.read_bytes()).hexdigest(),limits=c['limitations'])
 (a.out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8');print(json.dumps(receipt))
if __name__=='__main__':main()
