"""Raw complete-cartridge interface audit against all connected assembly parts."""
import json,pathlib,hashlib,argparse,itertools
import build_connected_candidate as a
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_SOLID

def common(s,t):
 op=BRepAlgoAPI_Common(s,t);op.Build()
 if not op.IsDone():raise RuntimeError('Boolean failed')
 shape=op.Shape()
 if shape.IsNull() or not a.h.BRepCheck_Analyzer(shape).IsValid():raise RuntimeError('invalid common shape')
 ex=a.h.TopExp_Explorer(shape,TopAbs_SOLID);v=0
 while ex.More():
  p=GProp_GProps();BRepGProp.VolumeProperties_s(ex.Current(),p);v+=abs(p.Mass());ex.Next()
 return v

def run(out):
 before=a.input_pins();rows=[]
 for state in ['floor','rotated','handoff']:
  parts,joints=a.build(state);bs={n:a.assembly.robot.bounds(s) for n,s,k in parts};hits=[];count=0
  for (n,s,k),(m,t,l) in itertools.combinations(parts,2):
   if not(n.startswith('cartridge ') or m.startswith('cartridge ')) or 'service' in [k,l]:continue
   b,q=bs[n],bs[m]
   if min(min(b[i+3],q[i+3])-max(b[i],q[i]) for i in range(3))<=1e-5:continue
   count+=1;v=common(s,t)
   if v>1e-4:hits.append({'a':n,'b':m,'commonMm3':v})
  rows.append({'state':state,'parts':len(parts),'testedPairs':count,'hits':hits});print(state,len(hits),flush=True)
 if before!=a.input_pins():raise RuntimeError('inputs changed')
 before['cad/assembly/audit_cartridge_interfaces.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
 r={'schema':'connected-cartridge-interface-audit/1','inputs':before,'gapMm':json.loads((a.ROOT/'cad/cycle-contract-A2.json').read_text())['geometry']['shooter']['nominalGap']*1000,'states':rows,'acceptance':False,'limits':['Raw overlaps retained including simplified engagement proxies','Three key poses only; complementary dense mechanism sweep required','No load, tool access, preload or complete mounting acceptance']}
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('output',type=pathlib.Path);run(p.parse_args().output)
