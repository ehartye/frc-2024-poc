"""Raw moving/moving common volumes; contact classifications do not waive collisions."""
import pathlib,json,hashlib,itertools
import build_intake_candidate as assembly
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def contact_kind(p,q):
 names=(p['name'],q['name'])
 if p['motion']!=q['motion']:return 'unresolved-cross-motion-intersection'
 pairs=[('side pinch belt','12T track pulley'),('pinch transfer belt','pinch transfer pulley'),('rotation chain run','rotation16T sprocket')]
 if any(any(a in n for n in names) and any(b in n for n in names) for a,b in pairs):
  return 'nominal-drive-envelope-engagement-unvalidated'
 return 'unresolved-solid-intersection'

def run():
 before=assembly.input_pins();rows=[]
 for state in ['floor','rotated','handoff']:
  parts=assembly.intake_components.build(state,metadata=True);moving=[p for p in parts if p['motion']!='fixed'];bounds={p['id']:assembly.robot.bounds(p['shape']) for p in moving};hits=[]
  for p,q in itertools.combinations(moving,2):
   a=bounds[p['id']];b=bounds[q['id']]
   if min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))<=1e-5:continue
   props=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(p['shape'],q['shape']).Shape(),props)
   if props.Mass()>1e-3:hits.append({'a':p['id'],'aName':p['name'],'aMotion':p['motion'],'b':q['id'],'bName':q['name'],'bMotion':q['motion'],'commonMm3':props.Mass(),'classification':contact_kind(p,q)})
  rows.append({'state':state,'movingParts':len(moving),'hits':hits})
 if before!=assembly.input_pins():raise RuntimeError('source changed during internal audit')
 before['cad/assembly/audit_internal_motion.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
 return {'schema':'intake-internal-moving-audit/1','inputs':before,'states':rows,'acceptance':False,'classificationDoesNotWaiveRawIntersections':True,'limits':['All overlaps retained; source envelopes may include intentional engagement or omitted bores','Three key poses not continuous sweep','Requires connection/contact classification and physical interface completion']}
if __name__=='__main__':
 q=run();p=assembly.ROOT/'artifacts/intake-component-candidate/internal-motion.json';p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'states':[(r['state'],len(r['hits'])) for r in q['states']]}))
