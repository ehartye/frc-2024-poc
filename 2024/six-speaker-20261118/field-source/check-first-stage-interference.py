import hashlib,json,math,sys
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt,gp_Trsf
# Reuse the registered field reader, without running its occurrence export.
root=Path(__file__).parent
reader_code=(root/'extract_field.py').read_text().split('seen=[]; saved={}; rows=[]')[0]
exec(compile(reader_code,'extract_field.py','exec'))
work=root.parent
inv=json.loads((work.parent/'track-tread-candidate-20261118/route-relieved/scan.json').read_bytes())
part=next(x for x in inv['inventory'] if x['name']=='fixed bumper end backing 1')
lo,hi=np.array(part['boundsMm']).reshape(2,3)
assert abs(np.prod(hi-lo)-part['volumeMm3'])<1e-6
boxbody=BRepPrimAPI_MakeBox(gp_Pnt(*lo),gp_Pnt(*hi)).Shape()
src=work.parent/'turn-lift-20261118/end-to-end-fine-left/ground/receipt.json'
r=json.loads(src.read_bytes());sample=min(r['trace'],key=lambda r:abs(r['chassis']['x']-.25))
base=sample['chassis'];ang=base['heading']+math.pi/2;c=math.cos(ang);s=math.sin(ang)
basis=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
matrix=basis.T@np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
pos=[114*.0254-base['y'],161.625*.0254+base['x']-.658,0.]
translate=basis.T@(1000*np.array(pos)-[8270.875,4105.275,-4.7752])
transform=gp_Trsf();transform.SetValues(*np.column_stack([matrix,translate]).ravel())
placed=BRepBuilderAPI_Transform(boxbody,transform,True).Shape()
hits=[]
def walk(label,loc,path):
    title=name(label);p=path+[title]
    if len(p)>1 and p[1]!='GE-24200_Blue':return
    children=TDF_LabelSequence();st.GetComponents_s(label,children)
    if not children.Length() and title in ('GE-24207_Blue','FE-01003-01'):
        shape=st.GetShape_s(label).Moved(loc)
        common=BRepAlgoAPI_Common(placed,shape);common.Build();assert common.IsDone()
        props=GProp_GProps();BRepGProp.VolumeProperties_s(common.Shape(),props);v=props.Mass()
        hits.append(dict(path=p,volumeMm3=v))
    for i in range(1,children.Length()+1):
        ch=children.Value(i);original=TDF_Label()
        if st.GetReferredShape_s(ch,original):walk(original,loc.Multiplied(st.GetLocation_s(ch)),p)
for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location(),[])
result=dict(sourceNative=inv['sourceSha256'],sourceGround=hashlib.sha256(src.read_bytes()).hexdigest(),body=part['key'],
    installedBoxVolumeMm3=part['volumeMm3'],clock=sample['timeS'],chassisFieldXY=pos[:2],hits=hits)
(work/'inner-wing-stage-interference.json').write_text(json.dumps(result,indent=2))
print([(x['path'][-1],round(x['volumeMm3'],2)) for x in hits if x['volumeMm3']>.001])
