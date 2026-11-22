from pathlib import Path
import numpy as np,sys,json,hashlib
p=Path(__file__).resolve().parent;root=p.parent
t=(root/'mission-physics/fixtures.py').read_text();exec(t[:t.index('packs={}')]);from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepTools import BRepTools
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.TDF import TDF_Label
st=XCAFDoc_ShapeTool
def walk(label,loc,path):
    attr=TDataStd_Name();name=attr.Get().ToExtString() if label.FindAttribute(attr.GetID_s(),attr) else ''
    path=path+[name]
    if len(path)>1 and not 'GE-24200_Blue' in path:return
    if name=='GE-24205_hanging':
        native=st.GetShape_s(label).Moved(loc)
        tr=gp_Trsf();tr.SetValues(-1,0,0,8270.875,0,0,1,4105.275,0,1,0,-4.7752)
        shape=BRepBuilderAPI_Transform(native,tr,True).Shape();b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b,False,False);bb=tuple(v for pt in (b.CornerMin(),b.CornerMax()) for v in (pt.X(),pt.Y(),pt.Z()))
        if bb[3]-bb[0]<100:
            BRepTools.Write_s(shape,str(p/'working-chain.brep'));(p/'source.json').write_text(json.dumps(dict(field=expected,path=path,bounds=bb,brep=sha(p/'working-chain.brep'))));print('extracted',bb)
        return
    children=TDF_LabelSequence();st.GetComponents_s(label,children)
    for i in range(1,children.Length()+1):
        child=children.Value(i);orig=TDF_Label()
        if st.GetReferredShape_s(child,orig):walk(orig,loc.Multiplied(st.GetLocation_s(child)),path)
for j in range(1,free.Length()+1):walk(free.Value(j),TopLoc_Location(),[])


