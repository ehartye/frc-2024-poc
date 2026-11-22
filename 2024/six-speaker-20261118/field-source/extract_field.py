from pathlib import Path
import json, re
import numpy as np
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

root = Path(__file__).parent
out = root/'cad-extraction'
out.mkdir(exist_ok=True)
st = XCAFDoc_ShapeTool
def name(label):
    a=TDataStd_Name()
    return a.Get().ToExtString() if label.FindAttribute(a.GetID_s(),a) else ''
def box(shape):
    b=Bnd_Box()
    BRepBndLib.AddOptimal_s(shape,b,False,False)
    return list(b.CornerMin().Coord())+list(b.CornerMax().Coord()) if not b.IsVoid() else []
def mat(loc):
    t=loc.Transformation()
    return [[t.Value(r,c) for c in range(1,5)] for r in range(1,4)]
doc=TDocStd_Document(TCollection_ExtendedString("XCAF"))
reader=STEPCAFControl_Reader()
reader.SetColorMode(False)
assert reader.ReadFile(str(root/'FE-2024.STEP')) == IFSelect_RetDone
print('loaded',flush=True)
assert reader.Transfer(doc)
print('transferred',flush=True)
tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
free=TDF_LabelSequence()
tool.GetFreeShapes(free)
seen=[]; saved={}; rows=[]
def walk(label,loc,path):
    title=name(label)
    path=path+[title]
    children=TDF_LabelSequence()
    st.GetComponents_s(label,children)
    blue=any('GE-24300' in s and 'blue' in s.lower() for s in path)
    tag='ID' in title and 'Vinyl' in title and re.search(r'ID\d+',title)
    if len(path)<=2: seen.append((title,children.Length(),mat(loc)))
    datum=title in ['FE-2024-01','GE-24500']
    if blue or tag or datum:
        row=dict(path=path,fieldCadFromPart=mat(loc),children=children.Length())
        rows.append(row)
        if children.Length()==0:
            shape=st.GetShape_s(label)
            if title not in saved:
                filename=re.sub(r'[^A-Za-z0-9_.-]','_',title)+'.brep'
                BRepTools.Write_s(shape,str(out/filename))
                saved[title]=filename
            row['brepLocal']=saved[title]
            row['boundsCadMm']=box(shape.Moved(loc))
    for i in range(1,children.Length()+1):
        child=children.Value(i)
        original=TDF_Label()
        if not st.GetReferredShape_s(child, original):continue
        walk(original,loc.Multiplied(st.GetLocation_s(child)),path)
for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location(),[])
(out/'occurrences.json').write_text(json.dumps(rows,indent=1))
(out/'top-level.json').write_text(json.dumps(seen,indent=1))
print('wrote',len(rows),len(saved),flush=True)
