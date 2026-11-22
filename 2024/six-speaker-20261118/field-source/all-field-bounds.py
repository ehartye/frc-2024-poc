"""Extract installed conservative bounds of every leaf in FIRST's official field."""
import hashlib,json,sys,itertools
from pathlib import Path
import numpy as np
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool,XCAFDoc_ShapeTool
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

root=Path(__file__).parent
src=root/'FE-2024.STEP'
with src.open('rb') as s:sha=hashlib.file_digest(s,'sha256').hexdigest()
assert sha=='42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a'
doc=TDocStd_Document(TCollection_ExtendedString('XCAF'))
r=STEPCAFControl_Reader();r.SetColorMode(False);assert r.ReadFile(str(src))==IFSelect_RetDone and r.Transfer(doc)
tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());free=TDF_LabelSequence();tool.GetFreeShapes(free)
rows=[];corners={};basis=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
def walk(label,loc,path):
    a=TDataStd_Name();name=a.Get().ToExtString() if label.FindAttribute(a.GetID_s(),a) else ''
    p=path+[name];kids=TDF_LabelSequence();XCAFDoc_ShapeTool.GetComponents_s(label,kids)
    if not kids.Length():
        identity=label.EntryDumpToString() if hasattr(label,'EntryDumpToString') else str(label.Tag())+'|'+name
        if identity not in corners:
            box=Bnd_Box();BRepBndLib.AddOptimal_s(XCAFDoc_ShapeTool.GetShape_s(label),box,False,False)
            if box.IsVoid():return
            a=box.CornerMin().Coord();b=box.CornerMax().Coord()
            corners[identity]=np.array(list(itertools.product(*zip(a,b))))
        t=loc.Transformation();m=np.array([[t.Value(i,j) for j in range(1,5)] for i in range(1,4)])
        world=((corners[identity]@m[:,:3].T+m[:,3])@basis.T+[8270.875,4105.275,-4.7752])/1000
        rows.append(dict(path=p,bounds=[world.min(axis=0).tolist(),world.max(axis=0).tolist()]))
    for i in range(1,kids.Length()+1):
        ch=kids.Value(i);src=TDF_Label()
        if XCAFDoc_ShapeTool.GetReferredShape_s(ch,src):walk(src,loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(ch)),p)
for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location(),[])
dst=root/'whole-field-leaf-bounds.json'
with dst.open('x') as f:json.dump(dict(source=sha,leafCount=len(rows),parts=rows),f)
print(len(rows),len(corners),flush=True)
