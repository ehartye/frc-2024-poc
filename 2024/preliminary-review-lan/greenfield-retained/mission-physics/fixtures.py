"""Display meshes in blue field metres, from project-pinned FIRST STEP placements."""
from pathlib import Path
import base64,hashlib,json,sys
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool,XCAFDoc_ShapeTool
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
out=Path(__file__).resolve().parent
root=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
src=root/'FE-2024.STEP';expected='42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a';assert sha(src)==expected
doc=TDocStd_Document(TCollection_ExtendedString('XCAF'));reader=STEPCAFControl_Reader();reader.SetColorMode(False)
assert reader.ReadFile(str(src))==IFSelect_RetDone and reader.Transfer(doc)
free=TDF_LabelSequence();XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).GetFreeShapes(free)
packs={};counts={};memo={}
def push(group,v,f):
    vs,fs,offset=packs.setdefault(group,[[],[],0]);vs.append(v);fs.append(f+offset);packs[group][2]+=len(v)
def visit(label,loc,path):
    attr=TDataStd_Name();name=attr.Get().ToExtString() if False else (attr.Get().ToExtString() if label.FindAttribute(attr.GetID_s(),attr) else '')
    path=path+[name]
    if len(path)>1 and path[1] not in ('GE-24200_Blue','GE-24300_Briweld Field Blue'):return
    kids=TDF_LabelSequence();XCAFDoc_ShapeTool.GetComponents_s(label,kids)
    if kids.Length():
        for j in range(1,kids.Length()+1):
            ch=kids.Value(j);ref=TDF_Label()
            if XCAFDoc_ShapeTool.GetReferredShape_s(ch,ref):visit(ref,loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(ch)),path)
        return
    if len(path)<2:return
    speaker='24300' in path[1]
    if not (name.startswith(('GE-','FE-','Truss','Connection Plate','Hook_4')) or name=='42SMR' or 'Truss' in name or 'Grip_Wheel' in name):return
    if name not in memo:
        sh=XCAFDoc_ShapeTool.GetShape_s(label);mesh=BRepMesh_IncrementalMesh(sh,1.,False,.27,True);exp=TopExp_Explorer(sh,TopAbs_FACE);v=[];f=[]
        while exp.More():
            face=TopoDS.Face(exp.Current());l=TopLoc_Location();t=BRep_Tool.Triangulation_s(face,l)
            if t:
                off=len(v);v.extend(t.Node(j).Transformed(l.Transformation()).Coord() for j in range(1,t.NbNodes()+1))
                ff=np.array([t.Triangle(j).Get() for j in range(1,t.NbTriangles()+1)],dtype=int)-1+off
                if face.Orientation()==TopAbs_REVERSED:ff=ff[:,::-1]
                f.extend(ff)
            exp.Next()
        if not v:return
        vv,ii=np.unique(np.round(v,4),axis=0,return_inverse=True);memo[name]=vv,ii[np.array(f)]
    v,f=memo[name];tr=loc.Transformation();m=np.array([[tr.Value(i,j) for j in range(1,5)] for i in range(1,4)]);w=v@m[:,:3].T+m[:,3]
    v=np.column_stack([8270.875-w[:,0],4105.275+w[:,2],w[:,1]-4.7752])/1000
    cl='speakerPanel' if speaker else 'stageSteel'
    if speaker:
        if name=='42SMR':cl='sensors'
        elif name.startswith(('GE-24307','GE-24308','GE-24309','GE-24355','GE-24391','GE-24392')):cl='speakerRamp'
    elif name=='GE-24205_hanging':
        cl='workingChain' if np.ptp(v,axis=0)[0]<.1 else 'otherChain'
    elif 'Clamp' in name or 'Hook_4' in name:cl='anchors'
    counts[cl]=counts.get(cl,0)+1;push(cl,v,f)
for j in range(1,free.Length()+1):visit(free.Value(j),TopLoc_Location(),[])
encoded={}
for k,(v,f,_) in packs.items():
    V=np.concatenate(v).astype('<f4');F=np.concatenate(f).astype('<u4');encoded[k]=dict(v=base64.b64encode(V).decode(),f=base64.b64encode(F).decode(),n=len(F),bounds=[V.min(0).tolist(),V.max(0).tolist()])
(out/'fixtures.json').write_text(json.dumps(encoded,separators=(',',':')))
(out/'fixture-lineage.json').write_text(json.dumps(dict(fieldStep=expected,occurrences=counts,triangles={k:r['n'] for k,r in encoded.items()},scope='Named blue field structure and target hardware; selected minor fixtures and hardware omitted from display. Field contact uses the original separate physics surfaces.'),indent=2))
print(counts,flush=True)



