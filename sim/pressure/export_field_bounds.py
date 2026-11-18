"""Extract installed bounds from the privately supplied 2024 complete-field STEP."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.OCP.collections import Sequence_TDF_Label
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_Tool
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.gp import gp_Pnt, gp_Trsf

FIELD_SHA256 = "42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a"
CAD_TO_FIELD = np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
CARPET_ORIGIN_MM = np.array([8270.875,4105.275,-4.7752])


def bounds(shape, tolerance=False):
    box=Bnd_Box()
    BRepBndLib.AddOptimal_s(shape,box,False,tolerance)
    if not box.IsVoid():
        return np.array([box.CornerMin().Coord(),box.CornerMax().Coord()])


def volume(shape):
    props=GProp_GProps()
    BRepGProp.VolumeProperties_s(shape,props,1e-10)
    return props.Mass()


def sliced_bounds(shape, matrix):
    transform=gp_Trsf()
    transform.SetValues(*matrix.ravel())
    placed=BRepBuilderAPI_Transform(shape,transform,True).Shape()
    lo,hi=bounds(placed,True)
    edges=np.linspace(lo[0]-1.,hi[0]+1.,max(1,math.ceil((hi[0]-lo[0])/25.))+1)
    cells=[]
    total=0.
    for x,y in zip(edges,edges[1:]):
        slab=BRepPrimAPI_MakeBox(gp_Pnt(x,lo[1]-1,lo[2]-1),gp_Pnt(y,hi[1]+1,hi[2]+1)).Shape()
        common=BRepAlgoAPI_Common(placed,slab)
        common.Build()
        if not common.IsDone():
            raise ValueError("Failed to intersect field solid with its slab")
        clipped=common.Shape()
        box=bounds(clipped,True)
        if box is not None:
            cells.append(((box+np.array([[-.002],[.002]]))/1000.).tolist())
            total+=volume(clipped)
    original=volume(placed)
    if not cells or abs(total-original)>max(.001,abs(original)*1e-6):
        raise ValueError("Slabs failed to recover installed field solid volume")
    return cells


def export(source, split_parts):
    with source.open("rb") as stream:
        sha=hashlib.file_digest(stream,"sha256").hexdigest()
    if sha!=FIELD_SHA256:
        raise ValueError("Expected the pinned complete-field 2024 STEP")
    doc=TDocStd_Document(TCollection_ExtendedString("XCAF"))
    reader=STEPCAFControl_Reader()
    reader.SetColorMode(False)
    if reader.ReadFile(str(source))!=IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Field STEP import failed")
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots=Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    parts=[]
    corners={}
    def walk(label,location,parents):
        attribute=TDataStd_Name()
        name=attribute.Get().ToExtString() if label.FindAttribute(attribute.GetID_s(),attribute) else ""
        path=[*parents,name]
        children=Sequence_TDF_Label()
        XCAFDoc_ShapeTool.GetComponents_s(label,children)
        if children.Length():
            for j in range(1,children.Length()+1):
                instance=children.Value(j)
                ref=TDF_Label()
                if XCAFDoc_ShapeTool.GetReferredShape_s(instance,ref):
                    walk(ref,location.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(instance)),path)
            return
        key=TCollection_AsciiString()
        TDF_Tool.Entry_s(label,key)
        identity=key.ToCString()
        shape=XCAFDoc_ShapeTool.GetShape_s(label)
        if identity not in corners:
            raw=bounds(shape)
            if raw is None:
                return
            corners[identity]=np.array(list(itertools.product(*zip(*raw))))
        tr=location.Transformation()
        matrix=CAD_TO_FIELD@np.array([[tr.Value(r,c) for c in range(1,5)] for r in range(1,4)])
        matrix[:,3]+=CARPET_ORIGIN_MM
        world=(corners[identity]@matrix[:,:3].T+matrix[:,3])/1000.
        item=dict(path=path,bounds=[world.min(axis=0).tolist(),world.max(axis=0).tolist()])
        if name in split_parts and "GE-24330_Blue" in path:
            item["tightBounds"]=sliced_bounds(shape,matrix)
        parts.append(item)
    for i in range(1,roots.Length()+1):
        walk(roots.Value(i),TopLoc_Location(),[])
    if len(parts)!=5507:
        raise ValueError("Unexpected field occurrence count")
    return dict(source=sha,leafCount=len(parts),parts=parts)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    data=export(args.source,{"GE-24333_Blue","GE-24334","GE-24339","GE-24341","GE-24343-Vinyl"})
    with args.output.open("x") as stream:
        json.dump(data,stream,allow_nan=False)
