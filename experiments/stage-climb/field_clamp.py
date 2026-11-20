"""Measure the candidate shared-pose radio box against installed FE-2024 clamp BReps."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.IFSelect import IFSelect_RetDone
from OCP.OCP.collections import Sequence_TDF_Label as Labels
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.gp import gp_Pnt,gp_Trsf
from analyze import HERE, HOOK, load, orientation


def clamps(step):
    doc=TDocStd_Document(TCollection_ExtendedString("XCAF"))
    reader=STEPCAFControl_Reader()
    reader.SetColorMode(False)
    if reader.ReadFile(str(step))!=IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Field STEP read failed")
    free=Labels()
    XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).GetFreeShapes(free)
    builder=BRep_Builder()
    compound=TopoDS_Compound()
    builder.MakeCompound(compound)
    count=0
    def visit(label,location,path):
        nonlocal count
        attr=TDataStd_Name()
        path=path+[attr.Get().ToExtString() if label.FindAttribute(attr.GetID_s(),attr) else ""]
        if len(path)>1 and path[1]!="GE-24200_Blue":
            return
        if path[-1]=="GE-24251_Clampv2":
            builder.Add(compound,XCAFDoc_ShapeTool.GetShape_s(label).Moved(location))
            count+=1
            return
        kids=Labels()
        XCAFDoc_ShapeTool.GetComponents_s(label,kids)
        for i in range(1,kids.Length()+1):
            child=kids.Value(i)
            original=TDF_Label()
            if XCAFDoc_ShapeTool.GetReferredShape_s(child,original):
                visit(original,location.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(child)),path)
    for i in range(1,free.Length()+1): visit(free.Value(i),TopLoc_Location(),[])
    if not count:
        raise ValueError("No registered clamp placements")
    return compound,count


def measure(compound,count,radio,screen):
    # FE-2024 CAD frame to project field frame uses a rigid basis rotation and floor datum.
    basis=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
    datum=np.array([8270.875,4105.275,-4.7752])
    box=BRepPrimAPI_MakeBox(gp_Pnt(*radio[:3]),gp_Pnt(*radio[3:])).Shape()
    rot,_=orientation(np.array(screen['newNominalCenterMm']))
    samples=[]
    for s in screen['stageChecks']:
        offset=s['seatOffsetMm']
        for dy in sorted({offset,-offset}):
            target=np.array([5759.07320361,4105.35014977+dy,s['loadedSeatMm']])
            xyz=basis.T@(target-rot@HOOK-datum)
            tr=gp_Trsf()
            tr.SetValues(*np.column_stack((basis.T@rot,xyz)).ravel())
            placed=BRepBuilderAPI_Transform(box,tr,True).Shape()
            distance=BRepExtrema_DistShapeShape(placed,compound)
            distance.Perform()
            if not distance.IsDone(): raise ValueError("Exact field distance failed")
            samples.append(dict(alongChainMm=dy,minRadioToClampMm=distance.Value()))
    return dict(nativeSha256=screen['nativeSha256'],fieldStepSha256=screen['fieldStepSha256'],
        testedClampOccurrences=count,samples=samples,scope="Conservative R7 radio box versus every installed Blue GE-24251_Clampv2 at the specified terminal poses.")


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('field_step',type=Path)
    parser.add_argument('r7_survey',type=Path)
    args=parser.parse_args()
    screen=load(HERE/'static-screen.json')
    if hashlib.sha256(args.field_step.read_bytes()).hexdigest()!=screen['fieldStepSha256'] or hashlib.sha256(args.r7_survey.read_bytes()).hexdigest()!=screen['r7SurveySha256']:
        raise ValueError("Survey or field identity changed")
    field,count=clamps(args.field_step)
    radio=next(r['bboxMm'] for r in load(args.r7_survey)['rows'] if r['name']=='radio')
    result=measure(field,count,radio,screen)
    (HERE/'clamp-clearance.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8',newline='\n')
