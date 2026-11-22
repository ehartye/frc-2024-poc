"""Bind a reopened Fusion blade to the pickup trial and to the parked, selected layout."""
import hashlib
import json
import math
import sys
from pathlib import Path
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec


def center(solid):
    prop = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid, prop)
    return prop.CentreOfMass()


def bore_angle(solid):
    faces = TopExp_Explorer(solid, TopAbs_FACE)
    while faces.More():
        as_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face
        surf = BRepAdaptor_Surface(as_face(faces.Current()))
        if surf.GetType() == GeomAbs_Cylinder and abs(surf.Cylinder().Radius() - 1.7) < .00001:
            v = surf.Cylinder().Axis().Direction()
            return math.atan2(v.X(), v.Z())
        faces.Next()
    raise ValueError("Missing 3.4 mm bore")


def align(solid, target, angle):
    rotation = gp_Trsf()
    rotation.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0, 1, 0)), angle)
    rotation.SetTranslationPart(gp_Vec(center(solid).Transformed(rotation), center(target)))
    return rotation, BRepBuilderAPI_Transform(solid, rotation, True).Shape()


if __name__ == "__main__":
    plugin, source, assembly, new, receipt = map(Path, sys.argv[1:])
    sys.path.insert(0, str(plugin / "scripts"))
    from component_shape import measured_difference
    from mechanical_motion import common, distance
    from showcase_contacts import load_solids
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    def only(p):
        solids, _ = load_solids(p)
        assert len(solids) == 1
        return next(iter(solids.values()))
    original, blade = only(source), only(new)
    rows, _ = load_solids(assembly)
    def member(label):
        matches = [v for k, v in rows.items() if label in str(k)]
        assert len(matches) == 1, (label, len(matches))
        return matches[0]
    layout_blade = member("Replaceable pickup lip")
    same = measured_difference(original, blade, [])
    for flip in (0, math.pi):
        matrix, comparison = align(original, layout_blade, bore_angle(layout_blade)-bore_angle(original)+flip)
        placed = measured_difference(comparison, layout_blade, [])
        if "reason" not in placed and max(placed["absentMm3"], placed["extraMm3"]) < .001:
            break
    mounted = BRepBuilderAPI_Transform(blade, matrix, True).Shape()
    offset = gp_Trsf()
    offset.SetTranslation(gp_Vec(0, 2, 0))
    control = measured_difference(original, BRepBuilderAPI_Transform(blade, offset, True).Shape(), [])
    result = dict(reference=sha(source), parkedAssembly=sha(assembly), exportedBlade=sha(new),
                  coordinateSystem="mm, deployed blade to parked assembly", parkedBodies=len(rows),
                  deployedToParked=[[matrix.Value(i,j) for j in range(1,5)] for i in range(1,4)],
                  nativeVsReference=same, referenceVsParked=placed, displaced2mm=control,
                  mountCommonMm3=common(mounted,member("Pickup lip crossbar")),
                  mountGapMm=distance(mounted,member("Pickup lip crossbar")),
                  rollerGapMm=distance(mounted,member("Nose pickup roller")))
    result["pass"] = all("reason" not in row and max(row["absentMm3"],row["extraMm3"]) < .001
                         for row in (same, placed)) and min(control["absentMm3"],control["extraMm3"]) > 1 and "reason" not in control and result["mountCommonMm3"] < .001 and result["mountGapMm"] < .001 and result["rollerGapMm"] > 2
    receipt.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result))
    sys.exit(0 if result["pass"] else 1)
