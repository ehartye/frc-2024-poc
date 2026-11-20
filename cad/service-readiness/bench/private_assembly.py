"""Assemble restricted purchased geometry in an owner-supplied local destination."""

import argparse
import math
from pathlib import Path
from OCP.BRep import BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS_Compound
from OCP.gp import gp_Pnt, gp_Vec
from measure import read_step, transform, cylinder, place, write_step, volume
from controller_mount_review import explore
from layout import CENTERS, CLINCH_HOLES


def hardware():
    # Press-in and thread teeth are external envelopes. The hex recess is real.
    nut = BRepAlgoAPI_Fuse(cylinder(0, 0, -4, 2, 3.935), cylinder(0, 0, -2, 1.38, 2.69)).Shape()
    nut = BRepAlgoAPI_Cut(nut, cylinder(0, 0, -4.01, 3.4, 1.65)).Shape()
    bolt = BRepAlgoAPI_Fuse(cylinder(0, 0, -5.5, 8, 2), cylinder(0, 0, 2.5, 4, 3.5)).Shape()
    polygon = BRepBuilderAPI_MakePolygon()
    for i in range(6):
        angle = i * math.pi / 3
        r = 3 / math.sqrt(3)
        polygon.Add(gp_Pnt(r * math.cos(angle), r * math.sin(angle), 4.5))
    polygon.Close()
    socket = BRepPrimAPI_MakePrism(
        BRepBuilderAPI_MakeFace(polygon.Wire()).Face(), gp_Vec(0, 0, 2.1)
    ).Shape()
    return nut, BRepAlgoAPI_Cut(bolt, socket).Shape()


def export(bracket, controller, output):
    import hashlib, json

    review = json.loads(Path(__file__).with_name("bench-fit.json").read_bytes())
    for p in (bracket, controller):
        if hashlib.sha256(p.read_bytes()).hexdigest() != review["supplierSha256"][p.name]:
            raise ValueError("Unknown supplier revision")
    if output.resolve().is_relative_to(Path(__file__).resolve().parents[3]):
        raise ValueError("Choose a private destination outside the source checkout")
    if output.exists():
        raise FileExistsError(output)
    b = transform(read_step(bracket), (1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0.5))
    c = transform(read_step(controller), (-1, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 22.95))
    plate = read_step(Path(__file__).with_name("bench-plate.step"))
    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    builder.Add(compound, plate)
    for x, y in CENTERS:
        builder.Add(compound, place(b, x, y))
        builder.Add(compound, place(c, x, y))
    nut, bolt = hardware()
    for x, y in CLINCH_HOLES:
        for part in (nut, bolt):
            builder.Add(compound, place(part, x, y))
    write_step(compound, output)
    cold = read_step(output)
    solids = list(explore(cold, TopAbs_SOLID))
    if len(solids) != 101 or abs(volume(compound) - volume(cold)) > 0.05:
        raise RuntimeError("Stored assembly failed the independent STEP read")
    print(
        f"{len(solids)} solids reopened; geometry volume delta {volume(cold) - volume(compound):.6f} mm3"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ("bracket", "controller", "output"):
        p.add_argument(arg, type=Path)
    args = p.parse_args()
    export(args.bracket, args.controller, args.output)
