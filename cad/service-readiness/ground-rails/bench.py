"""Build and reread the original service gauge, screen the sleeve and reconcile the mass delta."""

import re
import json
import hashlib
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer

from interface import (
    BORE,
    BOTTOM,
    GRIP,
    GRIP_LENGTH,
    LENGTH,
    OFFSET,
    PARENT,
    SHAFT,
    SLEEVE_OD,
    TOP,
    retired,
)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def cylinder(x, z, diameter, height):
    axis = gp_Ax2(gp_Pnt(x, 0, z), gp_Dir(0, 0, 1))
    return BRepPrimAPI_MakeCylinder(axis, diameter / 2, height).Shape()


def volume(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def overlap(a, b):
    return round(volume(BRepAlgoAPI_Common(a, b).Shape()), 4)


def measure():
    sleeve = BRepAlgoAPI_Cut(
        cylinder(0, BOTTOM, SLEEVE_OD, TOP - BOTTOM),
        cylinder(0, BOTTOM - 1, BORE, TOP - BOTTOM + 2),
    ).Shape()
    stem = cylinder(0, TOP - LENGTH, SHAFT, LENGTH)
    gauge = BRepAlgoAPI_Fuse(stem, cylinder(0, TOP, GRIP, GRIP_LENGTH)).Shape()
    out = HERE / "rail-service-gauge.step"
    writer = STEPControl_Writer()
    if (
        writer.Transfer(gauge, STEPControl_AsIs) != IFSelect_RetDone
        or writer.Write(str(out)) != IFSelect_RetDone
    ):
        raise RuntimeError("Gauge STEP export")
    # Keep exchange bytes independent of the wall clock, as for the R11 plate.
    out.write_bytes(
        re.sub(
            rb"(FILE_NAME\('Open CASCADE Shape Model',')[-0-9:T]+",
            rb"\g<1>2026-11-20T00:00:00",
            out.read_bytes(),
            count=1,
        )
    )
    out.write_bytes(b"\n".join(line.rstrip() for line in out.read_bytes().splitlines()) + b"\n")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(out)) != IFSelect_RetDone or reader.TransferRoots() != 1:
        raise RuntimeError("Gauge STEP reread")
    actual = reader.OneShape()
    # The original lower shank occupies the sleeve for its entire length.
    retained_bolt = cylinder(0, BOTTOM, 8.0, TOP - BOTTOM)
    result = {
        "gaugeVolumeMm3": round(volume(actual), 4),
        "nominalRadialClearanceMm": (BORE - SHAFT) / 2,
        "projectionBelowSleeveMm": round(LENGTH - (TOP - BOTTOM), 3),
        "retainedBoltOverlapMm3": overlap(actual, retained_bolt),
        "clearedSleeveOverlapMm3": overlap(actual, sleeve),
        "offsetSleeveOverlapMm3": overlap(cylinder(OFFSET, TOP - LENGTH, SHAFT, LENGTH), sleeve),
    }
    if (
        result["clearedSleeveOverlapMm3"]
        or min(result["offsetSleeveOverlapMm3"], result["retainedBoltOverlapMm3"]) <= 0.01
    ):
        raise ValueError(result)
    return result


def mass_delta():
    folder = ROOT / "cad/assembly-readiness/mass"
    data = json.loads((folder / "actual.json").read_bytes())
    mapping = json.loads((folder / "mapping.json").read_bytes())
    parent = ROOT / "cad/student/r9-physical-assembly.f3d"
    if data["archiveSha256"] != PARENT or hashlib.sha256(parent.read_bytes()).hexdigest() != PARENT:
        raise ValueError("Wrong assembly")
    removed, retained = [], []
    for body in data["bodies"]:
        component = body["occurrence"].split(":")[0]
        (removed if retired(component, body["body"]) else retained).append(body)
    ids = {r["instance"] for r in removed}
    if ids & {r["instance"] for r in retained}:
        raise ValueError("Partial removal of a grouped mass-accounting record")
    records = [i for i in mapping["instances"] if i["id"] in ids]
    known = [i["mass"]["rangeKg"] for i in records if i["mass"]]
    if len(retained) != 892 or len(removed) != 31 or len(ids) != len(records):
        raise ValueError("Configuration inventory changed")
    budget = json.loads((folder / "result.json").read_bytes())
    known_removed = [sum(bounds[k] for bounds in known) for k in (0, 1)]
    return {
        "parentSha256": PARENT,
        "parentBodies": len(data["bodies"]),
        "selectedBodies": len(retained),
        "retired": [{k: b[k] for k in ("occurrence", "body", "instance")} for b in removed],
        "oldNativeMaterialTotalKg": data["rawBodyTotalKg"],
        "retiredNativeProxyKg": sum(b["massKg"] for b in removed),
        "retainedWeldedSleevesNativeKg": sum(
            b["massKg"] for b in retained if b["body"].startswith("Rail flush weld sleeve ")
        ),
        "knownRetiredMassKg": known_removed,
        "retiredUnknownMassRecords": [i["id"] for i in records if not i["mass"]],
        "selectedKnownInstalledSubtotalKg": [
            budget["knownInstalledRangeKg"][k] - known_removed[k] for k in (0, 1)
        ],
        "actualInstalledKg": None,
    }


if __name__ == "__main__":
    receipt = {"offlineInterface": measure(), "configuration": mass_delta()}
    (HERE / "bench-fit.json").write_text(
        json.dumps(receipt, indent=2) + "\n", encoding="utf8", newline="\n"
    )
