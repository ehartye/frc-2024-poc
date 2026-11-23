"""Write the release 004 layout STEP: the release 003 solids with repaired names and one design revision.

Revision 004-A. The whole-cycle layout check found that the floor NOTE, riding over the pickup lip, passes
between the two launch belt motors with no clearance: each motor can is 36 mm in diameter on a centreline
195.8 mm from the cassette centre, so its inner face is 177.8 mm out, exactly the NOTE's outer radius. The
motors are raised 60 mm along the cassette normal, away from the deck, and each drive shaft is lengthened by
the same amount. Every other solid is the reviewed release 003 geometry.

The file is written through the CAD kernel, so it is a new export: 232 named solids, checked below against
the source. The double-encoded middle dot of the source names is written as its ISO 10303-21 directive.

Usage: python cad/preliminary-design/build-004/revise_step.py SOURCE.step TARGET.step
"""
import hashlib
import math
import re
import sys
from pathlib import Path

SOURCE_SHA256 = "a29a146d8897f64e8f056fcb46a431ba234c7aecccbc3233b6834c43c703bc88"
PARK_ANGLE_DEG = 125.0
RAISE_MM = 60.0
MOTORS = ("Launch Belt motor · left", "Launch Belt motor · right")
SHAFTS = ("Launch Belt drive shaft · left", "Launch Belt drive shaft · right")
STAMP = "2026-11-22T00:00:00"


def repaired(name):
    return name.replace("Â·", "·")


def read_named(path):
    """[(name, shape)] of the free shapes of a flat STEP, in file order."""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    try:
        from OCP.TDF import TDF_LabelSequence
    except ImportError:
        from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    reader = STEPCAFControl_Reader()
    assert reader.ReadFile(str(path)) == IFSelect_RetDone and reader.Transfer(doc), "Could not read " + str(path)
    roots = TDF_LabelSequence()
    table.GetFreeShapes(roots)
    out = []
    for i in range(1, roots.Length() + 1):
        label = roots.Value(i)
        attribute = TDataStd_Name()
        assert label.FindAttribute(TDataStd_Name.GetID_s(), attribute)
        out.append((attribute.Get().ToExtString(), table.GetShape_s(label)))
    return out


def measure(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    lo, hi = box.CornerMin(), box.CornerMax()
    return props.Mass(), (lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z())


def moved(shape, distance):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf, gp_Vec
    a = math.radians(PARK_ANGLE_DEG)
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(-math.sin(a) * distance, 0.0, math.cos(a) * distance))   # cassette normal when parked
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def lengthened(shaft):
    """The shaft joined to copies of itself along its axis, as one solid RAISE_MM longer."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    out = shaft
    for step in (RAISE_MM / 2, RAISE_MM):
        out = BRepAlgoAPI_Fuse(out, moved(shaft, step)).Shape()
    unify = ShapeUpgrade_UnifySameDomain(out, True, True, True)
    unify.Build()
    it = TopExp_Explorer(unify.Shape(), TopAbs_SOLID)
    solids = []
    while it.More():
        solids.append(it.Current())
        it.Next()
    assert len(solids) == 1, "The lengthened shaft is not one solid"
    return solids[0]


def write_named(rows, path):
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for name, shape in rows:
        TDataStd_Name.Set_s(table.AddShape(shape, False), TCollection_ExtendedString(name))
    writer = STEPCAFControl_Writer()
    assert writer.Transfer(doc, STEPControl_AsIs) and writer.Write(str(path)) == IFSelect_RetDone
    # A fixed header time, so the same inputs give the same bytes.
    data = Path(path).read_bytes()
    data, n = re.subn(rb"(FILE_NAME\('[^']*',')[^']*(')", rb"\g<1>" + STAMP.encode() + rb"\2", data, count=1)
    assert n == 1, "No STEP header time to fix"
    # The kernel binding takes the UTF-8 bytes of a name as single-byte characters and writes each middle dot
    # encoded twice. Write the ISO 10303-21 directive for U+00B7 in its place, as step_names.py does.
    import step_names
    data = data.replace(step_names.DOUBLE, step_names.DIRECTIVE)
    assert all(b < 0x80 for b in data), "Non-ASCII bytes remain in the STEP"
    Path(path).write_bytes(data)


def build(source, target):
    """Write the revised STEP and return what changed, after checking it against the source."""
    source, target = Path(source), Path(target)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE_SHA256, "Unexpected source STEP"
    before = [(repaired(n), s) for n, s in read_named(source)]
    names = [n for n, _ in before]
    assert len(names) == len(set(names)) == 232 and set(MOTORS + SHAFTS) <= set(names)
    rows = [(n, moved(s, RAISE_MM) if n in MOTORS else lengthened(s) if n in SHAFTS else s) for n, s in before]
    write_named(rows, target)
    after = {repaired(n): s for n, s in read_named(target)}
    assert list(after) == names, "The written STEP does not hold the same named solids"
    changed = {}
    for name, shape in before:
        v0, b0 = measure(shape)
        v1, b1 = measure(after[name])
        if name in MOTORS + SHAFTS:
            changed[name] = dict(volumeMm3=[v0, v1], boundsBeforeMm=list(b0), boundsAfterMm=list(b1))
            if name in MOTORS:
                assert abs(v1 - v0) < 1e-3 * v0
            else:
                assert abs(v1 / v0 - (1 + RAISE_MM / 56.0)) < 0.01, (name, v0, v1)
        else:
            assert abs(v1 - v0) <= 1e-6 * max(1.0, v0) and max(abs(x - y) for x, y in zip(b0, b1)) < 1e-6, name
    return dict(revision="004-A", raiseMm=RAISE_MM, sourceSha256=SOURCE_SHA256,
                sha256=hashlib.sha256(target.read_bytes()).hexdigest(), solids=len(names), changed=changed)


if __name__ == "__main__":
    import json
    print(json.dumps(build(*sys.argv[1:3]), indent=1, ensure_ascii=False))
