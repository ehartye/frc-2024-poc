"""Tessellate the official blue speaker and stage from FIRST's 2024 field STEP for the review's replays.

The selection follows the draft layout's match-replay viewer: the same product subtrees, the same display
groups and the same field transform. That viewer tessellated the parts in the visitor's browser; this
script does it once, so the review page needs no network.

Usage: python cad/preliminary-design/builder/field.py FE-2024.STEP OUT.json
"""
import hashlib
import json
import re
import sys
import tempfile
from pathlib import Path

FIELD_STEP_SHA256 = "42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a"
RAMP = re.compile(r"^GE-(24307|24308|24309|24355|24391|24392)")
LEAF = re.compile(r"^(GE-|FE-|Connection Plate|Hook_4|42SMR$)|Truss|Grip_Wheel")


def subtree(source, fixture):
    """Index the AP203 assembly and return a function that writes the STEP text of one display role."""
    records, names, children, products, breps, context = {}, {}, {}, {}, {}, {}
    ids = lambda line: [int(m) for m in re.findall(r"#(\d+)", line)]
    for match in re.finditer(r"^#(\d+)\s*=\s*([^;]+);", source, re.M):
        number, value = int(match.group(1)), match.group(2).strip()
        records[number] = match.group(0)
        if value.startswith("SHAPE_REPRESENTATION ("):
            names[number] = re.search(r"'([^']*)'", value).group(1)
        elif value.startswith("( REPRESENTATION_RELATIONSHIP"):
            parent, child = ids(value)[:2]
            children.setdefault(parent, []).append((number, child))
        elif value.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            context[ids(value)[0]] = number
        elif value.startswith("SHAPE_REPRESENTATION_RELATIONSHIP"):
            breps.setdefault(ids(value)[0], []).append(number)
        elif value.startswith("SHAPE_DEFINITION_REPRESENTATION"):
            products[ids(value)[1]] = number
    wanted = "GE-24300_Briweld Field Blue" if fixture == "speaker" else "GE-24200_Blue"
    targets = [k for k, n in names.items() if n == wanted]
    roots = [k for k, edges in children.items() if targets and any(child == targets[0] for _, child in edges)]
    assert len(targets) >= 1 and len(roots) == 1, "Blue field assembly registration changed"
    target, root = targets[0], roots[0]
    header = source[:source.index("DATA;") + 5]

    def select(role):
        seeds = set()

        def include(number):
            if number in seeds:
                return
            seeds.add(number)
            if number in products:
                seeds.add(products[number])
            seeds.update(breps.get(number, []))
            for edge, child in children.get(number, []):
                if number == root and child != target:
                    continue
                name = names.get(child, "")
                if child not in children:
                    if not LEAF.search(name):
                        continue
                    if fixture == "speaker":
                        kind = "speakerRamp" if RAMP.search(name) else "speakerBody"
                    else:
                        kind = "chains" if name == "GE-24205_hanging" else "anchors" if "Clamp" in name or "Hook_4" in name else "stageSteel"
                    if kind != role:
                        continue
                seeds.add(context[edge])
                include(child)

        include(root)
        seen, stack, lines = set(), list(seeds), [header]
        while stack:
            number = stack.pop()
            if number in seen:
                continue
            seen.add(number)
            line = records[number]
            lines.append(line)
            stack.extend(ref for ref in ids(line) if ref not in seen)
        return "\n".join(lines + ["ENDSEC;", "END-ISO-10303-21;"])

    return select


def solids(step_text):
    """Every solid of a STEP text, placed as the assembly places it."""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "subset.step"
        path.write_text(step_text, encoding="latin-1")
        reader = STEPControl_Reader()
        assert reader.ReadFile(str(path)) == IFSelect_RetDone, "Could not read the field subset"
        reader.TransferRoots()
        shape = reader.OneShape()
    out, explorer = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        out.append(explorer.Current())
        explorer.Next()
    return out


def triangles(solid, angular):
    """Vertices in field coordinates and triangles of one solid."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    BRepMesh_IncrementalMesh(solid, 1.0, False, angular, False)
    points, faces, explorer = [], [], TopExp_Explorer(solid, TopAbs_FACE)
    while explorer.More():
        face = (getattr(TopoDS, "Face_s", None) or TopoDS.Face)(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is not None:
            placement, base = location.Transformation(), len(points)
            for i in range(1, mesh.NbNodes() + 1):
                x, y, z = mesh.Node(i).Transformed(placement).Coord()
                # FIRST's model is Y-up about the field centre; the project field frame is Z-up from the blue corner.
                points.append((8270.875 - x, 4105.275 + z, y - 4.7752))
            for i in range(1, mesh.NbTriangles() + 1):
                a, b, c = (base + k - 1 for k in mesh.Triangle(i).Get())
                faces.append((a, c, b) if face.Orientation() == TopAbs_REVERSED else (a, b, c))
        explorer.Next()
    return points, faces


def merged(parts):
    """One indexed mesh from several, with vertices rounded to 0.01 mm and shared."""
    index, vertices, faces = {}, [], []
    for points, tris in parts:
        local = []
        for p in points:
            key = tuple(round(v, 2) + 0.0 for v in p)
            if key not in index:
                index[key] = len(vertices)
                vertices.append(list(key))
            local.append(index[key])
        for a, b, c in tris:
            a, b, c = local[a], local[b], local[c]
            if a != b and b != c and a != c:
                faces.append([a, b, c])
    return dict(v=vertices, f=faces)


def build(step_path):
    raw = Path(step_path).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIELD_STEP_SHA256, "Supply FIRST's FE-2024.STEP"
    source = raw.decode("latin-1")
    groups = {k: [] for k in ("speakerPanel", "speakerRamp", "sensors", "stageSteel", "anchors", "workingChain", "otherChain")}
    counts = {}
    select = subtree(source, "speaker")
    for role in ("speakerBody", "speakerRamp"):
        found = solids(select(role))
        counts[role] = len(found)
        for solid in found:
            points, faces = triangles(solid, 0.27)
            # The sensor housings are unnamed sub-solids; the draft replay viewer found them by their placed bounds.
            sensor = role == "speakerBody" and all(-1180 <= x <= -1063 and 5735 <= y <= 5788 and 1277 <= z <= 1435 for x, y, z in points)
            groups["sensors" if sensor else "speakerPanel" if role == "speakerBody" else "speakerRamp"].append((points, faces))
    select = subtree(source, "stage")
    for role in ("stageSteel", "anchors", "chains"):
        found = solids(select(role))
        counts[role] = len(found)
        for solid in found:
            points, faces = triangles(solid, 0.48)
            if role != "chains":
                groups[role].append((points, faces))
                continue
            xs = [p[0] for p in points]
            if min(xs) >= 5743 and max(xs) <= 5778:
                groups["workingChain"].append((points, faces))
            elif min(xs) >= 3371 and max(xs) <= 5472:
                groups["otherChain"].append((points, faces))
            else:
                raise ValueError("Unexpected field location for a chain link")
    out = {name: merged(parts) for name, parts in groups.items()}
    for name, mesh in out.items():
        assert mesh["f"], "Missing field geometry: " + name
    return dict(schema="frc-2024-poc/field-display-meshes@1",
                source=dict(file="FE-2024.STEP", sha256=FIELD_STEP_SHA256, archive="CRESCENDO2024Field-STEP.zip",
                            archiveSha256="87185d3bf324bb3f35787ed6be715caf6151f6a9e81794830b3f743c381b4f18",
                            products=["GE-24300_Briweld Field Blue", "GE-24200_Blue"]),
                tessellation=dict(linearDeflectionMm=1.0, angularDeflectionRad=dict(speaker=0.27, stage=0.48), roundedToMm=0.01),
                transform="field = (8270.875 - x, 4105.275 + z, y - 4.7752) mm from the STEP's coordinates",
                solids=counts, meshes=out)


if __name__ == "__main__":
    result = build(sys.argv[1])
    Path(sys.argv[2]).write_bytes((json.dumps(result, separators=(",", ":")) + "\n").encode("utf-8"))
    for name, mesh in result["meshes"].items():
        print(name, len(mesh["v"]), "vertices", len(mesh["f"]), "triangles")
    print(result["solids"])
