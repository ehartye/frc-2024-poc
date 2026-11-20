"""Audit the compact public subassembly registration against the original native export."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def read(path):
    return json.loads(path.read_bytes())


def audit(folder):
    rows = read(folder / "r9.json")
    mesh = read(folder / "mesh/manifest.json")
    bill = read(ROOT / "cad/assembly-readiness/mass/actual.json")
    native = read(ROOT / "cad/service-readiness/stiffness/native-fit.json")
    assert mesh["sourceSha256"] == rows["archiveSha256"] == bill["archiveSha256"] == native["parentSha256"]
    assert mesh["frame"] == "robot-floor-XYZ" and mesh["units"] == "mm"
    source = rows["rows"]
    assert len(source) == len(bill["bodies"]) == 923
    assert all(x["identity"] == y["identity"] and x["body"] == y["body"]
               and abs(x["volumeCm3"]-y["volumeCm3"]) < .01 for x,y in zip(source,bill["bodies"]))
    retired = set(read(ROOT / "experiments/ground-relay/footprint.json")["retiredFromR9"])
    assert sum(x["body"] not in retired for x in source) == native["fullParkedVisibleBodies"] == 909
    groups = {m["name"]: [source[int(i)] for i in m["sourceBodyKeys"]] for m in mesh["meshes"]}
    assert sum(map(len,groups.values())) == 923
    cradle = groups["R7 cradle / R7 mechanism"]
    assert read(HERE / "cradle-bounds.json") == {r["body"]:r["boundsMm"] for r in cradle}
    treads = groups["R7 cradle / R7 structure"]
    assert {r["identity"] for r in treads} == {"r7-body-534","r7-body-535"}
    assert all(r["body"] not in retired for r in cradle+treads)
    for row in treads:
        lo, hi = row["boundsMm"][:3], row["boundsMm"][3:]
        assert abs(lo[0]-144.32239)<.001 and abs(hi[0]-338.67761)<.001
        assert sorted([abs(lo[1]),abs(hi[1])]) == [176.42239,205.67761]
        assert [lo[2],hi[2]] == [19.05,31.75]
    print("Reconciled 923 source rows, 909 selected R14 bodies, 34 cradle bodies and both treads.")


if __name__ == "__main__":
    audit(Path(sys.argv[1]))
