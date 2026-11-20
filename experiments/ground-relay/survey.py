"""Reconcile registered R9 bounding boxes to the 909-body R14 parked selection."""
import hashlib
import json
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def load(p):return json.loads(p.read_bytes())

def survey(inventory):
    source=load(inventory)
    actual=load(ROOT/"cad/assembly-readiness/mass/actual.json")
    r14=load(ROOT/"cad/service-readiness/stiffness/native-fit.json")
    assert r14["parentSha256"]==source["archiveSha256"]==actual["archiveSha256"]
    removed={i["body"] for i in load(ROOT/"cad/service-readiness/ground-rails/bench-fit.json")["configuration"]["retired"]}
    removed-=set(load(ROOT/"cad/service-readiness/stiffness/result.json")["selection"]["addedArticles"])
    rows=source["rows"]
    native={r["body"]:r for r in actual["bodies"]}
    assert len(rows)==len(native)==923 and {r["body"] for r in rows}==native.keys()
    assert all(abs(r["volumeCm3"]-native[r["body"]]["volumeCm3"])<.01 for r in rows)
    fitted=[r for r in rows if r["body"] not in removed]
    assert len(fitted)==r14["fullParkedVisibleBodies"]==909
    extremes=[(min if a<3 else max)(fitted,key=lambda r:r["boundsMm"][a]) for a in range(6)]
    return dict(sourceParkedArchiveSha256=source["archiveSha256"],fixtureSha256=r14["archiveSha256"],
                selectedBodies=len(fitted),boundsMm=[r["boundsMm"][i] for i,r in enumerate(extremes)],
                extremalBodies=[r["body"] for r in extremes],retiredFromR9=sorted(removed))

if __name__=="__main__":
    observed=survey(Path(sys.argv[1]))
    assert observed==load(HERE/"footprint.json")
    print("Registered envelope and all six extremal articles match.")
