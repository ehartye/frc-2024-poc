"""Tie-only articles and installed mass, using project-issued R9 and R13 identities."""
import json
from pathlib import Path

TUBE_NAME = "Crossmember with four rail mounting holes"
STATIONS = [(x,y) for x in (23,53) for y in (-329,329)]
PREFIXES = ("Crossmember flush weld sleeve", "MISUMI CB8-100", "M8 plain washer", "Accu HPN-M8-C8-Z")
TIE_NAMES = {TUBE_NAME} | {f"{p} {x} {y}" for p in PREFIXES for x,y in STATIONS}


def selected_mass():
    here = Path(__file__).resolve().parent
    folder = here.parents[1] / "assembly-readiness/mass"
    base = json.loads((here.parent / "ground-rails/bench-fit.json").read_bytes())["configuration"]
    native = json.loads((folder/"actual.json").read_bytes())
    mapping = json.loads((folder/"mapping.json").read_bytes())
    bodies = native["bodies"]
    selected = [r for r in bodies if r["body"] in TIE_NAMES]
    assert len(selected) == len(TIE_NAMES) == 17
    assert TIE_NAMES <= {r["body"] for r in base["retired"]}
    ids = {r["instance"] for r in selected}
    assert not ids.intersection(r["instance"] for r in bodies if r["body"] not in TIE_NAMES)
    records = [i for i in mapping["instances"] if i["id"] in ids]
    assert len(records) == len(ids) and all(r["mass"] for r in records)
    delta = [sum(r["mass"]["rangeKg"][k] for r in records) for k in range(2)]
    return dict(retainedParkedBodies=base["selectedBodies"]+len(selected),
                addedArticles=sorted(TIE_NAMES), nativeMaterialKg=sum(r["massKg"] for r in selected),
                addedInstalledRangeKg=delta,
                knownInstalledSubtotalKg=[a+b for a,b in zip(base["selectedKnownInstalledSubtotalKg"],delta)])
