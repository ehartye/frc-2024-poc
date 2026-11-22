"""Summarise the single-shot scoring campaign into summary.json.

Usage: python analyse.py

Reads results/*.jsonl and, for the released operating point, the 46 shot records of counted-reset-power-02
(which this campaign's solver reproduces exactly). A shot is counted when both speaker beams are shadowed
within the 7 s it is observed; it is delivered when the whole NOTE passes through the opening.
"""
import collections
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "counted-reset-power-02"
AXES = ("speed", "angle", "x", "y", "yaw", "lower", "upper", "fieldMu")
NOMINAL = dict(speed=6.88, angle=122.0, x=1.39, y=5.871, yaw=0.0, lower=0.75, upper=0.75, fieldMu=0.45)


def released():
    rows = []
    for f in sorted(SOURCE.glob("split-tail_.25_0-*-6.88.json")):
        r = json.loads(f.read_bytes())
        tag, n = f.name[len("split-tail_.25_0-"):-len("-6.88.json")].rsplit("-", 1)
        last = r["trace"][-1]
        rows.append(dict(NOMINAL, tag=tag, n=int(n), stage="released", passShot=r["passShot"], opening=r["opening"],
                         both=r["both"], minBusV=r["minBusV"], endT=last["t"], endQ=last["q"][:3], source=f.name))
    return rows


def wilson(k, n, z=1.96):
    if not n:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [max(0.0, c - h), min(1.0, c + h)]


def tally(rows):
    n = len(rows)
    delivered = [r for r in rows if r.get("opening")]
    counted = [r for r in rows if r.get("passShot")]
    margins = [1000 * r["opening"]["minimumMarginM"] for r in delivered]
    times = sorted(r["both"] for r in counted if r.get("both"))
    inside = [r for r in rows if r.get("opening") and not r.get("passShot") and r["endQ"][0] < 0.46 and r["endQ"][2] > 1.0]
    out = [r for r in rows if r.get("opening") and not r.get("passShot") and not (r["endQ"][0] < 0.46 and r["endQ"][2] > 1.0)]
    return dict(shots=n, delivered=len(delivered), counted=len(counted),
                countedFraction=len(counted) / n if n else None, counted95=wilson(len(counted), n),
                smallestOpeningMarginMm=min(margins) if margins else None,
                medianCountS=times[len(times) // 2] if times else None, latestCountS=times[-1] if times else None,
                uncountedStillInSpeaker=len(inside), uncountedLeftSpeaker=len(out),
                lowestBusV=min(r["minBusV"] for r in rows) if rows else None,
                uncounted=sorted(f"{r['tag']}-{r['n']}" for r in rows if not r.get("passShot")))


def main():
    rows = released()
    errors = []
    for f in sorted((HERE / "results").glob("*.jsonl")):
        for line in f.read_text().splitlines():
            r = json.loads(line)
            if "error" in r:
                errors.append(r["id"])
                continue
            r["stage"] = f.stem
            rows.append(r)
    frozen = [r for r in rows if r["tag"].startswith("R0")]
    dev = [r for r in rows if r["tag"].startswith("dev")]
    summary = dict(criterion="Counted: both speaker beams shadowed within 7 s of the shot start, firing bus at or above 8 V. "
                             "Delivered: whole NOTE through the opening.",
                   population=dict(frozenCarries=len({(r["tag"], r["n"]) for r in frozen}),
                                   nominalCarries=len({(r["tag"], r["n"]) for r in dev})),
                   errors=errors, sweeps={})
    for axis in AXES:
        for name, pop in (("frozen", frozen), ("nominal", dev)):
            line = [r for r in pop if r.get("duration", 7.0) == 7.0 and all(r.get(k, NOMINAL[k]) == NOMINAL[k] for k in AXES if k != axis
                                          and not (axis == "lower" and k == "upper"))]
            if axis == "lower":
                line = [r for r in line if r.get("upper") == r.get("lower")]
            values = sorted({r.get(axis, NOMINAL[axis]) for r in line})
            if len(values) < 2:
                continue
            summary["sweeps"].setdefault(axis, {})[name] = [
                dict(value=v, **tally([r for r in line if r.get(axis, NOMINAL[axis]) == v])) for v in values]
    summary["allFrozenShots"] = tally(frozen)
    summary["allNominalShots"] = tally(dev)
    (HERE / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    for axis, pops in summary["sweeps"].items():
        for name, line in pops.items():
            print(axis, name)
            for t in line:
                print("  %8.3f  counted %3d/%3d  delivered %3d  margin %s mm  median %s s  inside %d  left %d" % (
                    t["value"], t["counted"], t["shots"], t["delivered"],
                    None if t["smallestOpeningMarginMm"] is None else round(t["smallestOpeningMarginMm"], 1),
                    None if t["medianCountS"] is None else round(t["medianCountS"], 2),
                    t["uncountedStillInSpeaker"], t["uncountedLeftSpeaker"]))
    print("frozen", {k: summary["allFrozenShots"][k] for k in ("shots", "delivered", "counted", "counted95")})
    print("errors", len(errors))


if __name__ == "__main__":
    main()
