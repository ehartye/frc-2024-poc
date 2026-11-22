"""Single-input brackets of the Profile A floor pickup.

Usage: python run.py   (PYTHONPATH must hold SciPy; the field-query-libs folder supplies it)

Runs pickup-height-control-01/height.py (copied here unchanged) with one input moved at a time from the
nominal pickup: rigid tip height 5 mm, centred NOTE, nose grip 0.45, 8 A phase limit, 48 ring segments.
Each case writes LABEL.json here; results.json lists the outcome of every case and the bracket of each input.
"""
import json, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
HERE = Path(__file__).resolve().parent
NOM = dict(n=48, height=5.0, y=0.0, mu=0.45, amp=8.0)
AXES = dict(height=[1.5, 2, 3.5, 5, 6.5, 8, 10, 12, 14, 16, 18, 20, 24],
            y=[-0.20, -0.15, -0.10, -0.05, 0.05, 0.10, 0.15, 0.20],
            mu=[0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.60, 0.75],
            amp=[0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0])
UNITS = dict(height="mm", y="m", mu="", amp="A", n="segments")
NAMES = dict(height="rigid blade tip height above the carpet", y="lateral NOTE offset from the cassette centre",
             mu="nose roller and pad grip coefficient", amp="nose motor phase current limit")
cases = [dict(NOM)] + [dict(NOM, **{k: v}) for k, vs in AXES.items() for v in vs] + [dict(NOM, n=96)]
cases += [dict(NOM, n=96, **{k: v}) for k, v in (("height", 14), ("height", 16), ("mu", 0.05), ("mu", 0.10), ("amp", 1.0), ("amp", 2.0))]

def label(c):
    return f"n{c['n']}-h{c['height']:g}-y{c['y']:g}-mu{c['mu']:g}-a{c['amp']:g}"

def run(c):
    out = HERE / (label(c) + ".json")
    if not out.is_file():
        subprocess.run([sys.executable, "-B", "height.py", label(c), str(c["n"]), str(c["height"]), "0", "0", str(c["y"]),
                        str(c["mu"]), str(c["amp"])], cwd=HERE, capture_output=True)
    r = json.loads(out.read_bytes()) if out.is_file() else {}
    return dict(c, label=label(c), ran=bool(r), seated=r.get("seated"), passScreen=r.get("passScreen"),
                seatResidenceS=r.get("seatResidenceS"), maxRiseMm=r.get("maxRiseMm"), skip=r.get("skipReason"),
                minimumBladeHeightMm=r.get("minimumBladeHeightMm"), flapClearanceMm=r.get("flapClearanceMm"))

with ThreadPoolExecutor(4) as pool:
    rows = list(pool.map(run, cases))
brackets = []
for k, vs in AXES.items():
    line = sorted([r for r in rows if r["n"] == 48 and all(r[j] == NOM[j] for j in NOM if j not in (k,))], key=lambda r: r[k])
    good = [r[k] for r in line if r["passScreen"]]
    bad = [r[k] for r in line if not r["passScreen"]]
    brackets.append(dict(input=NAMES[k], unit=UNITS[k], nominal=NOM[k], passingValues=good, failingValues=bad))
(HERE / "results.json").write_text(json.dumps(dict(nominal=NOM, cases=rows, brackets=brackets), indent=1) + "\n")
for b in brackets:
    print(b)
print([ (r["label"], r["passScreen"]) for r in rows if r["n"] == 96])
