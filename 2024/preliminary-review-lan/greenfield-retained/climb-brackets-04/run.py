"""Single-input brackets, solver-step repeat and failed-catch control of the guided climb.

Usage: python run.py

Runs climb.py one input at a time from the released forward-CG climb (CG 40 mm ahead, 0.25 ms step,
6.5 s hoist ramp, catch working). A case passes when both hooks capture, the carriage latches on its stops
before motor power is cut, the weak-pack bus stays at or above 8 V, and through the disabled hold no tire
or bumper comes within 10 mm of the carpet or loads it. results.json lists every case and bracket.
"""
import json, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
HERE = Path(__file__).resolve().parent
NOM = dict(cg=0.04, dt=0.00025, ramp=6.5, mode="normal")
CASES = [dict(NOM), dict(NOM, cg=-0.04), dict(NOM, dt=0.000125), dict(NOM, cg=-0.04, dt=0.000125), dict(NOM, mode="no-catch")]
CASES += [dict(NOM, cg=v) for v in (0.05, 0.06, 0.08, 0.10, 0.12, -0.08, -0.12)]
CASES += [dict(NOM, ramp=v) for v in (5.0, 4.0, 3.0, 2.0, 1.5)]

def label(c):
    return f"cg{c['cg']:+.2f}-dt{c['dt']:g}-ramp{c['ramp']:g}-{c['mode']}"

def run(c):
    out = HERE / (label(c) + ".json")
    if not out.is_file():
        done = subprocess.run([sys.executable, "-B", "climb.py", label(c), str(c["cg"]), repr(c["dt"]), str(c["ramp"]), c["mode"]],
                              cwd=HERE, capture_output=True, text=True)
        (HERE / (label(c) + ".log")).write_text(done.stdout[-3000:] + done.stderr[-3000:])
    if not out.is_file():
        return dict(c, label=label(c), ran=False, passed=False)
    r = json.loads(out.read_bytes())
    keys = ("captured", "latched", "weakBusMin", "peakBatteryA", "peakPhaseA", "maxCombinedCableN", "peakPitchLockNm",
            "disabledMinTireOrBumperM", "maxDisabledFloorN", "peakDraw", "modeledRobotMass")
    row = dict(c, label=label(c), ran=True, **{k: r.get(k) for k in keys})
    row["passed"] = bool(r["captured"] and r["latched"] and r["weakBusMin"] >= 8 and r["disabledMinTireOrBumperM"] >= 0.010
                         and r["maxDisabledFloorN"] == 0)
    return row

with ThreadPoolExecutor(3) as pool:
    rows = list(pool.map(run, CASES))
(HERE / "results.json").write_text(json.dumps(dict(nominal=NOM, cases=rows), indent=1) + "\n")
for r in rows:
    print(r["label"], r["passed"], r.get("captured"), r.get("latched"), r.get("disabledMinTireOrBumperM"), r.get("maxDisabledFloorN"), r.get("weakBusMin"))
