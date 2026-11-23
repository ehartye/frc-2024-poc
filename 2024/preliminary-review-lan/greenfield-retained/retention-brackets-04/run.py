"""Single-input brackets of NOTE retention on the loaded lift: lift duration and chassis acceleration.

Usage: python run.py   (PYTHONPATH must hold SciPy; the field-query-libs folder supplies it)

Runs the pickup-and-lift trial of the nominal first NOTE (dev-middle) with the lift completed in LIFT_S
seconds, or with the loaded drive to the firing station shortened so its peak chassis acceleration
rises from the released 2.6 m/s2 (the route times scale by the square root of the ratio). The released lift takes 4.8 s. `held` is the released criterion: the seated NOTE stays inside
its seat band for the whole lift and the stop. Each case keeps its record and log in its own folder; results.json lists them.
"""
import hashlib, json, os, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "counted-reset-power-02"
CASES = [(48, s, 2.6) for s in (4.8, 3.4, 2.4, 1.8, 1.4, 1.2, 1.0, 0.8, 0.6, 0.5, 0.4, 0.3)] + [(96, s, 2.6) for s in (4.8, 0.8, 0.6, 0.5, 0.4)]
CASES += [(48, 4.8, a) for a in (5.0, 8.0, 12.0, 20.0, 30.0, 45.0, 60.0, 90.0)]

def run(case):
    n, lift, accel = case
    name = f"lift-{lift:g}-{n}" + ("" if accel == 2.6 else f"-accel-{accel:g}")
    work = HERE.parent / f"_w-{name}"      # the trial reads its sources from sibling folders
    keep = HERE / name
    out = keep / f"dev-middle-{n}.json"
    if not out.is_file():
        keep.mkdir(exist_ok=True)
        work.mkdir(exist_ok=True)
        shutil.copyfile(HERE / "carry.py", work / "carry.py")
        plan = json.loads((SOURCE / "plan.json").read_bytes())
        if accel != 2.6:
            leg = next(k for k in plan["plan"] if k["id"] == "middle")
            leg["loadedT"] = [t * (2.6 / accel) ** 0.5 for t in leg["loadedT"]]
            (work / "plan.json").write_text(json.dumps(plan))
        else:
            shutil.copyfile(SOURCE / "plan.json", work / "plan.json")
        seed = work / f"initial-dev-middle-{n}.json"
        shutil.copyfile(SOURCE / seed.name, seed)
        done = subprocess.run([sys.executable, "-B", "carry.py", "middle", "dev", str(n), "working"], cwd=work,
                              env=dict(os.environ, CYCLE_STATE=str(seed), LIFT_S=f"{lift:g}"), capture_output=True, text=True)
        (keep / "run.log").write_text(done.stdout + done.stderr)
        if (work / out.name).is_file():
            shutil.copyfile(work / out.name, out)
        shutil.rmtree(work, ignore_errors=True)
    if not out.is_file():
        return dict(n=n, liftS=lift, peakChassisAccelMps2=accel, ran=False)
    r = json.loads(out.read_bytes())
    after = [v for v in r["trajectory"] if v[0] >= 2.82 + r["seatDelay"]]
    return dict(n=n, liftS=lift, peakChassisAccelMps2=accel, ran=True, held=r["held"], seated=r["seated"], failedGate=r["failedGate"],
                stopOvertravelDeg=__import__("math").degrees(r["stopViolationRad"]),
                seatAxialRangeM=[min(v[8] for v in after), max(v[8] for v in after)],
                seatHeightRangeM=[min(v[9] for v in after), max(v[9] for v in after)],
                lateralPeakM=max(abs(v[2]) for v in after), sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
                sameAsReleased=(out.read_bytes() == (SOURCE / out.name).read_bytes()) if lift == 4.8 and accel == 2.6 else None)

with ThreadPoolExecutor(3) as pool:
    rows = list(pool.map(run, CASES))
(HERE / "results.json").write_text(json.dumps(dict(
    inputs=[dict(input="lift duration", unit="s", released=4.8),
            dict(input="peak chassis acceleration on the loaded drive", unit="m/s2", released=2.6)],
    heldBand="Seated NOTE centre 0.12 to 0.35 m along the cassette, 0.018 to 0.09 m above its deck, within 0.05 m of centre",
    cases=rows), indent=1) + "\n")
for r in rows:
    print(r)
