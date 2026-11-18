"""Drive the axial shear hypothesis using the failed lift's recorded motor duties."""
import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "sim" / folder) for folder in
                ("pressure", "integrated", "drivetrain", "actuation", "acquisition")]
from axial_shear_lift import ShearLift
from native_pressure_lift import LiftDomainError

EVIDENCE = ROOT / "sim/pressure/evidence"
SOURCE_SHA = "d819bdf34a7e04413a32ee57c8a3e31c3eb9b43076a1daed602ccd3e76f479a8"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load(path):
    return json.loads(path.read_bytes())


def run(args):
    binding = load(EVIDENCE / "native-signed-20261118/native-pressure-binding.json")
    if digest(args.archive) != binding["nativeArchiveSha256"] or digest(args.stop) != SOURCE_SHA:
        raise ValueError("Native or acquired stop input differs")
    source = load(args.stop)
    contract = load(EVIDENCE / "native-lift-20261118/native-pressure-lift-contract.json")
    proof = load(EVIDENCE / "native-lift-20261118/native-lift-clocked-verification.json")
    commands = load(args.commands)
    if digest(args.commands) != proof["runs"][0]["receiptSha256"]:
        raise ValueError("Use the recorded half-step lift controls")
    contract["resolution"] = [args.grid, args.grid]
    m = ShearLift(source["final"], load(ROOT / "sim/drivetrain/config.json"),
                  load(ROOT / "sim/acquisition/free-planar-config.json")["model"],
                  load(ROOT / "cad/assembly/intake_components.json"), binding,
                  load(ROOT / "sim/actuation/config.json"), contract, shear_stiffness=args.stiffness)
    initial = source["final"]
    for key, value in initial.items():
        if key == "ledger":
            assert all(m.s[key][field] == amount for field, amount in value.items())
        else:
            assert m.s[key] == value
    join = m.start_mechanical - sum(source["trace"][-1]["energyJ"][k] for k in ("kinetic", "spring"))
    assert abs(join) < (1e-12 if args.grid == 1024 else contract["comparisonBounds"]["incrementalEnergyJ"])
    events = commands["events"]
    ticks = round(.01 / args.dt)
    args.output.mkdir(parents=True, exist_ok=False)
    status = "HORIZON"
    with (args.output / "trace.jsonl").open("x", encoding="utf-8") as trace:
        for i in range(ticks * len(events)):
            event = events[i // ticks]
            if i % ticks == 0:
                assert abs(m.s["timeS"] - event["timeS"]) < 1e-9
            try:
                row = m.step(args.dt, event["command"])
            except LiftDomainError as error:
                status = str(error)
                break
            if max(abs(row[k]) for k in ("sourcePowerResidualW", "portMidpointPowerDifferenceW",
                                         "pressureWorkResidualW")) > 1e-8:
                raise ValueError("power budget exceeded")
            trace.write(json.dumps(row, separators=(",", ":")) + "\n")
            if (i + 1) % (10*ticks) == 0:
                trace.flush()
                print(i+1, m.s["noteZ"], m.s["zPivot"], flush=True)
    result = dict(outcome=status, dt=args.dt, pressureGrid=args.grid, axialStiffnessPerRegionNpm=args.stiffness,
                  sourceSha256=SOURCE_SHA, archiveSha256=binding["nativeArchiveSha256"],
                  commandTraceSha256=digest(args.commands), startTimeS=initial["timeS"],
                  initialMechanicalDifferenceJ=join, endTimeS=m.s["timeS"],
                  inheritedResidualJ=source["trace"][-1]["energyJ"]["residual"],
                  producerSha256=digest(pathlib.Path(__file__)),
                  modelSha256=digest(ROOT / "sim/pressure/axial_shear_lift.py"))
    (args.output / "result.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("archive", "stop", "commands", "output"):
        parser.add_argument("--"+name, type=pathlib.Path, required=True)
    parser.add_argument("--dt", type=float, choices=(.0005, .00025, .000125), default=.0005)
    parser.add_argument("--grid", type=int, choices=(512, 1024, 2048), default=1024)
    parser.add_argument("--stiffness", type=float, default=2500.)
    run(parser.parse_args())
