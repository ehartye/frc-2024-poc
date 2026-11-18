"""Continue a verified turning-lift trace into powered, simultaneous roller contact."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from launcher_handoff import RaisedFeed
from run_axial_shear_lift import EVIDENCE, ROOT, digest, load


def run(args):
    reference = load(EVIDENCE/"turning-lift-20261118/refined-end-to-end.json")
    inputs = reference["freshInputDigests"]
    lift = args.pickup/"lift"
    packets = {"native":args.archive, "ground":args.pickup/"ground/receipt.json",
               "stop":args.pickup/"stop/receipt.json"}
    if any(digest(path) != inputs[key] for key,path in packets.items()):
        raise ValueError("Native archive or pickup receipt differs from the verified floor run")
    trace = lift/"trace.jsonl"
    if digest(trace) != reference["liftTraceMatchesIndependentNominalSha256"]:
        raise ValueError("Turning-lift trace mismatch")
    with trace.open() as stream:
        for line in stream:
            last = line
    row = json.loads(last)
    if abs(row["clock"]-reference["holdEndsAtS"]) > 1e-9:
        raise ValueError("Unexpected handoff clock")
    calibration_path = ROOT/"sim/mujoco/evidence/launcher-quadrature-verification-20261118.json"
    verifier = load(calibration_path)
    calibration = verifier["runs"]["grid-16384"]["parameters"]
    if (verifier["nativeArchiveSha256"] != inputs["native"] or not verifier["numericalGatePassed"]
            or calibration["contactLaw"] != "uniform-ring-constraint-quadrature"):
        raise ValueError("Launcher calibration source mismatch")
    binding = load(ROOT/"sim/mujoco/evidence/native-launcher-binding-20261118.json")
    if (binding["nativeArchiveSha256"] != inputs["native"]
            or any(digest(ROOT/name) != value for name,value in binding["inputs"].items())
            or digest(ROOT/"sim/mujoco/native_launcher_quadrature.py") != verifier["runnerSha256"]):
        raise ValueError("Native launcher inputs differ from their verified revision")
    model = RaisedFeed(load(packets["stop"])["final"], row, args.dt, args.grid, args.segments,
                       calibration)
    args.output.mkdir(parents=True, exist_ok=False)
    expected_q = np.r_[model.data.qpos[:5],model.data.qpos[model.nq:]]
    expected_v = np.r_[model.data.qvel[:5],model.data.qvel[model.nv:]]
    if not (np.array_equal(expected_q,row["q"]) and np.array_equal(expected_v,row["v"])
            and np.all(model.data.qvel[model.wheel_dofs]==0)):
        raise ValueError("State continuity failed")
    drive = 0. if args.case=="unpowered" else model.launcher["duty"]
    period = round(.01/args.dt)
    if not math.isclose(period*args.dt,.01):raise ValueError("Inexact command clock")
    if args.case == "unpowered":
        # Replay the nominal feed command clock with motor power removed.
        model.feed_start = row["clock"] + .08
    first = last_contact = first_both = None
    outcome = "HORIZON"
    min_bus = model.bus
    with (args.output/"trace.jsonl").open("x") as stream:
        for step in range(round(args.horizon/args.dt)):
            try:
                belt, wheel = model.step(step % period == 0, drive, args.case != "hold")
            except (ValueError, ArithmeticError) as error:
                outcome = str(error)
                break
            d = model.data
            t = float(d.time)
            min_bus = min(min_bus,model.bus)
            if wheel > 1e-6:
                if first is None: first = t-args.dt
                last_contact = t-args.dt
                if belt > 1e-6 and first_both is None:first_both = t-args.dt
            separated = (last_contact is not None and t-last_contact > .03 and
                         belt < 1e-6 and d.qvel[model.nv+2]>0)
            if step % args.trace_stride == 0 or separated or step + 1 == round(args.horizon/args.dt):
                sample = dict(time=t,q=d.qpos.tolist(),v=d.qvel.tolist(),externalRotors=model.rotors.tolist(),
                              shearM=model.shear.tolist(),beltNormalN=belt,wheelNormalN=wheel,
                              forceTime=t-args.dt,busV=model.bus,sourceJ=model.source_work)
                stream.write(json.dumps(sample,allow_nan=False)+"\n")
            if separated:
                outcome = "UPWARD_FREE_RELEASE"
                break
    if any(w.number for w in model.data.warning):outcome="MUJOCO_WARNING"
    receipt = dict(schema="native-raised-feed/1",case=args.case,outcome=outcome,dt=args.dt,grid=args.grid,
                   segments=args.segments,sourceTraceSha256=digest(trace),nativeSha256=inputs["native"],
                   runnerSha256=digest(Path(__file__)),plantSha256=digest(Path(__file__).with_name("launcher_handoff.py")),
                   traceStride=args.trace_stride,initialQ=model.initial_position,initialV=model.initial_velocity,
                   calibrationSha256=digest(calibration_path),start=row["clock"],end=float(model.data.time),
                   feedStart=model.feed_start,firstLauncherContact=first,firstSimultaneousContact=first_both,
                   lastLauncherContact=last_contact,minimumBusV=min_bus,sourceWorkJ=model.source_work,
                   unloadingHeatJ=model.plastic_heat,traceSha256=digest(args.output/"trace.jsonl"),
                   finalNotePositionM=model.data.qpos[model.nq:model.nq+3].tolist(),
                   finalNoteVelocityMps=model.data.qvel[model.nv:model.nv+3].tolist(),
                   finalWheelRadps=model.data.qvel[model.wheel_dofs].tolist(),verifiedSpeakerScores=0)
    (args.output/"result.json").write_text(json.dumps(receipt,indent=2,allow_nan=False)+"\n")
    print(json.dumps(receipt))
    return outcome


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("pickup","archive","output"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--dt",type=float,choices=(.0001,.00005),default=.0001)
    p.add_argument("--grid",type=int,choices=(256,512,1024,2048),default=1024)
    p.add_argument("--segments",type=int,choices=(512,1024,2048,4096,8192,16384),default=8192)
    p.add_argument("--horizon",type=float,default=.8)
    p.add_argument("--case",choices=("nominal","hold","unpowered"),default="nominal")
    p.add_argument("--trace-stride",type=int,default=1)
    options=p.parse_args()
    if options.trace_stride < 1:p.error("trace stride must be positive")
    if not 0 < options.horizon <= 3:p.error("horizon must be in (0,3] seconds")
    outcome = run(options)
    raise SystemExit(0 if outcome in ("UPWARD_FREE_RELEASE", "HORIZON") else 1)
