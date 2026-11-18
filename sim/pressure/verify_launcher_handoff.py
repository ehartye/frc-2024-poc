"""Independently read the actual handoff samples and compare them on one simulated clock."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import digest


def examine(folder):
    result = json.loads((folder/"result.json").read_bytes())
    trace = folder/"trace.jsonl"
    if result["schema"] != "native-raised-feed/1":
        raise ValueError("Unexpected experiment schema")
    if result["outcome"] not in ("HORIZON", "UPWARD_FREE_RELEASE"):
        raise ValueError("Incomplete physical trial: " + result["outcome"])
    if result["traceStride"] != 1 or digest(trace) != result["traceSha256"]:
        raise ValueError("Verification requires the original every-step trace")
    rows = [json.loads(line) for line in trace.read_bytes().splitlines()]
    start, dt = result["start"], result["dt"]
    times = np.array([r["time"] for r in rows])
    if not len(times) or not np.allclose(times,start+dt*np.arange(1,len(times)+1),atol=1e-8,rtol=0):
        raise ValueError("Missing or retimed handoff step")
    if not math.isclose(times[-1], result["end"], abs_tol=1e-8, rel_tol=0):
        raise ValueError("Missing final physical state")
    p = np.array([r["q"][-7:-4] for r in rows])
    v = np.array([r["v"][-6:-3] for r in rows])
    wheel = np.array([r["wheelNormalN"] for r in rows])
    belt = np.array([r["beltNormalN"] for r in rows])
    both = (wheel > 1e-6) & (belt > 1e-6)
    engaged = wheel > 1e-6
    if not np.isfinite(np.column_stack((p,v,belt,wheel))).all():
        raise ValueError("Nonfinite contact trajectory")
    if any(result["initialV"][5:9]):
        raise ValueError("Launcher must spin up from rest")
    separated = (np.any(both) and not engaged[-1] and belt[-1] < 1e-6 and v[-1,2] > 0 and
                 times[-1]-times[engaged][-1]+dt >= .03)
    if separated != (result["outcome"]=="UPWARD_FREE_RELEASE"):
        raise ValueError("Reported release disagrees with contact history")
    summary = dict(case=result["case"], sourceTraceSha256=result["sourceTraceSha256"],
                   runnerSha256=result["runnerSha256"], plantSha256=result["plantSha256"],
                   nativeArchiveSha256=result["nativeSha256"], traceSha256=result["traceSha256"],
                   startS=start, endS=result["end"], dtS=dt, ringSegments=result["segments"],
                   pressureGrid=result["grid"], minBusV=min(r["busV"] for r in rows),
                   initialLauncherRadps=result["initialV"][5:9], simultaneousSeconds=float(both.sum()*dt),
                   firstSimultaneousS=float(times[both][0]-dt) if np.any(both) else None,
                   lastLauncherS=float(times[engaged][-1]-dt) if np.any(engaged) else None,
                   finalVerticalMps=float(v[-1,2]), attainedUpwardRelease=bool(separated))
    return summary, times, p, v


def compare(a,b):
    # Compare states at their common integration times. Discard neither transient.
    count = int(np.searchsorted(a[1], b[1][-1]+1e-8, side="right"))
    times = a[1][:count]
    i = np.rint((times-b[0]["startS"])/b[0]["dtS"]).astype(int)-1
    if not np.allclose(times,b[1][i],rtol=0,atol=1e-8):
        raise ValueError("No exact common clock")
    dp=float(np.linalg.norm(a[2][:count]-b[2][i],axis=1).max())
    dv=float(np.linalg.norm(a[3][:count]-b[3][i],axis=1).max())
    return dict(sharedSamples=count,maxPositionDifferenceM=dp,maxVelocityDifferenceMps=dv,
                meetsEarlierStagedBounds=dp <= .001 and dv <= .01)


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("spatial","temporal","refined","hold","passive","output"):
        p.add_argument("--"+name,type=Path,required=True)
    args=p.parse_args()
    runs={k:examine(getattr(args,k)) for k in ("spatial","temporal","refined","hold","passive")}
    if len({tuple(v[0][k] for k in ("sourceTraceSha256","nativeArchiveSha256","runnerSha256","plantSha256"))
            for v in runs.values()})!=1:
        raise ValueError("Runs use different source states")
    if not all(runs[k][0]["attainedUpwardRelease"] for k in ("spatial","temporal","refined")):
        raise ValueError("Powered trials must release the acquired NOTE")
    if (runs["hold"][0]["lastLauncherS"] is not None or runs["passive"][0]["attainedUpwardRelease"]
            or runs["passive"][0]["firstSimultaneousS"] is None):
        raise ValueError("Feed or power counterexample failed")
    report=dict(schema="native-raised-feed-comparison/1",runs={k:v[0] for k,v in runs.items()},
                spatial=compare(runs["spatial"],runs["temporal"]),
                timestep=compare(runs["temporal"],runs["refined"]),verifiedSpeakerScores=0)
    with args.output.open("x") as f:
        json.dump(report,f,indent=2,allow_nan=False)
        f.write("\n")
    print(json.dumps({k:report[k] for k in ("spatial","timestep")}))
