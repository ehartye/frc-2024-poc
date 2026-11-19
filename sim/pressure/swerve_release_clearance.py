"""Screen every release sample against the R7 bumper and lift-bridge envelopes."""
import argparse
import json
from pathlib import Path

import numpy as np

from run_axial_shear_lift import ROOT, digest, load
from swerve_bumper_note import envelope_rows


def screen(shot, bumper, mast):
    rc = load(shot / "result.json")
    r7 = load(ROOT / "cad/drivetrain/maxswerve-front-crossmember.json")["archiveSha256"]
    if (bumper["archiveSha256"] != r7 or mast["archiveSha256"] != r7 or mast["intersections"]
            or rc["drivetrain"] != "maxswerve" or rc["outcome"] != "UPWARD_FREE_RELEASE"
            or rc["traceStride"] != 1 or digest(shot/"trace.jsonl") != rc["traceSha256"]):
        raise ValueError("Expected verified R7 installation data and the dense attained swerve release")
    rows = envelope_rows(bumper) + [(a["article"], a["boundsMm"]) for a in mast["articles"]]
    boxes = np.asarray([bounds for _,bounds in rows])
    geom = load(ROOT / "cad/assembly/intake_components.json")["note"]
    times, pose = [], []
    with (shot/"trace.jsonl").open() as stream:
        for line in stream:
            sample = json.loads(line)
            times.append(sample["time"])
            pose.append(sample["q"])
    q = np.asarray(pose)
    if (not np.isfinite(q).all() or q.shape[1] != 24 or times[-1] != rc["end"] or
            not np.allclose(np.diff([rc["start"], *times]), rc["dt"], atol=1e-9, rtol=0)):
        raise ValueError("Incomplete release samples")
    c, s = np.cos(q[:,2]), np.sin(q[:,2])
    dx, dy = q[:,-7] - q[:,0], q[:,-6] - q[:,1]
    centers = 1000*np.stack([c*dx+s*dy, -s*dx+c*dy, q[:,-5]], axis=1)
    w,x,y,z = q[:,-4:].T
    if not np.allclose(w*w+x*x+y*y+z*z, 1, atol=1e-7, rtol=0):
        raise ValueError("Invalid NOTE orientation")
    nx,ny = 2*(x*z+w*y), 2*(y*z-w*x)
    normal = np.stack([c*nx+s*ny, -s*nx+c*ny, 1-2*(x*x+y*y)], axis=1)
    extent = geom["majorRadius"]*np.sqrt(np.maximum(0.,1-normal**2)) + geom["tubeRadius"]
    low, high = centers-extent, centers+extent
    nearest = None
    for (name,_), box in zip(rows, boxes):
        gaps = np.maximum(box[:3]-high, low-box[3:]).max(axis=1)
        idx = gaps.argmin()
        hit = (float(gaps[idx]),times[idx],name)
        if nearest is None or hit[0] < nearest[0]:
            nearest = hit
    return dict(piece=rc["pieceId"], plantArchive=rc["nativeSha256"], displayArchive=r7,
                shotTrace=rc["traceSha256"], samples=len(times), minAxisGapMm=nearest[0],
                witnessTimeS=nearest[1], closestEnvelope=nearest[2],
                separatedAtSamples=nearest[0]>0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("shot",type=Path)
    parser.add_argument("bumper",type=Path)
    parser.add_argument("mast",type=Path)
    args=parser.parse_args()
    print(json.dumps(screen(args.shot, load(args.bumper), load(args.mast)), indent=2))
