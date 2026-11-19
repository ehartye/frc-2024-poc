"""Screen the free NOTE against reopened frame article boxes at each attained sample."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import ROOT, digest, load
from verify_swerve_carry import verify


def measure(frame, pickup, carry, *, support="frame"):
    verified = verify(pickup, carry)
    geometry = load(ROOT/"cad/assembly/intake_components.json")["note"]
    boxes = np.asarray([a["boundsMm"] for a in frame["articles"]])
    count_expected = {"frame":46,"mast":6}[support]
    if boxes.shape!=(count_expected,6) or not np.isfinite(boxes).all() or (boxes[:,3:]<=boxes[:,:3]).any():
        raise ValueError(f"Expected {count_expected} complete {support} body bounds")
    minimum = math.inf
    witness = None
    count = 0
    for directory in (pickup,carry):
        with (directory/"trace.jsonl").open() as stream:
            for line in stream:
                row = json.loads(line)
                q = row["q"]
                c,s = math.cos(q[2]),math.sin(q[2])
                dx,dy = q[-7]-q[0],q[-6]-q[1]
                center = np.array([c*dx+s*dy,-s*dx+c*dy,q[-5]])*1000
                w,x,y,z = q[-4:]
                if abs(w*w+x*x+y*y+z*z-1)>1e-7:
                    raise ValueError("NOTE attitude differs from a unit quaternion")
                nx,ny,nz = 2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)
                normal = np.array([c*nx+s*ny,-s*nx+c*ny,nz])
                half = geometry["majorRadius"]*np.sqrt(np.maximum(0.,1-normal**2))+geometry["tubeRadius"]
                gaps = np.maximum(boxes[:,:3]-(center+half),center-half-boxes[:,3:]).max(axis=1)
                index = int(np.argmin(gaps))
                if gaps[index]<minimum:
                    minimum = float(gaps[index])
                    witness = dict(timeS=row["t"],article=frame["articles"][index]["article"])
                count += 1
    if minimum<=0:
        raise ValueError(f"Frame and NOTE bounds overlap at {witness}")
    return dict(attempt=verified["attempt"],pickupTrace=verified["pickupTrace"],carryTrace=verified["carryTrace"],
                samples=count,minimumSampledAxisSeparationMm=minimum,witness=witness)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame", type=Path)
    parser.add_argument("pickup", type=Path)
    parser.add_argument("carry", type=Path)
    parser.add_argument("--support", choices=("frame","mast"), default="frame")
    args = parser.parse_args()
    result = measure(load(args.frame),args.pickup,args.carry,support=args.support)
    print(json.dumps(dict(frameMeasurementSha256=digest(args.frame),**result),indent=2))
