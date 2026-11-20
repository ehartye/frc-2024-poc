"""Bound the carried torus against the reopened frame and removable bumper hardware."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import ROOT, digest, load
from verify_swerve_carry import verify


def envelope_rows(native):
    if len(native["articles"])!=24 or native["intersections"] or native["toolIntersections"]:
        raise ValueError("Expected a complete installed bumper fit check")
    rows = [(a["name"],a["boundsMm"]) for a in native["articles"] if a["name"]!="Front bumper frame rail"]
    rows.extend(("Front bumper frame rail",b) for b in native["frontRegionsMm"])
    if len(native["foam"])!=8:
        raise ValueError("Expected eight physical bumper cushions")
    # Conservatively include the project's 1.5 mm cover allowance around all
    # plywood and foam faces, including the portions covered by adjacent parts.
    for a in [*native["foam"],*(a for a in native["articles"] if a["name"].endswith(" bumper wood"))]:
        padded = [x+(-1.5 if i<3 else 1.5) for i,x in enumerate(a["boundsMm"])]
        rows.append((a["name"]+" with cover allowance",padded))
    return rows


def measure(native,pickup,carry,*,rows=None):
    receipt = verify(pickup,carry)
    if rows is None:
        rows = envelope_rows(native)
    boxes = np.asarray([box for _,box in rows])
    geometry = load(ROOT/"cad/assembly/intake_components.json")["note"]
    radius,tube = geometry["majorRadius"],geometry["tubeRadius"]
    resolution = 4096
    angles = np.arange(resolution)*2*math.pi/resolution
    ring = radius*np.column_stack((np.cos(angles),np.sin(angles)))
    chord = 2*radius*math.sin(math.pi/(2*resolution))
    lowest = best = math.inf
    witness = None
    samples = 0
    for folder in (pickup,carry):
        with (folder/"trace.jsonl").open() as stream:
            for line in stream:
                row = json.loads(line)
                q = row["q"]
                c,s = math.cos(q[2]),math.sin(q[2])
                yaw = np.array([[c,-s,0],[s,c,0],[0,0,1]])
                center = 1000*np.array([q[-7]-q[0],q[-6]-q[1],q[-5]])@yaw
                w,x,y,z = q[-4:]
                if abs(w*w+x*x+y*y+z*z-1)>1e-7:
                    raise ValueError("Invalid NOTE orientation")
                basis = np.array([[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)],
                                  [2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)]])@yaw
                extent = radius*np.linalg.norm(basis,axis=0)+tube
                broad = np.maximum(boxes[:,:3]-center-extent,center-extent-boxes[:,3:]).max(axis=1)
                curve = None
                for i in np.flatnonzero(broad<=best):
                    if curve is None:
                        curve = center+ring@basis
                    delta = np.maximum(np.maximum(boxes[i,:3]-curve,curve-boxes[i,3:]),0.)
                    upper = math.sqrt(float(np.einsum("ij,ij->i",delta,delta).min()))-tube
                    # Distance to a box is 1-Lipschitz. Every point of the
                    # centerline is within one half-step chord of a sample.
                    lowest = min(lowest,max(broad[i],upper-chord))
                    if upper<best:
                        best = upper
                        witness = dict(part=rows[i][0],timeS=row["t"])
                samples += 1
    if not math.isfinite(lowest) or lowest<=0:
        raise ValueError(f"Unresolved bumper or hardware clearance at {witness}")
    return dict(attempt=receipt["attempt"],pickupTrace=receipt["pickupTrace"],carryTrace=receipt["carryTrace"],
                samples=samples,closestEnvelopeLowerMm=lowest,closestEnvelopeUpperMm=best,
                angularSamples=resolution,witness=witness)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("native","pickup","carry"):
        parser.add_argument(name,type=Path)
    args = parser.parse_args()
    print(json.dumps(dict(nativeMeasurementSha256=digest(args.native),
                          **measure(load(args.native),args.pickup,args.carry)),indent=2))
