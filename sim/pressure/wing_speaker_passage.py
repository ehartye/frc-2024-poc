"""Bound the free ring's passage through the nominal 2024 blue Speaker opening."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import digest
from wing_field_frame import WingFrame


# FE-2024 blue carpet frame; GE-24300 sheet 2 and registered panel faces.
LOW = np.array([-.001050, (5.020365+6.071290)/2, 1.984464])
HIGH = np.array([.458944, LOW[1], 2.105733])
WIDTH = 6.071290-5.020365
MAJOR, TUBE = .1524, .0254


def rotation(q):
    w,x,y,z = q
    if abs(np.linalg.norm(q)-1.)>1e-8:
        raise ValueError("Invalid NOTE quaternion")
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def screen(row, azimuths=512, dt=.00025, lateral_offset=0., frame=None):
    frame = frame or WingFrame()
    q,v = row["q"], row["v"]
    center = frame.to_field(q[-7:-4]) + [0., lateral_offset, 0.]
    velocity = frame.rotation @ v[-6:-3]
    axes = frame.rotation @ rotation(q[-4:])
    transverse = MAJOR**2/2 + 5*TUBE**2/8
    axial = MAJOR**2 + 3*TUBE**2/4
    momentum = axes @ (np.array([transverse,transverse,axial])*v[-3:])
    # An axisymmetric rigid ring precesses about fixed angular momentum.
    # Spin about its symmetry axis leaves the occupied material set invariant.
    frequency = np.linalg.norm(momentum)/transverse
    axis = momentum/np.linalg.norm(momentum) if frequency else np.array([0.,0.,1.])
    angles = 2*math.pi*np.arange(azimuths)/azimuths
    ring = MAJOR*(np.outer(np.cos(angles),axes[:,0])+np.outer(np.sin(angles),axes[:,1]))
    parallel = np.outer(ring@axis,axis)
    cross = np.cross(axis,ring)
    along = (HIGH-LOW)/np.linalg.norm(HIGH-LOW)
    normal = np.cross(along,[0.,1.,0.])
    gravity = np.array([0.,0.,-9.80665])
    horizon = .25
    # Every unsampled space-time centerline point lies in one of these
    # inflated spheres. Project the actual sphere/plane disk, not its center.
    rate_bound = np.linalg.norm(velocity)+9.80665*horizon+MAJOR*frequency
    pad = MAJOR*math.pi/azimuths + rate_bound*dt/2 + 1e-9
    radius = TUBE+pad
    minimum = math.inf
    first = last = None
    before = after = False
    for t in np.linspace(0,horizon,math.ceil(horizon/dt)+1):
        offset = parallel+(ring-parallel)*math.cos(frequency*t)+cross*math.sin(frequency*t)
        points = center + t*velocity + .5*t*t*gravity + offset - LOW
        heights = points@normal
        if t == 0.:before = bool(heights.max()+radius < 0)
        if t == horizon:after = bool(heights.min()-radius > 0)
        near = np.abs(heights)<=radius
        if not np.any(near):continue
        disk = np.sqrt(np.maximum(0.,radius**2-heights[near]**2))
        u = points[near]@along
        y = points[near,1]
        margin = np.minimum.reduce((u-disk,np.linalg.norm(HIGH-LOW)-u-disk,WIDTH/2-np.abs(y)-disk))
        minimum = min(minimum,float(np.min(margin)))
        if first is None:first = t
        last = t
    return dict(startBelow=before,endAbove=after,firstPossibleContactS=first,lastPossibleContactS=last,
                spaceTimePadM=pad,minimumCertifiedMarginM=minimum if first is not None else None,
                wholeRingPassageCertified=bool(before and after and first is not None and minimum>0),
                releaseFieldPositionM=center.tolist(),releaseFieldVelocityMps=velocity.tolist(),
                sensorEventObserved=False)


def assess(folder):
    receipt = json.loads((folder/"result.json").read_bytes())
    trace = folder/"trace.jsonl"
    if receipt["outcome"] != "UPWARD_FREE_RELEASE" or digest(trace) != receipt["traceSha256"]:
        raise ValueError("Requires an attained upward release and its original trace")
    with trace.open() as stream:
        for line in stream:
            last = json.loads(line)
    if last["wheelNormalN"] or last["beltNormalN"] or abs(last["time"]-receipt["end"])>1e-8:
        raise ValueError("Final state is inside the mechanism or missing")
    frame = WingFrame(receipt.get("firstMark", "inner"), receipt.get("fieldYawDeg", 90.))
    schema = ("blue-inner-wing-speaker-opening/1" if (frame.first_mark, frame.yaw_degrees)==("inner", 90.)
              else "blue-wing-speaker-opening/2")
    return dict(schema=schema,sourceTraceSha256=receipt["traceSha256"],
                firstMark=frame.first_mark, fieldYawDeg=frame.yaw_degrees,
                releaseS=last["time"],nominal=screen(last, frame=frame),refined=screen(last,1024,.000125,frame=frame),
                lateralMissControl=screen(last,lateral_offset=.75,frame=frame),verifiedSpeakerScores=0,
                limits=["One rigid field registration for the entire run",
                        "Rigid free flight under gravity; field-panel contact and airflow omitted",
                        "Nominal rectangular opening; downstream counter evidence required"])


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    result=assess(args.run)
    with args.output.open("x") as out:
        json.dump(result,out,indent=2,allow_nan=False)
        out.write("\n")
    print(json.dumps(result["refined"]))
