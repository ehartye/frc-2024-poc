"""Resolve swept native wheel-shell intervals against the installed annular material."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import digest, load
from turning_sweep import NativeTurnPath


def shell_dimension(body):
    # The original wheel shell is the bored extrusion in cad/assembly/build_robot.py.
    b=np.array(body["boundsMm"]).reshape(2,3)
    half=(b[1]-b[0])/2
    volume=math.pi*(46.99**2-42.2275**2)*31.75
    if (body["motionGroup"]!="fixed" or not body["name"].endswith("original wheel shell representation")
            or not np.allclose(half,[46.99,15.875,46.99],rtol=0.,atol=.00001)
            or abs(volume-body["volumeMm3"])>.001):
        raise ValueError("Expected the registered rigid annular shell")
    return b.mean(axis=0)


def ring_distances(center,normal,shell,cosine,sine):
    reference=np.eye(3)[np.argmin(abs(normal))]
    a=np.cross(normal,reference)
    a/=np.linalg.norm(a)
    b=np.cross(normal,a)
    p=center-shell+152.4*(cosine*a+sine*b)
    radial=np.hypot(p[:,0],p[:,2])
    dr=np.maximum.reduce([42.2275-radial,radial-46.99,np.zeros(len(p))])
    dy=np.maximum(0.,abs(p[:,1])-15.875)
    return np.hypot(dr,dy)-25.4


def refine(shot, inventory_file, native_file, samples=8192, subdivisions=8):
    packet=load(shot/"result.json")
    native=load(native_file)
    inventory=load(inventory_file)
    if (packet["traceStride"]!=1 or digest(shot/"trace.jsonl")!=packet["traceSha256"]
            or native["source"]!=packet["traceSha256"] or packet["nativeSha256"]!=inventory["sourceSha256"]):
        raise ValueError("Mismatched release or native sweep")
    bodies={r["key"]:r for r in inventory["inventory"]}
    with (shot/"trace.jsonl").open("rb") as stream:
        rows=[packet["initialQ"],*[json.loads(line)["q"] for line in stream]]
    if len(rows)-1!=native["steps"] or abs((len(rows)-1)*packet["dt"]+packet["start"]-packet["end"])>1e-7:
        raise ValueError("Incomplete native sweep")
    q=np.asarray(rows)
    path=NativeTurnPath.from_poses(np.column_stack((q[:,:5],q[:,-7:])),packet["dt"])
    centers,normals=path.frames["fixed"]
    center_error,normal_error=path.errors["fixed"]
    angles=np.arange(samples)[:,None]*2*np.pi/samples
    cosine,sine=np.cos(angles),np.sin(angles)
    arc_error=152.4*np.pi/samples
    worst=math.inf
    remaining=[]
    for lo,hi,keys in native["unresolved"]:
        if hi!=lo+1:
            raise ValueError("Refinement requires single-step native witnesses")
        dc=centers[hi]-centers[lo]
        dn=normals[hi]-normals[lo]
        shells=[shell_dimension(bodies[key]) for key in keys]
        lower=math.inf
        for j in range(subdivisions):
            u=(j+.5)/subdivisions
            center=centers[lo]+u*dc
            n=normals[lo]+u*dn
            length=np.linalg.norm(n)
            normal=n/length
            # Path frame errors enclose the actual center and normal about the
            # endpoint chord. The 1-Lipschitz distance to closed material bounds
            # every intermediate time and every unsampled point on the major circle.
            pad=(center_error[lo]+np.linalg.norm(dc)/(2*subdivisions)+arc_error+.002
                 +152.4*(normal_error[lo]+abs(length-1)+np.linalg.norm(dn)/(2*subdivisions)))
            for shell in shells:
                lower=min(lower,float(ring_distances(center,normal,shell,cosine,sine).min())-pad)
        worst=min(worst,lower)
        if lower<=0.:
            remaining.append([lo,hi,keys,lower])
    return dict(native=packet["nativeSha256"],trace=packet["traceSha256"],nativeSweep=digest(native_file),
                initialUnresolved=len(native["unresolved"]),unresolved=remaining,samples=samples,
                substeps=subdivisions,minimumLowerBoundMm=worst if math.isfinite(worst) else None)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("shot","inventory","native_sweep","output"):
        p.add_argument(key,type=Path)
    args=p.parse_args()
    result=refine(args.shot,args.inventory,args.native_sweep)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(result["initialUnresolved"],len(result["unresolved"]),result["minimumLowerBoundMm"])
