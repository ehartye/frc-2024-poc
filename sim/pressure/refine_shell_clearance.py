"""Bound release clearance to registered wheel shells and side-pinch belts."""
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


def ring_points(center,normal,cosine,sine):
    reference=np.eye(3)[np.argmin(abs(normal))]
    a=np.cross(normal,reference)
    a/=np.linalg.norm(a)
    b=np.cross(normal,a)
    return center+152.4*(cosine*a+sine*b)


def ring_distances(center,normal,shell,cosine,sine):
    p=ring_points(center-shell,normal,cosine,sine)
    radial=np.hypot(p[:,0],p[:,2])
    dr=np.maximum.reduce([42.2275-radial,radial-46.99,np.zeros(len(p))])
    dy=np.maximum(0.,abs(p[:,1])-15.875)
    return np.hypot(dr,dy)-25.4


def belt_envelope(body):
    # belt_x in cad/assembly/intake_components.py extrudes two nested stadiums.
    # The filled outer stadium conservatively includes the entire installed belt.
    config = load(Path(__file__).resolve().parents[2]/"cad/assembly/intake_components.json")["pinch"]
    pitch_radius = config["teeth"]*config["pitch"]/(2*math.pi)
    radius = pitch_radius+1.
    length = (config["beltTeethHypothesis"]-config["teeth"])*config["pitch"]/2
    width = config["beltWidth"]
    volume = width*(4*length+4*math.pi*pitch_radius)
    bounds = np.array(body["boundsMm"]).reshape(2,3)
    if (body["motionGroup"]!="cradle" or body["name"] not in
            ("intake candidate side pinch belt -1", "intake candidate side pinch belt 1")
            or not np.allclose(bounds[1]-bounds[0], [length+2*radius,2*radius,width],rtol=0.,atol=.00001)
            or abs(volume-body["volumeMm3"])>.001):
        raise ValueError("Expected the registered side-pinch belt extrusion")
    return bounds.mean(axis=0), length/2, radius, width/2


def belt_distances(center,normal,belt,cosine,sine):
    origin,half_length,radius,half_width = belt
    p = ring_points(center-origin,normal,cosine,sine)
    radial = np.hypot(np.maximum(0.,abs(p[:,0])-half_length),p[:,1])
    return np.hypot(np.maximum(0.,radial-radius),np.maximum(0.,abs(p[:,2])-half_width))-25.4


def interval_bound(path,lo,hi,group,distances,shape,angles,subdivisions):
    centers,normals=path.frames[group]
    center_error,normal_error=path.errors[group]
    cosine,sine=angles
    arc_error=152.4*math.pi/len(cosine)
    dc=centers[hi]-centers[lo]
    dn=normals[hi]-normals[lo]
    lower=math.inf
    for j in range(subdivisions):
        u=(j+.5)/subdivisions
        center=centers[lo]+u*dc
        n=normals[lo]+u*dn
        length=np.linalg.norm(n)
        # Distance to a closed solid is 1-Lipschitz. Bound every ring azimuth
        # and the deviation of its moving center and axis from this midpoint.
        pad=(center_error[lo]+np.linalg.norm(dc)/(2*subdivisions)+arc_error+.002
             +152.4*(normal_error[lo]+abs(length-1)+np.linalg.norm(dn)/(2*subdivisions)))
        lower=min(lower,float(distances(center,n/length,shape,cosine,sine).min())-pad)
    return lower


def refine(shot, inventory_file, native_file, samples=8192, subdivisions=8, refine_below=0.):
    if samples < 3 or subdivisions < 1:
        raise ValueError("Refinement requires at least three ring samples and one time subdivision")
    if not math.isfinite(refine_below) or refine_below<0:
        raise ValueError("Expected a finite nonnegative clearance threshold")
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
    angles=np.arange(samples)[:,None]*2*np.pi/samples
    grid=(np.cos(angles),np.sin(angles))
    coarse_angles=np.arange(min(8192,samples))[:,None]*2*np.pi/min(8192,samples)
    coarse_grid=(np.cos(coarse_angles),np.sin(coarse_angles))
    shapes={}
    worst=math.inf
    remaining=[]
    for lo,hi,keys in native["unresolved"]:
        if hi!=lo+1:
            raise ValueError("Refinement requires single-step native witnesses")
        lower=math.inf
        for key in keys:
            body=bodies[key]
            if key not in shapes:
                if body["name"].startswith("intake candidate side pinch belt "):
                    shapes[key]=(belt_distances,belt_envelope(body))
                else:
                    shapes[key]=(ring_distances,shell_dimension(body))
            distances,shape=shapes[key]
            gap=interval_bound(path,lo,hi,body["motionGroup"],distances,shape,coarse_grid,min(8,subdivisions))
            if gap<=refine_below and (samples>8192 or subdivisions>8):
                gap=max(gap,interval_bound(path,lo,hi,body["motionGroup"],distances,shape,grid,subdivisions))
            lower=min(lower,gap)
        worst=min(worst,lower)
        if lower<=0.:
            remaining.append([lo,hi,keys,lower])
    return dict(native=packet["nativeSha256"],trace=packet["traceSha256"],nativeSweep=digest(native_file),
                initialUnresolved=len(native["unresolved"]),unresolved=remaining,samples=samples,
                substeps=subdivisions,refineBelowMm=refine_below,minimumLowerBoundMm=worst if math.isfinite(worst) else None)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("shot","inventory","native_sweep","output"):
        p.add_argument(key,type=Path)
    p.add_argument("--samples", type=int, default=8192)
    p.add_argument("--subdivisions", type=int, default=8)
    p.add_argument("--refine-below-mm", type=float, default=0.,
                   help="Recheck already-clear intervals below this reported clearance")
    args=p.parse_args()
    result=refine(args.shot,args.inventory,args.native_sweep,args.samples,args.subdivisions,args.refine_below_mm)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(result["initialUnresolved"],len(result["unresolved"]),result["minimumLowerBoundMm"])
