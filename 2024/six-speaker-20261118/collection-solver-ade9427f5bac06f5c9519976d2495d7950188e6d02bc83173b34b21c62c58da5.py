"""Continue already-scored NOTES together through a shared practice-field collection area."""
import argparse
from contextlib import ExitStack
import itertools
import json
import math
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from run_speaker_field import collection_floor, field_wrench
from speaker_surfaces import SpeakerSurfaces, digest
from wing_speaker_passage import MAJOR, TUBE


def pair_wrenches(a, b, parameters):
    """Integrate both centerlines symmetrically, with equal and opposite forces at each contact."""
    forces = np.zeros((2, 6))
    depth_max = 0.
    if np.linalg.norm(a.center-b.center) > 2*(MAJOR+TUBE):
        return forces, depth_max
    for index, (left, right) in enumerate(((a,b),(b,a))):
        delta = left.points-right.center
        axis = right.rotation[:,2]
        plane = delta-(delta@axis)[:,None]*axis
        radius = np.linalg.norm(plane,axis=1)
        # A point on the symmetry axis is at least one major radius from the circle.
        near = (np.hypot(radius-MAJOR,delta@axis)<2*TUBE)
        if not near.any():
            continue
        closest = right.center+MAJOR*plane[near]/radius[near,None]
        gap_vector = left.points[near]-closest
        distance = np.linalg.norm(gap_vector,axis=1)
        depth = 2*TUBE-distance
        depth_max = max(depth_max, float(depth.max()))
        if depth_max>.85*TUBE:
            raise ValueError("Excessive ring-to-ring indentation")
        normal = gap_vector/distance[:,None]
        point = (left.points[near]+closest)/2
        arm_a,arm_b = point-left.center,point-right.center
        velocity = left.velocity+np.cross(left.omega,arm_a)-right.velocity-np.cross(right.omega,arm_b)
        vn = np.sum(velocity*normal,axis=1)
        load = np.maximum(0., parameters.stiffness*depth-parameters.damping*vn)/(2*len(left.points))
        tangent = velocity-vn[:,None]*normal
        slip = np.maximum(.05,np.linalg.norm(tangent,axis=1))
        force = load[:,None]*(normal-parameters.friction*tangent/slip[:,None])
        forces[index,:3] += force.sum(axis=0)
        forces[1-index,:3] -= force.sum(axis=0)
        forces[index,3:] += np.cross(arm_a,force).sum(axis=0)
        forces[1-index,3:] -= np.cross(arm_b,force).sum(axis=0)
    return forces,depth_max


def terminal(folder):
    receipt=json.loads((folder/"result.json").read_bytes())
    path=folder/"trace.jsonl"
    if not receipt["parameters"]["trace_every_step"] or digest(path)!=receipt["trace"]:
        raise ValueError("Collection needs the original dense trajectory")
    with path.open("rb") as stream:
        for line in stream:
            last=line
            if not receipt["parameters"].get("collection_floor") and json.loads(line)["q"][2]<MAJOR+TUBE:
                raise ValueError("A missing collection floor could have changed this trajectory")
    row=json.loads(last)
    if abs(row["t"]-receipt["endS"])>1e-8:
        raise ValueError("Incomplete incoming trajectory")
    return receipt,row


def run(inputs,field,output,duration):
    if len(inputs)<2 or not math.isfinite(duration) or duration<=0:
        raise ValueError("Expected multiple NOTES and a positive collection duration")
    sources,states=zip(*(terminal(folder) for folder in inputs))
    ids=[r["sourceTrace"] for r in sources]
    if len(set(ids))!=len(ids):
        raise ValueError("Each collection body must have a distinct release")
    reference=sources[0]
    params=SimpleNamespace(**reference["parameters"])
    keys=("dt","nodes","stiffness","damping","friction","belt_speed","first_mark","field_yaw_deg")
    for rc in sources[1:]:
        if (rc["field"]!=reference["field"] or rc["nativeSha256"]!=reference["nativeSha256"]
            or any(rc["parameters"][k]!=getattr(params,k) for k in keys)
            or abs(rc["endS"]-reference["endS"])>1e-8):
            raise ValueError("Collection participants must share a field, clock and contact model")
    mass=json.loads((Path(__file__).resolve().parents[1]/"component-launcher/six-drive-config.json").read_bytes())["pieceMassKg"]
    ix=mass*(MAJOR**2/2+5*TUBE**2/8)
    iz=mass*(MAJOR**2+3*TUBE**2/4)
    body=f'<body><freejoint/><inertial pos="0 0 0" mass="{mass}" diaginertia="{ix} {ix} {iz}"/></body>'
    model=mujoco.MjModel.from_xml_string(f'<mujoco><option timestep="{params.dt}" gravity="0 0 -9.80665" integrator="implicitfast"/><worldbody>{body*len(ids)}</worldbody></mujoco>')
    data=mujoco.MjData(model)
    data.qpos[:]=np.concatenate([row["q"] for row in states])
    data.qvel[:]=np.concatenate([row["v"] for row in states])
    data.time=reference["endS"]
    angles=np.arange(params.nodes)*2*np.pi/params.nodes
    ring=MAJOR*np.column_stack((np.cos(angles),np.sin(angles),np.zeros(params.nodes)))
    steps=round(duration/params.dt)
    if steps<1:
        raise ValueError("Collection duration is shorter than one timestep")
    folders=[output/str(i) for i in range(len(ids))]
    first_contact={}
    max_depth=0.
    with ExitStack() as stack:
        surfaces=SpeakerSurfaces(field)
        stack.callback(surfaces.close)
        if surfaces.pins!=reference["field"]:
            raise ValueError("Different field geometry")
        output.mkdir()
        files=[]
        for folder in folders:
            folder.mkdir()
            files.append(stack.enter_context((folder/"trace.jsonl").open("x")))
        for tick in range(steps+1):
            mujoco.mj_step1(model,data)
            bodies=[]
            wrenches=[]
            for i in range(len(ids)):
                rot=data.xmat[i+1].reshape(3,3)
                q=data.qpos[7*i:7*i+7]
                v=data.qvel[6*i:6*i+6]
                shape=SimpleNamespace(center=q[:3],points=q[:3]+ring@rot.T,rotation=rot,
                                      velocity=v[:3],omega=rot@v[3:])
                wrench,contact,_=field_wrench(surfaces,shape.points,shape.center,shape.velocity,shape.omega,params)
                floor,depth=collection_floor(shape.points,shape.center,shape.velocity,shape.omega,params)
                if depth:contact["collectionFloor"]=float(floor[2])
                bodies.append(shape)
                wrenches.append(wrench+floor)
                files[i].write(json.dumps(dict(t=data.time if tick else states[i]["t"],q=q.tolist(),v=v.tolist(),
                                              contacts=contact),allow_nan=False)+"\n")
            for i,j in itertools.combinations(range(len(ids)),2):
                forces,depth=pair_wrenches(bodies[i],bodies[j],params)
                if depth>0:
                    first_contact.setdefault(f"{i}-{j}",data.time)
                    max_depth=max(max_depth,depth)
                    wrenches[i]+=forces[0]
                    wrenches[j]+=forces[1]
            if tick!=steps:
                data.xfrc_applied[1:]=wrenches
                mujoco.mj_step2(model,data)
    hashes=[digest(folder/"trace.jsonl") for folder in folders]
    manifest=output/"manifest.json"
    manifest.write_text(json.dumps(dict(schema="coupled-note-collection/1",shots=ids,traces=hashes,
        inputs=[r["trace"] for r in sources],start=reference["endS"],end=data.time,firstPairContactS=first_contact,
        maximumPairIndentationM=max_depth,solver=digest(Path(__file__))),indent=2))
    for folder,rc,pin in zip(folders,sources,hashes):
        result=dict(sourceTrace=rc["sourceTrace"],nativeSha256=rc["nativeSha256"],field=rc["field"],trace=pin,
                    previousFieldTrace=rc["trace"],segmentStartS=rc["endS"],releaseTimeS=rc["releaseTimeS"],endS=data.time,
                    parameters={**rc["parameters"],"collection_floor":True,"horizon":duration},
                    coupledManifest=digest(manifest))
        (folder/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False))
    return first_contact,max_depth


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs",nargs="+",type=Path,required=True)
    parser.add_argument("--field",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--duration",type=float,required=True)
    a=parser.parse_args()
    print(run(a.inputs,a.field,a.output,a.duration))
