"""Continue an attained moving NOTE into the 2024 blue Speaker on its original clock."""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from speaker_surfaces import SpeakerSurfaces, digest
from wing_field_frame import WingFrame, WING_MARKS_INCHES
from wing_speaker_passage import MAJOR, TUBE, rotation


def attained_state(pickup, model, data, frame=None):
    receipt = json.loads((pickup / "result.json").read_bytes())
    trace = pickup / "trace.jsonl"
    if receipt["outcome"] != "UPWARD_FREE_RELEASE" or digest(trace) != receipt["traceSha256"]:
        raise ValueError("Expected the original attained release")
    with trace.open("rb") as stream:
        for line in stream:
            last = line
    row = json.loads(last)
    if row["beltNormalN"] or row["wheelNormalN"] or abs(row["time"]-receipt["end"]) > 1e-8:
        raise ValueError("NOTE is still in contact or the trace is incomplete")
    frame = frame or WingFrame()
    if "firstMark" in receipt and (receipt["firstMark"],receipt["fieldYawDeg"])!=(frame.first_mark,frame.yaw_degrees):
        raise ValueError("Release and field trajectory use different registrations")
    base = frame.rotation
    data.qpos[:3] = frame.to_field(row["q"][-7:-4])
    mujoco.mju_mat2Quat(data.qpos[3:], (base @ rotation(row["q"][-4:])).ravel())
    data.qvel[:3] = base @ row["v"][-6:-3]
    data.qvel[3:] = row["v"][-3:]
    data.time = row["time"]
    mujoco.mj_forward(model, data)
    return receipt


def field_wrench(surfaces, points, center, velocity, omega, args):
    wrench = np.zeros(6)
    contact = {}
    maximum_depth = 0.
    for index, selected in surfaces.candidates(center, points, MAJOR, TUBE):
        part, distance, nearest = surfaces.nearest(index, points[selected])
        touching = distance < TUBE
        if not np.any(touching):
            continue
        nodes = points[selected[touching]]
        gap = distance[touching]
        depth = TUBE-gap
        maximum_depth = max(maximum_depth, float(max(depth)))
        if maximum_depth > .85*TUBE:
            raise ValueError("Excessive indentation invalidates closest-surface orientation")
        normal = (nodes-nearest[touching])/gap[:, None]
        arm = nodes-center-normal*(TUBE-depth[:, None]/2)
        relative = velocity + np.cross(omega, arm)
        if part["name"] == "GE-24355":
            relative -= args.belt_speed * np.cross(normal, [0., 1., 0.])
        normal_speed = (relative*normal).sum(axis=1)
        normal_load = np.maximum(0., args.stiffness*depth-args.damping*normal_speed)/len(points)
        tangent = relative-normal_speed[:, None]*normal
        slip = np.maximum(.05, np.linalg.norm(tangent, axis=1))
        force = normal_load[:, None]*(normal-args.friction*tangent/slip[:, None])
        wrench[:3] += force.sum(axis=0)
        wrench[3:] += np.cross(arm, force).sum(axis=0)
        if normal_load.sum() > 0:
            contact[part["key"]] = float(normal_load.sum())
    return wrench, contact, maximum_depth


def collection_floor(points, center, velocity, omega, args):
    """Passive flat collection floor at carpet datum on the practice field."""
    depth=TUBE-points[:,2]
    mask=depth>0
    if not mask.any():
        return np.zeros(6),0.
    if center[0]+MAJOR+TUBE>=0:
        raise ValueError("Collection trajectory reached the playing side of the end wall")
    arm=points[mask]-center-[0.,0.,TUBE]
    speed=velocity+np.cross(omega,arm)
    normal=np.maximum(0.,args.stiffness*depth[mask]-args.damping*speed[:,2])/len(points)
    friction=-args.friction*normal[:,None]*speed[:,:2]/np.maximum(.05,np.linalg.norm(speed[:,:2],axis=1))[:,None]
    force=np.column_stack((friction,normal))
    return np.r_[force.sum(axis=0),np.cross(arm,force).sum(axis=0)],float(max(depth[mask]))


def run(args):
    controls = [args.dt, args.horizon, args.stiffness, args.damping, args.friction, args.belt_speed]
    if not np.isfinite(controls).all() or min(controls[:3]) <= 0 or min(controls[3:5]) < 0 or args.nodes < 16:
        raise ValueError("Invalid field study parameters")
    if args.output.exists():
        raise FileExistsError(args.output)
    config = Path(__file__).resolve().parents[1] / "component-launcher/six-drive-config.json"
    mass = json.loads(config.read_bytes())["pieceMassKg"]
    ix = mass*(MAJOR**2/2 + 5*TUBE**2/8)
    iz = mass*(MAJOR**2 + 3*TUBE**2/4)
    model = mujoco.MjModel.from_xml_string(f"""<mujoco>
      <option timestep="{args.dt}" gravity="0 0 -9.80665" integrator="implicitfast"/>
      <worldbody><body><freejoint/><inertial pos="0 0 0" mass="{mass}"
        diaginertia="{ix} {ix} {iz}"/></body></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    frame = WingFrame(args.first_mark,args.field_yaw_deg)
    release = attained_state(args.pickup, model, data, frame)
    surfaces = SpeakerSurfaces(args.field)
    release_time = data.time
    resume = getattr(args,"resume_field",None)
    if resume:
        old=json.loads((resume/"result.json").read_bytes())
        trace=resume/"trace.jsonl"
        params=old["parameters"]
        keys=("dt","nodes","first_mark","field_yaw_deg","belt_speed","stiffness","damping","friction")
        if (old["sourceTrace"]!=release["traceSha256"] or old["field"]!=surfaces.pins
                or old["trace"]!=digest(trace) or old["releaseTimeS"]!=release_time
                or any(params[k]!=getattr(args,k) for k in keys)):
            raise ValueError("Cannot continue a different field trajectory or contact model")
        with trace.open("rb") as stream:
            for line in stream:
                last=line
        terminal=json.loads(last)
        if params.get("collection_floor",False) and not args.collection_floor:
            raise ValueError("Cannot remove the floor during collection")
        if args.collection_floor and not params.get("collection_floor",False):
            with trace.open("rb") as stream:
                if any(json.loads(line)["q"][2]<MAJOR+TUBE for line in stream):
                    raise ValueError("Collection floor could have affected the previous trajectory")
        if abs(terminal["t"]-old["endS"])>1e-8:
            raise ValueError("Missing terminal field pose")
        data.qpos[:]=terminal["q"]
        data.qvel[:]=terminal["v"]
        data.time=terminal["t"]
        mujoco.mj_forward(model,data)
    start = data.time
    angles = 2*math.pi*np.arange(args.nodes)/args.nodes
    local = MAJOR*np.stack((np.cos(angles), np.sin(angles), np.zeros(args.nodes)), axis=1)
    first = {}
    worst = 0.
    stride = 1 if getattr(args, "trace_every_step", False) else max(1, round(.001/args.dt))
    steps = math.ceil(args.horizon/args.dt)
    args.output.mkdir()
    trace = args.output / "trace.jsonl"
    try:
        with trace.open("x") as stream:
            for tick in range(steps+1):
                mujoco.mj_step1(model, data)
                axes = data.xmat[1].reshape(3, 3)
                center = data.qpos[:3]
                points=center+local @ axes.T
                omega=axes @ data.qvel[3:]
                wrench, touch, depth = field_wrench(surfaces, points, center,data.qvel[:3],omega,args)
                if getattr(args,"collection_floor",False):
                    floor_wrench,floor_depth=collection_floor(points,center,data.qvel[:3],omega,args)
                    wrench+=floor_wrench
                    if floor_depth:
                        touch["collectionFloor"]=float(floor_wrench[2])
                        depth=max(depth,floor_depth)
                for key in touch:
                    first.setdefault(key, float(data.time))
                worst = max(worst, depth)
                terminal = tick == steps or (center[2] < .2 and not getattr(args,"collection_floor",False))
                if data.time-start < .35 or tick % stride == 0 or terminal:
                    stream.write(json.dumps(dict(t=data.time, q=data.qpos.tolist(), v=data.qvel.tolist(),
                                                 contacts=touch, indentationM=depth), allow_nan=False)+"\n")
                if terminal:
                    break
                data.xfrc_applied[1] = wrench
                mujoco.mj_step2(model, data)
    finally:
        surfaces.close()
    result = dict(schema="speaker-field-contact/1", sourceTrace=release["traceSha256"],
                  nativeSha256=release["nativeSha256"], field=surfaces.pins, trace=digest(trace),
                  runner=digest(Path(__file__)), surfaces=digest(Path(__file__).with_name("speaker_surfaces.py")),
                  firstContactS=first, maximumIndentM=worst, releaseTimeS=release_time, segmentStartS=start,endS=data.time,
                  parameters={k: v for k, v in vars(args).items() if not isinstance(v, Path)})
    if resume:
        result["previousFieldTrace"]=old["trace"]
    (args.output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(f"Completed {data.time-start:.3f} seconds with {len(first)} contacted occurrences")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("pickup", "field", "output"):
        parser.add_argument(arg, type=Path)
    parser.add_argument("--dt", type=float, default=.0002)
    parser.add_argument("--nodes", type=int, default=128)
    parser.add_argument("--first-mark", choices=WING_MARKS_INCHES, default="inner")
    parser.add_argument("--field-yaw-deg", type=float, default=90.)
    parser.add_argument("--belt-speed", type=float, default=1.5, help="signed top belt speed in m/s toward the counter")
    parser.add_argument("--trace-every-step", action="store_true", help="save every field pose for comparison with the moving robot")
    parser.add_argument("--horizon", type=float, default=4.)
    parser.add_argument("--resume-field",type=Path,help="Continue a saved flight from its terminal pose and velocity")
    parser.add_argument("--collection-floor",action="store_true",help="Keep tracking the discharged NOTE on a passive floor behind the end wall")
    parser.add_argument("--stiffness", type=float, default=1.2e6)
    parser.add_argument("--damping", type=float, default=450.)
    parser.add_argument("--friction", type=float, default=.45)
    run(parser.parse_args())
