"""Attach a chassis-relative lift and shot replay to the Fusion display mesh."""
import argparse
import hashlib
from itertools import chain
import json
from pathlib import Path
import struct
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/pressure"))
from wing_field_frame import WingFrame

# Same robot-floor datum as fusion_visual_display. glTF uses metres and Y-up.
TO_GLTF = Rotation.from_euler("x", -90, degrees=True)
PIVOT = TO_GLTF.apply([.178, 0., .0254])
CLIP = "Lift and shoot"


def trace(folder, digest):
    path = folder / "trace.jsonl"
    if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
        raise ValueError("Trace differs from the verified scoring run")
    with path.open("rb") as stream:
        yield from map(json.loads, stream)


def recorded_poses(receipt, delivery, release, flight, acquisition=None):
    frame = WingFrame(receipt["firstMark"], receipt["fieldYawDeg"])
    field_rotation = Rotation.from_matrix(frame.rotation)
    poses = []
    endpoint = receipt.get("sensorS", receipt["openedByS"])
    stages = [(delivery, "carry"), (release, "shot"), (flight, "field")]
    if acquisition:
        stages.insert(0, (acquisition, "pickup"))
    for folder, key in stages:
        last = -float("inf")
        rows = trace(folder, receipt[key])
        if key == "pickup":
            initial = json.loads((folder / "result.json").read_bytes())
            rows = chain([dict(t=initial["start"], q=initial["initialQ"])], rows)
        for row in rows:
            t = row.get("time", row.get("t"))
            if key == "field" and t > endpoint + 1e-8:
                break
            q = row["q"]
            if key != "field":
                chassis = np.array([*q[:2], 0.])
                heading = Rotation.from_euler("z", q[2])
                height, pitch = q[3:5]
            center = np.array(q[-7:-4])
            orientation = Rotation.from_quat(q[-4:], scalar_first=True)
            if key == "field":
                center = frame.to_local(center)
                orientation = field_rotation.inv() * orientation
            note = TO_GLTF.apply(heading.inv().apply(center - chassis))
            angle = TO_GLTF * heading.inv() * orientation * TO_GLTF.inv()
            if key != "field":
                modules = q[9:17] if len(q) == 24 else None
            pose = (t, height, pitch, note, angle.as_quat(), modules)
            if poses and abs(poses[-1][0] - t) < 1e-8:
                if np.linalg.norm(poses[-1][3] - note) > 1e-5:
                    raise ValueError("Discontinuous NOTE position at field handoff")
                continue
            if t - last >= 1/90 or t >= receipt["releaseS"]-1e-9 and key == "shot":
                poses.append(pose)
                last = t
        if poses[-1][0] < pose[0] - 1e-8:
            poses.append(pose)
    if abs(poses[-1][0]-endpoint) > .00011:
        raise ValueError("Missing recorded field endpoint")
    return poses


def note_mesh():
    major, tube = .1524, .0254
    a, b = np.meshgrid(np.arange(64)*2*np.pi/64, np.arange(16)*2*np.pi/16, indexing="ij")
    normals = np.stack((np.cos(a)*np.cos(b), np.sin(b), -np.sin(a)*np.cos(b)), axis=-1)
    centers = major*np.stack((np.cos(a), np.zeros_like(a), -np.sin(a)), axis=-1)
    vertices = centers + tube*normals
    grid = np.arange(a.size).reshape(a.shape)
    nxt, up = np.roll(grid,-1,axis=0), np.roll(grid,-1,axis=1)
    diagonal = np.roll(nxt,-1,axis=1)
    faces = np.stack((grid,nxt,up,nxt,diagonal,up),axis=-1)
    return vertices.reshape(-1,3), normals.reshape(-1,3), faces.flatten()


def build(model_path, output, receipt_path, delivery, release, flight, *, registration=None, acquisition=None, sensors=None):
    raw = model_path.read_bytes()
    json_size = struct.unpack_from("<I",raw,12)[0]
    model = json.loads(raw[20:20+json_size])
    if "animations" in model:
        raise ValueError("Expected a static source model")
    binary = bytearray(raw[28+json_size:])
    receipt = json.loads(receipt_path.read_bytes())
    display_native = model["asset"]["extras"]["nativeSha256"]
    if (acquisition or sensors) and registration is None:
        raise ValueError("Acquisition replay requires a registered swerve run")
    if registration is None:
        if receipt["schema"] != "first-wing-full-field-score/1" or display_native != receipt["native"]:
            raise ValueError("Expected a verified first-wing score for this Fusion revision")
    else:
        pair = json.loads(registration.read_bytes())
        direct_r7 = receipt["schema"] == "r7-first-note-development/1"
        record = receipt["observation"] if direct_r7 else next(r for r in receipt["pairedDenseObservations"] if r["attempt"] == "D1")
        archived = receipt if direct_r7 else json.loads((ROOT/"sim/pressure/evidence/maxswerve-development-carry.json").read_bytes())["runs"]["D1"]
        carry = json.loads((delivery / "result.json").read_bytes())
        if direct_r7:
            from r7_dynamics import verify_binding
            if verify_binding(carry) != receipt["massBinding"] or record["sourceNativeSha256"] != display_native:
                raise ValueError("Expected the display archive and registered R7 mass case")
        elif receipt["schema"] != "maxswerve-development-first-shot/1" or pair["plantNative"] != record["sourceNativeSha256"]:
            raise ValueError("Unsupported scoring plant")
        if (pair["displayNative"] != display_native or
                pair["displayMesh"] != hashlib.sha256(raw).hexdigest() or
                pair["registeredBodyCount"] != 188 or not 0 <= pair["maximumSymmetricDifferenceMm3"] <= .001 or
                carry["nativeSha256"] != record["sourceNativeSha256"] or carry["trace"] != record["carryTrace"] or
                record["countedFloorNotes"] != 1):
            raise ValueError("Mismatched swerve geometry, registration or score")
        receipt = dict(firstMark=carry["firstMark"], fieldYawDeg=carry["fieldYawDeg"],
                       native=record["sourceNativeSha256"], carry=record["carryTrace"], shot=record["sourceTrace"],
                       field=record["fieldTrace"], releaseS=record["releaseS"], openedByS=record["opening"]["fullyThroughByS"])
        if sensors:
            if hashlib.sha256(sensors.read_bytes()).hexdigest() != record["sensorFile"]:
                raise ValueError("Sensor file differs from the scored field registration")
            receipt["sensorS"] = record["firstSimultaneousFullShadowS"]
        if acquisition:
            start = json.loads((acquisition/"result.json").read_bytes())
            if (archived["carryTrace"] != record["carryTrace"] or archived["pickupTrace"] != start["trace"] or
                    start["trace"] != carry["acquisitionTrace"] or start["end"] != carry["start"] or
                    start["outcome"] != "SEATED" or start["attempt"] != "D1" or start["start"] != 0 or
                    start["nativeSha256"] != carry["nativeSha256"] or start["pieceId"] != carry["pieceId"]):
                raise ValueError("Pickup differs from the verified D1 handoff")
            receipt.update(pickup=start["trace"], firstBilateralS=start["firstBilateral"], seatedS=start["end"])
    poses = recorded_poses(receipt, delivery, release, flight, acquisition)
    def accessor(values, kind, component=5126, bounds=False):
        array = np.asarray(values, dtype="<f4" if component==5126 else "<u4")
        if not np.isfinite(array).all():
            raise ValueError("Non-finite replay data")
        binary.extend(bytes(-len(binary)%4))
        model["bufferViews"].append(dict(buffer=0, byteOffset=len(binary), byteLength=array.nbytes))
        index = dict(bufferView=len(model["bufferViews"])-1, componentType=component, count=len(array), type=kind)
        if bounds:
            columns = array.reshape(len(array),-1)
            index.update(min=columns.min(axis=0).tolist(),max=columns.max(axis=0).tolist())
        model["accessors"].append(index)
        binary.extend(array.tobytes())
        return len(model["accessors"])-1

    vertices,normals,indices = note_mesh()
    material = len(model["materials"])
    model["materials"].append({"pbrMetallicRoughness":{"baseColorFactor":[1,.2,.012,1],"metallicFactor":0,"roughnessFactor":.85}})
    model["meshes"].append({"primitives":[{"attributes":{"POSITION":accessor(vertices,"VEC3",bounds=True),
                            "NORMAL":accessor(normals,"VEC3")},"indices":accessor(indices,"SCALAR",5125),"material":material}]})
    note = len(model["nodes"])
    model["nodes"].append(dict(name="NOTE",mesh=len(model["meshes"])-1, translation=poses[0][3].tolist(),rotation=poses[0][4].tolist()))
    model["scenes"][0]["nodes"].append(note)
    times = np.array([p[0]-poses[0][0] for p in poses])
    if not (np.diff(times)>0).all(): raise ValueError("Replay clock regressed")
    time_accessor = accessor(times,"SCALAR",bounds=True)
    clip = dict(name=CLIP, samplers=[], channels=[], extras={
        "sourceStartS":poses[0][0],"releaseS":receipt["releaseS"],"openingS":receipt["openedByS"],
        "carry":receipt["carry"],"shot":receipt["shot"],"field":receipt["field"],"native":receipt["native"],"displayNative":display_native})
    if acquisition:
        clip["extras"].update({key:receipt[key] for key in ("pickup","firstBilateralS","seatedS")})
    def channel(nodes, path, values, interpolation="LINEAR"):
        idx = accessor(values,"VEC4" if path=="rotation" else "VEC3")
        sampler = len(clip["samplers"])
        clip["samplers"].append(dict(input=time_accessor,output=idx,interpolation=interpolation))
        for node in nodes:
            clip["channels"].append(dict(sampler=sampler,target=dict(node=node,path=path)))
    travel = np.array([[0,p[1],0] for p in poses])
    pitch = Rotation.from_euler("z",np.array([p[2] for p in poses])[:,None])
    def group(name):
        nodes=[i for i,n in enumerate(model["nodes"]) if n.get("name","").split("-")[0]==name]
        if not nodes: raise ValueError("Missing native motion group "+name)
        return nodes
    channel(group("carriage"),"translation",travel)
    channel(group("cradle"),"translation",PIVOT+travel-pitch.apply(PIVOT))
    channel(group("cradle"),"rotation",pitch.as_quat())
    if registration is not None:
        from swerve_wheels import animate_wheels
        animate_wheels(model, pair["wheels"], [p[5] for p in poses], accessor, channel)
    channel([note],"translation",[p[3] for p in poses])
    quats = np.array([p[4] for p in poses])
    for i in range(1,len(quats)):
        if np.dot(quats[i-1],quats[i]) < 0: quats[i] *= -1
    channel([note],"rotation",quats)
    if sensors:
        from swerve_modules import primitive
        field = json.loads(sensors.read_bytes())
        frame = WingFrame(receipt["firstMark"], receipt["fieldYawDeg"])
        with (release/"trace.jsonl").open() as stream:
            for line in stream:
                last = line
        released = json.loads(last)
        if abs(released["time"] - receipt["releaseS"]) > 1e-8:
            raise ValueError("Wrong chassis pose for scoring references")
        q = released["q"]
        body = Rotation.from_euler("z", q[2]).inv()
        def to_scene(point):
            return TO_GLTF.apply(body.apply(frame.to_local(point) - [q[0],q[1],0]))
        guides = len(model["nodes"])
        model["nodes"].append(dict(name="Speaker measurement guides",children=[],scale=[0,0,0]))
        model["scenes"][0]["nodes"].append(guides)
        def segment(name, a, b, mesh, diameter):
            delta = b-a
            rotation, _ = Rotation.align_vectors([delta/np.linalg.norm(delta)], [[0,1,0]])
            model["nodes"][guides]["children"].append(len(model["nodes"]))
            model["nodes"].append(dict(name=name,mesh=mesh,rotation=rotation.as_quat().tolist(),
                translation=((a+b)/2).tolist(),scale=[diameter,float(np.linalg.norm(delta)),diameter]))
        rim = primitive(model, accessor, "cylinder", ("Opening reference", .85,.87,.93))
        beam = primitive(model, accessor, "cylinder", ("Counter sightline", .12,.72,.78))
        corners = [to_scene(pt) for pt in field["nominalOpeningCornersM"]]
        for i, a in enumerate(corners):
            segment("Nominal opening edge",a,corners[(i+1)%4],rim,.012)
        ends = {entry["key"]:to_scene(entry["caseCenterM"]) for entry in field["sensorCaseEnds"]}
        for i, (a,b) in enumerate(field["beamPairs"],1):
            segment(f"Sensor pair {i}",ends[a],ends[b],beam,.005)
        visible = np.repeat((times >= receipt["releaseS"]-poses[0][0]-1e-8)[:,None],3,axis=1)
        channel([guides], "scale", visible, interpolation="STEP")
        clip["extras"].update(sensorS=receipt["sensorS"], counterCenter=np.mean(list(ends.values()),axis=0).tolist(),
                             openingCenter=np.mean(corners,axis=0).tolist())
    model["animations"]=[clip]
    model["buffers"][0]["byteLength"]=len(binary)
    header=json.dumps(model,separators=(",",":"),allow_nan=False).encode()
    header+=b" "*(-len(header)%4)
    output.write_bytes(struct.pack("<4sIIII",b"glTF",2,28+len(header)+len(binary),len(header),0x4e4f534a)+header
                       +struct.pack("<II",len(binary),0x004e4942)+binary)
    print(f"{len(times)} recorded poses over {times[-1]:.4f} s; release at {receipt['releaseS']-poses[0][0]:.4f} s")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ("model_path","output","receipt","delivery","release","flight"):
        parser.add_argument(arg,type=Path)
    parser.add_argument("--registered-motion", type=Path, help="Verified moving-mesh registration to the swerve run")
    parser.add_argument("--acquisition", type=Path, help="Prepend the registered D1 floor pickup from the initial state")
    parser.add_argument("--sensors", type=Path, help="Extend through the observed counter event and display registered measurement guides")
    args=parser.parse_args()
    build(args.model_path,args.output,args.receipt,args.delivery,args.release,args.flight,registration=args.registered_motion,
          acquisition=args.acquisition,sensors=args.sensors)
