"""Attach a chassis-relative lift and shot replay to the Fusion display mesh."""
import argparse
import hashlib
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


def recorded_poses(receipt, delivery, release, flight):
    frame = WingFrame(receipt["firstMark"], receipt["fieldYawDeg"])
    field_rotation = Rotation.from_matrix(frame.rotation)
    poses = []
    for folder, key in ((delivery, "carry"), (release, "shot"), (flight, "field")):
        last = -float("inf")
        for row in trace(folder, receipt[key]):
            t = row.get("time", row.get("t"))
            if key == "field" and t > receipt["openedByS"] + 1e-8:
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
    if abs(poses[-1][0]-receipt["openedByS"]) > .00011:
        raise ValueError("Missing scored opening crossing")
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


def build(model_path, output, receipt_path, delivery, release, flight, *, registration=None):
    raw = model_path.read_bytes()
    json_size = struct.unpack_from("<I",raw,12)[0]
    model = json.loads(raw[20:20+json_size])
    if "animations" in model:
        raise ValueError("Expected a static source model")
    binary = bytearray(raw[28+json_size:])
    receipt = json.loads(receipt_path.read_bytes())
    display_native = model["asset"]["extras"]["nativeSha256"]
    if registration is None:
        if receipt["schema"] != "first-wing-full-field-score/1" or display_native != receipt["native"]:
            raise ValueError("Expected a verified first-wing score for this Fusion revision")
    else:
        pair = json.loads(registration.read_bytes())
        record = next(r for r in receipt["pairedDenseObservations"] if r["attempt"] == "D1")
        carry = json.loads((delivery / "result.json").read_bytes())
        if (receipt["schema"] != "maxswerve-development-first-shot/1" or
                pair["displayNative"] != display_native or pair["plantNative"] != record["sourceNativeSha256"] or
                pair["displayMesh"] != hashlib.sha256(raw).hexdigest() or
                pair["registeredBodyCount"] != 188 or not 0 <= pair["maximumSymmetricDifferenceMm3"] <= .001 or
                carry["nativeSha256"] != record["sourceNativeSha256"] or carry["trace"] != record["carryTrace"] or
                record["countedFloorNotes"] != 1):
            raise ValueError("Mismatched swerve geometry, registration or score")
        receipt = dict(firstMark=carry["firstMark"], fieldYawDeg=carry["fieldYawDeg"],
                       native=record["sourceNativeSha256"], carry=record["carryTrace"], shot=record["sourceTrace"],
                       field=record["fieldTrace"], releaseS=record["releaseS"], openedByS=record["opening"]["fullyThroughByS"])
    poses = recorded_poses(receipt, delivery, release, flight)
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
    def channel(nodes, path, values):
        idx = accessor(values,"VEC4" if path=="rotation" else "VEC3")
        sampler = len(clip["samplers"])
        clip["samplers"].append(dict(input=time_accessor,output=idx,interpolation="LINEAR"))
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
    args=parser.parse_args()
    build(args.model_path,args.output,args.receipt,args.delivery,args.release,args.flight,registration=args.registered_motion)
