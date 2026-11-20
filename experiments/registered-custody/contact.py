"""Signed distance from registered CAD triangles to a floor NOTE over a straight axial stroke."""
import hashlib
import json
import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MAJOR = 152.4
TUBE = 25.4
SEAT = 241.5
FRONT = 538.612 + 177.8 + 25.4
TREAD = "R7 cradle / R7 structure"


def cradle_components(triangles):
    bounds = json.loads((HERE / "cradle-bounds.json").read_bytes())
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    parent = list(range(len(vertices)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    indices = indices.reshape(-1, 3)
    for a, b, c in indices:
        parent[root(b)] = root(a)
        parent[root(c)] = root(a)
    labels = np.array([root(i) for i in indices[:, 0]])
    found = set()
    for label in np.unique(labels):
        shell = triangles[labels == label]
        box = np.r_[shell.min(axis=(0, 1)), shell.max(axis=(0, 1))]
        candidates = [name for name, native in bounds.items() if np.max(abs(native-box)) < .21]
        assert len(candidates) == 1 and candidates[0] not in found, candidates
        found.add(candidates[0])
        yield candidates[0], shell
    assert found == set(bounds)


def meshes():
    manifest = json.loads((ROOT / "docs/media/r9-assembly.json").read_bytes())
    asset = manifest["assets"]["glb"]
    raw = (ROOT / asset["path"]).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == asset["sha256"]
    footprint = json.loads((ROOT / "experiments/ground-relay/footprint.json").read_bytes())
    assert footprint["sourceParkedArchiveSha256"] == manifest["sourceSha256"]
    size = struct.unpack_from("<I", raw, 12)[0]
    header = json.loads(raw[20:20+size])
    blob = raw[28+size:]

    def array(index):
        spec = header["accessors"][index]
        view = header["bufferViews"][spec["bufferView"]]
        scalar = {5126: "<f4", 5123: "<u2", 5125: "<u4"}[spec["componentType"]]
        width = {"SCALAR": 1, "VEC3": 3}[spec["type"]]
        offset = view.get("byteOffset", 0) + spec.get("byteOffset", 0)
        assert "byteStride" not in view and view["buffer"] == 0
        return np.frombuffer(blob, scalar, spec["count"]*width, offset).reshape(-1, width)

    for node in header["nodes"]:
        assert set(node) == {"mesh", "name"}
        part, = header["meshes"][node["mesh"]]["primitives"]
        vertices = array(part["attributes"]["POSITION"]).astype(float)
        xyz = vertices[:, [0, 2, 1]] * [1000, -1000, 1000]
        tris = xyz[array(part["indices"]).reshape(-1, 3)]
        if node["name"] == TREAD:
            for side, suffix in ((-1, "534"), (1, "535")):
                yield suffix, tris[(side*tris[:, :, 1] > 0).all(axis=1)]
        elif node["name"] == "R7 cradle / R7 mechanism":
            yield from cradle_components(tris)
        else:
            yield node["name"], tris


def distance(points, start, end, y, z):
    # Distances from a stationary mesh point to the NOTE centre's circular axis.
    # Continuous axial travel produces every radius in [near, far] in the xy plane.
    x = points[..., 0]
    dy = points[..., 1]-y
    near = np.hypot(np.maximum(0, np.maximum(start-x, x-end)), dy)
    far = np.hypot(np.maximum(abs(start-x), abs(end-x)), dy)
    radial = np.maximum(0, np.maximum(near-MAJOR, MAJOR-far))
    return np.hypot(radial, points[..., 2]-z)-TUBE


def band(tris, start, end, y, z, error=.015):
    # A triangle's largest vertex distance from its centroid bounds the error
    # in a centroid distance sample (the signed distance is 1-Lipschitz).
    best = float("inf")
    while len(tris):
        mids = tris.mean(axis=1)
        values = distance(mids, start, end, y, z)
        best = min(best, float(values.min()), float(distance(tris, start, end, y, z).min()))
        radius = np.linalg.norm(tris-mids[:, None, :], axis=2).max(axis=1)
        lo, hi = tris.min(axis=1), tris.max(axis=1)
        # Independent lower bound from each triangle AABB. This remains tight
        # on the long, flat tread faces even for an axial interval.
        low_x = np.maximum(0, np.maximum(lo[:, 0]-end, start-hi[:, 0]))
        low_y = np.maximum(0, np.maximum(lo[:, 1]-y, y-hi[:, 1]))
        low_z = np.maximum(0, np.maximum(lo[:, 2]-z, z-hi[:, 2]))
        high_x = np.maximum(abs(lo[:, 0]-end), abs(hi[:, 0]-start))
        high_y = np.maximum(abs(lo[:, 1]-y), abs(hi[:, 1]-y))
        r0, r1 = np.hypot(low_x, low_y), np.hypot(high_x, high_y)
        low_radial = np.maximum(0, np.maximum(r0-MAJOR, MAJOR-r1))
        bound = np.maximum(values-radius, np.hypot(low_radial, low_z)-TUBE)
        tris = tris[bound < best-error]
        if not len(tris):
            break
        a, b, c = tris.transpose(1, 0, 2)
        ab, bc, ca = (a+b)/2, (b+c)/2, (c+a)/2
        tris = np.concatenate([np.stack(row, axis=1) for row in
                               [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]])
    return [round(best-error, 3), round(best, 3)]


def screen(groups, start, end, y, z):
    # Discard groups whose bounding box is separated by more than 4 mm from
    # the swept ring box. Keep the numeric lower bound for discarded groups.
    lower = np.array([start-MAJOR-TUBE, y-MAJOR-TUBE, z-TUBE])
    upper = np.array([end+MAJOR+TUBE, y+MAJOR+TUBE, z+TUBE])
    out = {}
    for name, triangles in groups:
        a = triangles.min(axis=1)
        b = triangles.max(axis=1)
        gaps = np.maximum(0, np.maximum(a-upper, lower-b)).max(axis=1)
        candidates = triangles[gaps<=4]
        if len(candidates):
            interval = band(candidates, start, end, y, z)
            if np.any(gaps>4):
                interval[0] = min(interval[0], round(float(gaps[gaps>4].min()),3))
            out[name] = interval
    return out


def receipt():
    groups = list(meshes())
    cases = {"externalDock": (FRONT, FRONT, 0, TUBE),
             "floorStroke": (SEAT, FRONT, 0, TUBE),
             "floorSeat": (SEAT, SEAT, 0, TUBE),
             "offset3mm": (SEAT, FRONT, 3, TUBE),
             "offset20mm": (SEAT, FRONT, 20, TUBE)}
    out = {key: screen(groups, *params) for key, params in cases.items()}
    # Carry at the registered seat. During the lift the carriage, cradle
    # and held NOTE translate together; check fixed shapes over the entire
    # vertical interval using endpoint samples and a Lipschitz allowance.
    native = json.loads((HERE / "cradle-bounds.json").read_bytes())
    moving = [(name, ts) for name, ts in groups
              if name in native or name in ("534", "535", "R7 carriage / R7 mechanism")]
    fixed = [(name, ts) for name, ts in groups if name not in dict(moving)]
    raised = fixed + [(name, ts+[0,0,8]) for name, ts in moving]
    out["raisedStroke"] = screen(raised, SEAT, FRONT, 0, TUBE+8)
    minimum = {}
    for dz in np.linspace(0, 8, 17):
        for name, pair in screen(fixed, SEAT, SEAT, 0, TUBE+dz).items():
            minimum[name] = min(minimum.get(name, 4), pair[0]-.25)
    out["seatedLift"] = {"riseMm": 8, "unlistedFixedLowerMm": 3.75,
                         "fixedLowerMm": minimum, "moving": screen(moving, SEAT, SEAT, 0, TUBE)}
    return out


def nominal_permitted(evidence):
    intended = ("534", "535")
    stroke = evidence["floorStroke"]
    seated = evidence["floorSeat"]
    lift = evidence["seatedLift"]
    return (all(seated[t][1]<0 for t in intended)
            and all(band[0]>0 for name,band in stroke.items() if name not in intended)
            and min([lift["unlistedFixedLowerMm"], *lift["fixedLowerMm"].values()])>0
            and all(band[0]>0 for name,band in lift["moving"].items() if name not in intended))


if __name__ == "__main__":
    (HERE / "contact.json").write_bytes((json.dumps(receipt(), indent=2)+"\n").encode())
