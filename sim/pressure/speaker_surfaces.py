"""Private STEP tessellations used by the 2024 blue Speaker contact study."""
import hashlib
import json

import igl
import numpy as np


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class SpeakerSurfaces:
    """Distance to each installed part separately, including simultaneous contacts."""

    def __init__(self, folder):
        self.parts = []
        self.packets = []
        self.pins = {}
        self.trees = {}
        self.bounds = []
        self.cache_center = None
        for name in ("panels.json", "extras.json"):
            manifest = json.loads((folder / name).read_bytes())
            mesh = folder / manifest["mesh"]
            if digest(mesh) != manifest["meshSha256"]:
                raise ValueError(f"Field geometry hash mismatch in {name}")
            self.pins[name] = digest(folder / name)
            self.pins[mesh.name] = manifest["meshSha256"]
            self.packets.append(np.load(mesh, allow_pickle=False))
            for part in manifest["parts"]:
                rotation = np.asarray(part.get("rotation", np.eye(3)))
                translation = np.asarray(part.get("translation", [0., 0., 0.]))
                if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0):
                    raise ValueError("Invalid occurrence rotation")
                self.parts.append((part, len(self.packets)-1, rotation, translation))
                self.bounds.append(part["blueFieldBoundsM"])
        keys = [part["key"] for part, *_ in self.parts]
        if len(set(keys)) != len(keys) or len(keys) != 1143:
            raise ValueError("Expected the complete set of blue Speaker leaf occurrences")
        self.bounds = np.asarray(self.bounds)

    def candidates(self, center, points, major, tube):
        slack = .05
        if self.cache_center is None or max(abs(center-self.cache_center)) > slack:
            reach = major + tube + slack
            near = (center + reach >= self.bounds[:, 0]) & (center - reach <= self.bounds[:, 1])
            self.near = np.flatnonzero(near.all(axis=1))
            self.cache_center = center.copy()
        boxes = self.bounds[self.near]
        low, high = points.min(axis=0)-tube, points.max(axis=0)+tube
        near = (high >= boxes[:, 0]) & (low <= boxes[:, 1])
        for index in self.near[near.all(axis=1)]:
            low, high = self.bounds[index]
            selected = np.flatnonzero(((points+tube >= low) & (points-tube <= high)).all(axis=1))
            if len(selected):
                yield index, selected

    def nearest(self, index, points):
        part, packet, rotation, translation = self.parts[index]
        mesh_key = part.get("mesh", part["key"])
        key = packet, mesh_key
        if key not in self.trees:
            archive = self.packets[packet]
            vertices = np.ascontiguousarray(archive[mesh_key+"_v"], dtype=np.float64)
            triangles = np.ascontiguousarray(archive[mesh_key+"_t"], dtype=np.int64)
            tree = igl.AABB()
            tree.init(vertices, triangles)
            self.trees[key] = tree, vertices, triangles
        tree, vertices, triangles = self.trees[key]
        local = (points-translation) @ rotation
        squared, _, closest = tree.squared_distance(vertices, triangles, local)
        return part, np.sqrt(squared), closest @ rotation.T + translation

    def close(self):
        for packet in self.packets:
            packet.close()
