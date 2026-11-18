"""Check light paths between opposing case faces with the empty field counter."""
import argparse
import json
from pathlib import Path

import numpy as np

from speaker_surfaces import SpeakerSurfaces


def check(folder):
    surfaces = SpeakerSurfaces(folder)
    target = json.loads((folder / "opening-and-counter.json").read_bytes())
    cases = {p["key"]: np.array(p["caseEndCornersM"]) for p in target["sensorCaseEnds"]}
    result = {}
    fractions = np.linspace(.05, .95, 9)
    offsets = np.array([[u, v, 1.] for u in fractions for v in fractions])
    try:
        for upper, lower in target["beamPairs"]:
            ends = np.stack([cases[upper], cases[lower]])
            low = ends.min(axis=(0, 1))
            high = ends.max(axis=(0, 1))
            origin = low+(high-low)*offsets
            origin[:, 2] -= .00001
            length = high[2]-low[2]-.00002
            direction = np.tile([0., 0., -1.], (len(origin), 1))
            blocked = np.zeros(len(origin), dtype=bool)
            near = np.flatnonzero(((high >= surfaces.bounds[:, 0]) & (low <= surfaces.bounds[:, 1])).all(axis=1))
            for i in near:
                part, packet, rotation, translation = surfaces.parts[i]
                surfaces.nearest(i, origin[:1])
                tree, vertices, faces = surfaces.trees[packet, part.get("mesh", part["key"])]
                rays = tree.intersect_ray(vertices, faces, (origin-translation) @ rotation, direction @ rotation)
                blocked |= np.array([any(0 < distance < length for _, distance, _, _ in hits) for hits in rays])
            result[f"{upper}-{lower}"] = dict(unblocked=int((~blocked).sum()), tried=len(blocked),
                                               centerUnblocked=not bool(blocked[len(blocked)//2]))
    finally:
        surfaces.close()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field", type=Path)
    args = parser.parse_args()
    print(json.dumps(check(args.field), indent=2))
