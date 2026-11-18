"""Prepare saved dynamic ring poses for Fusion's temporary BRep checker (meters to mm)."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import digest


def frames(q, note_address):
    c, s = math.cos(q[2]), math.sin(q[2])
    yaw = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    c, s = math.cos(q[4]), math.sin(q[4])
    pitch = np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])
    xyz = q[note_address:note_address+3]
    w, x, y, z = q[note_address+3:note_address+7]
    normal = yaw.T @ [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
    if not np.isclose(np.linalg.norm(normal), 1., atol=1e-9, rtol=0):
        raise ValueError("Nonunit recorded ring axis")
    fixed = 1000 * (yaw.T @ (np.array(xyz) - [q[0], q[1], 0.]))
    carriage = fixed - [0., 0., q[3]*1000]
    pivot = np.array([178., 0., 25.4])
    cradle = pivot + pitch.T @ (carriage-pivot)
    return {group: dict(center=center.tolist(), normal=axis.tolist(), padMm=0.)
            for group, center, axis in (("fixed", fixed, normal), ("carriage", carriage, normal),
                                       ("cradle", cradle, pitch.T @ normal))}


def prepare(run, output):
    receipt = json.loads((run/"result.json").read_bytes())
    trace = run/"trace.jsonl"
    if receipt["schema"] != "native-raised-feed/1" or digest(trace) != receipt["traceSha256"]:
        raise ValueError("Handoff trace mismatch")
    poses = []
    with trace.open() as stream:
        for line in stream:
            row = json.loads(line)
            groups = frames(row["q"], len(row["q"])-7)
            y = groups["fixed"]["center"][1]
            poses.append(dict(clock=row["time"], frames=groups, yBoundsMm=[y-177.81,y+177.81]))
    if not poses:
        raise ValueError("No recorded poses")
    with output.open("x") as stream:
        json.dump(poses, stream, allow_nan=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    prepare(args.run, args.output)
