"""Observe full NOTE passage and the installed counter case pairs from a field trace."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from wing_speaker_passage import LOW, HIGH, WIDTH, MAJOR, TUBE, rotation
from run_axial_shear_lift import digest


def opening_passage(rows, nodes=256):
    along = (HIGH-LOW)/np.linalg.norm(HIGH-LOW)
    normal = np.cross(along, [0., 1., 0.])
    angles = 2*math.pi*np.arange(nodes)/nodes
    ring = MAJOR*np.stack((np.cos(angles), np.sin(angles), np.zeros(nodes)), axis=1)
    minimum = math.inf
    first = None
    for row, following in zip(rows, rows[1:]):
        q = np.asarray(row["q"])
        end = np.asarray(following["q"])
        turn = 2*math.acos(np.clip(abs(q[3:] @ end[3:]), 0., 1.))
        # Before passage, every integration node is in the trace. Within one
        # semi-implicit step translation is linear and orientation follows its arc.
        radius = TUBE + MAJOR*math.pi/nodes + np.linalg.norm(q[:3]-end[:3]) + MAJOR*turn + 1e-9
        points = ring @ rotation(q[3:]).T + q[:3]-LOW
        height = points @ normal
        if row is rows[0] and height.max()+radius >= 0:
            return None
        if height.min()-radius > 0:
            if first is not None and minimum > 0:
                return dict(firstPossibleS=first, fullyThroughByS=row["t"], minimumMarginM=minimum)
            return None
        near = abs(height) <= radius
        if np.any(near):
            if first is None:
                first = row["t"]
            disk = np.sqrt(np.maximum(0., radius**2-height[near]**2))
            u = points[near] @ along
            y = points[near, 1]
            margin = np.minimum.reduce((u-disk, np.linalg.norm(HIGH-LOW)-u-disk, WIDTH/2-abs(y)-disk))
            minimum = min(minimum, float(margin.min()))
    return None


def beam_samples(a, b, nx=8, ny=16, nz=25):
    """Every ray between either end face passes through each horizontal slice."""
    ends = np.array([a, b])
    if (ends.shape != (2, 4, 3) or not np.isfinite(ends).all()
            or np.ptp(ends[:, :, 2], axis=1).max() > 1e-8
            or abs(ends[0, 0, 2]-ends[1, 0, 2]) < .002):
        raise ValueError("Expected separated horizontal sensor case ends")
    low = ends.min(axis=(0, 1))
    high = ends.max(axis=(0, 1))
    dx, dy = (high[:2]-low[:2])/[nx, ny]
    x = low[0] + (np.arange(nx)+.5)*dx
    y = low[1] + (np.arange(ny)+.5)*dy
    z = np.linspace(low[2]+.001, high[2]-.001, nz)
    zz, yy, xx = np.meshgrid(z, y, x, indexing="ij")
    points = np.stack((xx, yy, zz), axis=-1).reshape(nz, -1, 3)
    return points, math.hypot(dx, dy)/2


def shadow_margin(q, samples):
    points, pad = samples
    if np.any(abs(points[0, 0, :2]-q[:2]) > MAJOR+TUBE+.05):
        return -math.inf
    offset = points - q[:3]
    axis = rotation(q[3:])[:, 2]
    height = offset @ axis
    radial = np.sqrt(np.maximum(0., (offset**2).sum(axis=2)-height**2))
    tube_distance = np.hypot(radial-MAJOR, height)
    # Distance to the ring centerline is 1-Lipschitz, so each tile's corners
    # and interior have this certified maximum, including off-center rays.
    return float(TUBE-pad-tube_distance.max(axis=1).min())


def observe(folder, target):
    run = json.loads((folder / "result.json").read_bytes())
    trace = folder / "trace.jsonl"
    if digest(trace) != run["trace"]:
        raise ValueError("Field trace hash mismatch")
    sensor = json.loads(target.read_bytes())
    panels = target.with_name("panels.json")
    if (digest(panels) != run["field"]["panels.json"]
            or sensor["source"] != json.loads(panels.read_bytes())["source"]):
        raise ValueError("Case ends and field surfaces must share the registered source")
    corners = np.array([LOW, LOW, HIGH, HIGH])
    corners[:, 1] += WIDTH/2*np.array([-1., 1., 1., -1.])
    if not np.allclose(sensor["nominalOpeningCornersM"], corners, rtol=0., atol=1e-8):
        raise ValueError("Opening corners differ from the registered field")
    cases = {case["key"]: case["caseEndCornersM"] for case in sensor["sensorCaseEnds"]}
    pairs = [beam_samples(cases[a], cases[b]) for a, b in sensor["beamPairs"]]
    endpoints = [key for pair in sensor["beamPairs"] for key in pair]
    if len(cases) != 4 or len(pairs) != 2 or len(set(endpoints)) != 4:
        raise ValueError("Expected two installed opposing case pairs")
    with trace.open() as stream:
        rows = [json.loads(line) for line in stream]
    if len(rows) < 2:
        raise ValueError("Incomplete field history")
    state = np.array([r["q"] for r in rows])
    if (state.shape != (len(rows), 7) or not np.isfinite(state).all()
            or np.max(np.abs(np.linalg.norm(state[:, 3:], axis=1)-1.)) > 1e-8):
        raise ValueError("Invalid field pose")
    times = np.array([r["t"] for r in rows])
    if (not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
            or abs(times[0]-run["releaseTimeS"])>1e-8 or abs(times[-1]-run["endS"])>1e-8):
        raise ValueError("Incomplete or out-of-order field history")
    opening = opening_passage(rows)
    if opening:
        intervals = np.diff(times[times <= opening["fullyThroughByS"]+1e-10])
        if np.any(abs(intervals-run["parameters"]["dt"])>1e-8):
            raise ValueError("Opening verification needs every integration state")
    first = [None, None]
    best = [None, None]
    both = None
    peak = None
    if opening:
        for row in rows:
            if row["t"] <= opening["fullyThroughByS"]:
                continue
            q = np.array(row["q"])
            margins = [shadow_margin(q, sample) for sample in pairs]
            for i, margin in enumerate(margins):
                if math.isfinite(margin) and (best[i] is None or margin > best[i]):
                    best[i] = margin
                if margin > 0 and first[i] is None:
                    first[i] = row["t"]
            margin = min(margins)
            if math.isfinite(margin) and (peak is None or margin > peak):
                peak = margin
            if margin > 0 and both is None:
                both = row["t"]
    return dict(opening=opening, firstFullShadowsS=first, firstSimultaneousFullShadowS=both,
                bestMarginPerPairM=best, bestSimultaneousMarginM=peak,
                countedFloorNotes=int(both is not None), fieldTrace=run["trace"], sensorFile=digest(target),
                sourceTrace=run["sourceTrace"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("run", "target", "output"):
        parser.add_argument(arg, type=Path)
    args = parser.parse_args()
    result = observe(args.run, args.target)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result))
