"""Turn a sampled joint timeline into motion-study cases.

A state gives every joint value at one instant. Each rigid class has a pose for a state. Consecutive
samples are merged into one case while the straight-line path of every class stays within a tolerance
of the samples it skips, so a case never departs from its trace by more than the stated deviation.
"""
import math

import numpy as np

import kin

PIVOT_PARK = (kin.PARK_PIVOT_X_MM, 0.0, kin.PIVOT_Z_MM)
DRUM_RADIUS_M = 0.0295          # pivot cable drum, from the hoist publisher
# Largest distance in mm from the turning centre to a point of the class: converts a rotation error to mm.
REACH_MM = {"cassette": 720.0, "cover-leaf": 80.0, "note": 180.0, "drum": 60.0, "clutch": 60.0}

DEFAULTS = dict(angle=kin.PARK_ANGLE_DEG, px=kin.PARK_PIVOT_X_MM, flap=0.0, note=None, draw=0.0, gate=0.0,
                coil=0.0, dog=0.0, clutch=0.0, stop=0.0)
# Classes that ride on the cassette. The NOTE joins them while it is carried.
ON_CASSETTE = ("cover-leaf", "carriage", "keeper", "drum", "clutch", "rope", "stop-left", "stop-right")


def complete(state):
    return {**DEFAULTS, **state}


def class_pose(cls, st):
    """World transform of a rigid class, modelled parked, for a state."""
    if cls == "frame":
        return np.eye(4)
    if cls == "truck":
        return kin.slide(st["px"])
    if cls == "catch-dog":
        return kin.slide(st["px"]) @ kin.pose(0, 0, st["dog"], 0)
    if cls == "note":
        return st["note"]
    base = kin.pose(st["angle"], st["px"], 0, kin.PIVOT_Z_MM)
    if cls in ("cassette", "rope"):
        local = np.eye(4)
    elif cls == "cover-leaf":
        local = kin.AXLE @ kin.pose(st["flap"]) @ kin.INV_AXLE
    elif cls == "carriage":
        local = kin.pose(0, -st["draw"])
    elif cls == "keeper":
        local = kin.pose(0, -st["draw"], 0, st["gate"])
    elif cls == "drum":
        local = kin.pose(math.degrees(st["coil"] / DRUM_RADIUS_M))
    elif cls == "clutch":
        local = kin.pose(0, 0, st["clutch"], 0) @ kin.pose(math.degrees(st["coil"] / DRUM_RADIUS_M))
    elif cls == "stop-left":
        local = kin.pose(0, 0, -st["stop"], 0)
    elif cls == "stop-right":
        local = kin.pose(0, 0, st["stop"], 0)
    else:
        raise KeyError(cls)
    return base @ local @ kin.INV_PARK


def poses(st, classes):
    return {c: class_pose(c, st) for c in classes if not (c == "note" and st["note"] is None)}


def _origin(cls, start, pose_start):
    """World point a class turns about within a case, at the case start."""
    if cls == "cover-leaf":
        return (pose_start @ kin.PARK @ kin.AXLE)[:3, 3]
    if cls == "note":
        return (pose_start @ kin.SEAT)[:3, 3]
    return np.array([start["px"], 0.0, kin.PIVOT_Z_MM])


def segment_ops(a, b, classes, carried):
    """Motion operations of each class from state a to state b.

    A class that rides on the cassette turns or slides in the cassette's start pose first and is then
    carried by the cassette's own operations, so at every path fraction it sits where the mechanism puts it.
    """
    pa, pb = poses(a, classes), poses(b, classes)
    cassette = np.eye(4) if "cassette" not in pa else pb["cassette"] @ np.linalg.inv(pa["cassette"])
    cassette_ops = kin.operations(cassette, [a["px"], 0.0, kin.PIVOT_Z_MM])
    truck_ops = kin.operations(pb["truck"] @ np.linalg.inv(pa["truck"]), (0, 0, 0)) if "truck" in pa else []
    out = {}
    for cls in pa:
        d = pb[cls] @ np.linalg.inv(pa[cls])
        if cls == "frame":
            ops = []
        elif cls == "truck":
            ops = truck_ops
        elif cls == "catch-dog":
            ops = kin.operations(np.linalg.inv(pb["truck"] @ np.linalg.inv(pa["truck"])) @ d, (0, 0, 0)) + truck_ops
        elif cls in ("cassette", "rope"):
            ops = cassette_ops
        elif cls in ON_CASSETTE or (cls == "note" and carried):
            ops = kin.operations(np.linalg.inv(cassette) @ d, _origin(cls, a, pa[cls])) + cassette_ops
        else:
            ops = kin.operations(d, _origin(cls, a, pa[cls]))
        if np.abs(d - np.eye(4)).max() < 1e-12:
            ops = []
        assert np.abs(kin.matrix(ops) - d).max() < 1e-8, (cls, np.abs(kin.matrix(ops) - d).max())
        out[cls] = ops
    return out, pa, pb


def initial_ops(st, classes):
    return {c: kin.operations(m, PIVOT_PARK) for c, m in poses(st, classes).items()}


def _deviation(ops, start_pose, true_pose, fraction, cls):
    e = kin.matrix(ops, fraction) @ start_pose - true_pose
    return float(np.linalg.norm(e[:3, 3]) + REACH_MM.get(cls, 720.0) * np.abs(e[:3, :3]).max())


def path_parameter(states, classes):
    """A monotone path coordinate: accumulated largest point travel of any class between samples."""
    s = [0.0]
    for a, b in zip(states, states[1:]):
        pa, pb = poses(a, classes), poses(b, classes)
        step = 0.0
        for c in pa:
            d = pb[c] @ np.linalg.inv(pa[c])
            _, angle = kin.axis_angle(d[:3, :3])
            step = max(step, float(np.linalg.norm(d[:3, 3] - (np.eye(3) - d[:3, :3]) @ _origin(c, a, pa[c])))
                       + REACH_MM.get(c, 720.0) * math.radians(angle))
        s.append(s[-1] + step + 1e-9)
    return s


def cut(states, classes, carried, tolerance_mm, max_span=None):
    """Greedy segmentation. Returns [(i, j, deviationMm)] over sample indices."""
    s = path_parameter(states, classes)
    out, i = [], 0
    while i < len(states) - 1:
        j, worst = i + 1, 0.0
        while j + 1 < len(states) and (max_span is None or j + 1 - i <= max_span):
            ops, pa, _ = segment_ops(states[i], states[j + 1], classes, carried)
            span = s[j + 1] - s[i]
            trial = 0.0
            for k in range(i + 1, j + 1):
                f = (s[k] - s[i]) / span
                true = poses(states[k], classes)
                trial = max(trial, max(_deviation(ops[c], pa[c], true[c], f, c) for c in pa))
                if trial > tolerance_mm:
                    break
            if trial > tolerance_mm:
                break
            j, worst = j + 1, trial
        out.append((i, j, worst))
        i = j
    return out, s
