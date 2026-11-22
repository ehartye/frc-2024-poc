"""Rigid-body poses of the guided cassette in the parked STEP frame, and their motion-study operations.

Frame: millimetres, +X forward, +Y left, +Z up from the carpet. The layout STEP is modelled parked: the
cassette at 125 degrees with the pivot axis through (185, 0, 501), parallel to Y. The definitions repeat
those of the replay publisher (six-piece-02/publish.py in the evidence tree), so a pose built here equals
the published replay transform for the same joint values.
"""
import math

import numpy as np

PARK_ANGLE_DEG = 125.0
PARK_PIVOT_X_MM = 185.0
PIVOT_Z_MM = 501.0
SEAT_LOCAL_MM = (215.0, 0.0, 50.4)        # NOTE centre at its seat, cassette frame
FLAP_AXLE_LOCAL_MM = (457.5, 0.0, 96.0)   # passive cover hinge, cassette frame


def pose(angle_deg, x=0.0, y=0.0, z=0.0):
    """Cassette-style frame: rotation by -angle about Y, then translation."""
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    m[:3, :3] = [[c, 0, -s], [0, 1, 0], [s, 0, c]]
    m[:3, 3] = [x, y, z]
    return m


PARK = pose(PARK_ANGLE_DEG, PARK_PIVOT_X_MM, 0, PIVOT_Z_MM)
INV_PARK = np.linalg.inv(PARK)
SEAT = PARK @ pose(0, *SEAT_LOCAL_MM)
INV_SEAT = np.linalg.inv(SEAT)
AXLE = pose(0, *FLAP_AXLE_LOCAL_MM)
INV_AXLE = np.linalg.inv(AXLE)


def arm(angle_deg, pivot_x):
    """World transform of a cassette-borne body modelled at park."""
    return pose(angle_deg, pivot_x, 0, PIVOT_Z_MM) @ INV_PARK


def slide(pivot_x):
    """World transform of a body on the translating pivot truck."""
    return pose(0, pivot_x - PARK_PIVOT_X_MM, 0, 0)


def flap(angle_deg, pivot_x, flap_deg):
    """World transform of the passive cover leaf, hinged on the cassette."""
    return pose(angle_deg, pivot_x, 0, PIVOT_Z_MM) @ AXLE @ pose(flap_deg) @ INV_AXLE @ INV_PARK


def quat_matrix(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def piece(center_mm, quat_wxyz):
    """World transform of the NOTE body, modelled at its parked seat, for a centre and orientation."""
    m = np.eye(4)
    m[:3, :3] = quat_matrix(*quat_wxyz)
    m[:3, 3] = center_mm
    return m @ INV_SEAT


def axis_angle(r):
    """Unit axis and angle in degrees of a proper rotation matrix, angle in [0, 180]."""
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    s = np.linalg.norm(w) / 2
    c = (np.trace(r) - 1) / 2
    angle = math.atan2(s, c)
    if s > 1e-9:
        return w / (2 * s), math.degrees(angle)
    if c > 0:
        return np.array([0.0, 1.0, 0.0]), 0.0
    # Half turn: take the axis from the symmetric part.
    d = np.diag(r) + 1
    k = int(np.argmax(d))
    axis = (r[:, k] + np.eye(3)[:, k])
    return axis / np.linalg.norm(axis), 180.0


def rotation(origin, axis, degrees):
    return dict(kind="rotation", axis=[float(v) for v in axis], originMm=[float(v) for v in origin],
                travelDeg=float(degrees))


def translation(vector):
    return dict(kind="translation", travelMm=[float(v) for v in vector])


def operations(d, origin, *, angle_floor=1e-9, shift_floor=1e-9):
    """Decompose rigid transform d as a rotation about an axis through `origin`, then a translation.

    The checker scales each operation by the path fraction, so the body turns about `origin` while that
    point travels in a straight line to its end position.
    """
    r, t = d[:3, :3], d[:3, 3]
    origin = np.asarray(origin, float)
    axis, angle = axis_angle(r)
    ops = []
    if abs(angle) > angle_floor:
        ops.append(rotation(origin, axis, angle))
        shift = t - origin + r @ origin
    else:
        shift = t
    if np.linalg.norm(shift) > shift_floor:
        ops.append(translation(shift))
    return ops


def matrix(ops, fraction=1.0):
    """Compose operations in order, each scaled by the path fraction, as the checker does."""
    m = np.eye(4)
    for op in ops:
        step = np.eye(4)
        if op["kind"] == "translation":
            step[:3, 3] = np.array(op["travelMm"]) * fraction
        else:
            a = np.array(op["axis"], float)
            a = a / np.linalg.norm(a)
            th = math.radians(op["travelDeg"]) * fraction
            k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            r = np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * (k @ k)
            o = np.array(op["originMm"], float)
            step[:3, :3] = r
            step[:3, 3] = o - r @ o
        m = step @ m
    return m
