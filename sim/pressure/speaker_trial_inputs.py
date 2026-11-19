"""Physical setup errors for the 2024 six-NOTE campaign in blue field coordinates."""

import csv
import math

import numpy as np

from run_axial_shear_lift import EVIDENCE, digest
from wing_field_frame import WingFrame


COHORT = EVIDENCE / "speaker-repeatability-starts.csv"
FRAME = WingFrame("middle", 245.)
MARKS = ("middle", "outer", "inner", "center-high", "center-mid", "center-low")


def read_attempt(path, attempt):
    """Load physical perturbations in the fixed blue field frame."""
    dimensions = {key+"_"+axis+"_mm" for key in ("robot", *MARKS) for axis in ("x", "y")}
    expected = {"attempt", "robot_heading_deg"} | dimensions
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if (reader.fieldnames is None or len(reader.fieldnames) != len(expected)
                or set(reader.fieldnames) != expected):
            raise ValueError("Setup must specify robot pose and XY placement for all six NOTES")
        rows = list(reader)
    ids = [r["attempt"] for r in rows]
    if any(not key for key in ids) or len(ids) != len(set(ids)) or attempt not in ids:
        raise ValueError("Attempt IDs must be unique and include the requested attempt")
    row = rows[ids.index(attempt)]
    values = {key: float(value) for key, value in row.items() if key != "attempt"}
    if any(not math.isfinite(value) for value in values.values()):
        raise ValueError("All physical offsets must be finite")
    return values


def local_offset(row, key):
    field_offset = np.array([row[key+"_x_mm"], row[key+"_y_mm"], 0.]) / 1000
    return (FRAME.rotation.T @ field_offset)[:2]

def placed_marks(frame, receipt, roster):
    """Reconstruct physical field marks without moving the chassis registration."""
    nominal = {key: frame.mark(key) for key in MARKS}
    if roster is None and "attempt" not in receipt:
        return nominal
    if (not roster or not receipt.get("attempt") or receipt.get("rosterSha256") != digest(roster)
            or (frame.first_mark, frame.yaw_degrees) != ("middle", 245.)):
        raise ValueError("Attained run requires the matching physical setup roster and frame")
    row = read_attempt(roster, receipt["attempt"])
    for name, position in nominal.items():
        position[:2] += np.array([row[name+"_x_mm"], row[name+"_y_mm"]]) / 1000.
    return nominal


def verify_start(receipt, roster):
    if not roster or receipt["schema"] != "varied-first-wing-acquisition/1":
        raise ValueError("Expected a registered varied first pickup")
    marks = placed_marks(FRAME, receipt, roster)
    row = read_attempt(roster, receipt["attempt"])
    expected_pose = [*local_offset(row, "robot"), math.radians(row["robot_heading_deg"]), 0., 0.]
    initial = receipt["initialQ"]
    local_note = FRAME.to_local(marks["middle"])[:2]
    if (receipt["start"] != 0. or receipt["collectedMarks"] != ["middle"]
            or receipt["pieceId"] != "blue-wing-middle" or receipt["outcome"] != "SEATED"
            or len(initial) != 16 or len(receipt["initialV"]) != 15
            or len(receipt["initialRotors"]) != 4 or not np.isfinite(initial).all()
            or receipt["initialRobotQ"] != initial[:9]
            or receipt["initialRobotV"] != receipt["initialV"][:9]
            or not np.allclose(initial[:5], expected_pose, rtol=0., atol=1e-12)
            or not np.allclose(initial[9:11], local_note, rtol=0., atol=1e-12)
            or not np.allclose(receipt["waitingNoteLocalXY"], local_note, rtol=0., atol=1e-12)
            or not np.allclose(receipt["targetLocalXY"], [.658, 0.], rtol=0., atol=1e-12)
            or abs(initial[11]-(.0254-.235*9.80665/20000)) > 1e-12
            or any(initial[5:9]) or initial[12:] != [1., 0., 0., 0.]
            or any(receipt["initialV"]) or any(receipt["initialRotors"])
            or receipt["initialBusV"] != 12.):
        raise ValueError("Initial chassis or free NOTE differs from its declared physical setup")
