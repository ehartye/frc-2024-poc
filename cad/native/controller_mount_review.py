"""Check a proposed passive controller layout against privately supplied REV STEP files."""

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "assembly-readiness"))
from breaker_contacts import (
    BRepAlgoAPI_Common,
    BRepBuilderAPI_Transform,
    BRepGProp,
    GProp_GProps,
    STEPControl_Reader,
    TopAbs_SOLID,
    bounds,
    explore,
    gp_Trsf,
)

STATIONS = [
    (-26.5, -257, "Y"),
    (-26.5, 103, "Y"),
    (-57, -160.5, "X"),
    (-74.5, -251, "Y"),
    (-26.5, -59, "Y"),
]
HOLES = [(-12.7, -25.4), (12.7, -25.4), (0, 25.4)]


def read_step(path):
    reader = STEPControl_Reader()
    if int(reader.ReadFile(str(path))) != 1 or not reader.TransferRoots():
        raise ValueError(f"Invalid STEP input {path.name}")
    return reader.OneShape()


def transform(solid, values):
    tr = gp_Trsf()
    tr.SetValues(*values)
    return BRepBuilderAPI_Transform(solid, tr, True).Shape()


def common(first, second):
    result = BRepAlgoAPI_Common(first, second).Shape()
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(result, properties)
    return round(properties.Mass(), 5)


def check(bracket_path, controller_path):
    bracket, controller = map(read_step, (bracket_path, controller_path))
    cases = list(explore(controller, TopAbs_SOLID))
    if len(list(explore(bracket, TopAbs_SOLID))) != 1 or len(cases) != 18:
        raise ValueError("Unexpected product revision or solid count")

    def fit(flip, shift):
        return [
            common(bracket, transform(s, (flip, 0, 0, shift, 0, -1, 0, 22.45, 0, 0, -flip, 0)))
            for s in cases
        ]

    correct = fit(-1, 0)
    controls = [sum(fit(flip, shift)) for flip, shift in ((1, 0), (-1, 1.25), (-1, -1.25))]
    if max(correct) > 0.01 or min(controls) < 1:
        raise ValueError("Bracket registration or orientation control failed")
    placed = []
    stations = []
    for x, y, axis in STATIONS:
        # Local X and Z are in the mounting plane. Positive local Y faces away from deck.
        tr = (
            (1, 0, 0, x, 0, 0, -1, y, 0, 1, 0, 205.615)
            if axis == "X"
            else (0, 0, 1, x, 1, 0, 0, y, 0, 1, 0, 205.615)
        )
        placed.append(transform(bracket, tr))
        centers = [(x + u, y - v) if axis == "X" else (x + v, y + u) for u, v in HOLES]
        ligaments = [round(min(cx + 320, -cx, cy + 310, 310 - cy) - 2.5, 3) for cx, cy in centers]
        stations.append(
            dict(
                centerMm=[x, y],
                longAxis=axis,
                holeCentersMm=centers,
                materialToDeckOutsideEdgeMm=ligaments,
            )
        )
    pairs = []
    for i, j in itertools.combinations(range(len(placed)), 2):
        a, b = bounds(placed[i]), bounds(placed[j])
        if all(min(a[k + 3], b[k + 3]) > max(a[k], b[k]) for k in range(3)):
            pairs.append(
                dict(stations=[i + 1, j + 1], bracketOverlapMm3=common(placed[i], placed[j]))
            )
    return dict(
        stepSha256={
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (bracket_path, controller_path)
        },
        bracketBoundsMm=bounds(bracket),
        controllerBoundsMm=bounds(controller),
        controllerSolidCount=len(cases),
        nominalBracketOverlapMm3=sum(correct),
        controlOverlapMm3=controls,
        stations=stations,
        overlappingBracketBounds=pairs,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("bracket", "controller", "output"):
        parser.add_argument(arg, type=Path)
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(check(args.bracket, args.controller), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
