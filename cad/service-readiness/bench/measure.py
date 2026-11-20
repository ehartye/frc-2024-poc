"""Measure stock, tool and access gauges against the original private supplier solids."""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "native"))
from controller_mount_review import bounds, common, read_step, transform
from layout import WIDTH, DEPTH, THICKNESS, CENTERS, BRACKET_HOLES, CLINCH_HOLES, BENCH_HOLES


def box(x, y, z, X, Y, Z):
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), gp_Pnt(X, Y, Z)).Shape()


def cylinder(x, y, z, height, radius):
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1)), radius, height
    ).Shape()


def volume(s):
    prop = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, prop)
    return prop.Mass()


def separation(a, b):
    check = BRepExtrema_DistShapeShape(a, b)
    if not check.IsDone():
        raise ValueError("Distance evaluation failed")
    return round(check.Value(), 4)


def place(s, x, y, z=0):
    return transform(s, (1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z))


def write_step(s, p):
    writer = STEPControl_Writer()
    if (
        writer.Transfer(s, STEPControl_AsIs) != IFSelect_RetDone
        or writer.Write(str(p)) != IFSelect_RetDone
    ):
        raise ValueError("Exchange export failed")


def review(bracket_path, controller_path, folder):
    ref = json.loads((Path(__file__).parents[1] / "controller-mount-review.json").read_bytes())
    for p in (bracket_path, controller_path):
        if hashlib.sha256(p.read_bytes()).hexdigest() != ref["stepSha256"][p.name]:
            raise ValueError(f"Unrecognized supplier revision {p.name}")
    bracket = transform(read_step(bracket_path), (1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0.5))
    controller = transform(read_step(controller_path), (-1, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 22.95))
    stock = box(-WIDTH / 2, -DEPTH / 2, -THICKNESS, WIDTH / 2, DEPTH / 2, 0)
    for points, radius in ((CLINCH_HOLES, 2.705), (BENCH_HOLES, 3.3)):
        for x, y in points:
            stock = BRepAlgoAPI_Cut(stock, cylinder(x, y, -3, 4, radius)).Shape()
    analytic = THICKNESS * (
        WIDTH * DEPTH - math.pi * (len(CLINCH_HOLES) * 2.705**2 + len(BENCH_HOLES) * 3.3**2)
    )
    if abs(volume(stock) - analytic) > 0.01:
        raise ValueError("Perforated plate volume")
    articles = {}
    for i, (x, y) in enumerate(CENTERS, 1):
        articles[f"bracket {i}"] = place(bracket, x, y)
        articles[f"controller {i}"] = place(controller, x, y)
    article_bounds = {k: bounds(v) for k, v in articles.items()}

    def crosses(a, b):
        return all(min(a[i + 3], b[i + 3]) - max(a[i], b[i]) > 1.0e-5 for i in range(3))

    pairs = []
    for i, (key, a) in enumerate(articles.items()):
        for other, b in list(articles.items())[i + 1 :]:
            if crosses(article_bounds[key], article_bounds[other]) and (v := common(a, b)) > 0.01:
                pairs.append([key, other, v])
    gauges = {}

    def check(label, gauge):
        limits = bounds(gauge)
        gauges[label] = {
            key: common(gauge, s)
            for key, s in articles.items()
            if crosses(limits, article_bounds[key])
        }
        gauges[label] = {k: v for k, v in gauges[label].items() if v > 0.01}

    for i, (x, y) in enumerate(CENTERS, 1):
        for j, (u, v) in enumerate(BRACKET_HOLES, 1):
            check(f"head {i}.{j}", cylinder(x + u, y + v, 2.5, 4.0, 3.5))
            check(f"key {i}.{j}", cylinder(x + u, y + v, 6.5, 65.0, 3.0))
            check(f"shank {i}.{j}", cylinder(x + u, y + v, -5.5, 8.0, 2.0))
        for sign in (-1, 1):
            lo, hi = sorted((x + sign * 40.625, x + sign * 75.625))
            check(f"end withdrawal {i}.{sign}", box(lo, y - 15.0, 2.0, hi, y + 15.0, 18.0))
            # The accessory sockets sit in the upper end face above the
            # barrel exits. Reserve the approach to the case, including those
            # 6 mm omitted by a gauge that starts only at the barrel tips.
            a, b = sorted((x + sign * 34.625, x + sign * 75.625))
            check(f"upper end face {i}.{sign}", box(a, y - 17.25, 12.0, b, y + 17.25, 30.0))
        check(f"overhead {i}", box(x - 41, y - 30.3, 30.1, x + 41, y + 30.3, 55.1))
    nearest_bracket = separation(articles["bracket 1"], articles["bracket 2"])
    end_free = separation(articles["controller 1"], articles["controller 3"])
    # The complete supplier flange penetrates a solid plate when lowered.
    shift_control = common(place(bracket, CENTERS[0][0], CENTERS[0][1], -0.1), stock)
    seated = shift_control / 0.1
    failed_head = common(
        cylinder(
            CENTERS[0][0] + BRACKET_HOLES[0][0] + 3, CENTERS[0][1] + BRACKET_HOLES[0][1], 0, 2.5, 2
        ),
        articles["bracket 1"],
    )
    mirror_control = common(
        bracket, transform(read_step(controller_path), (1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 22.95))
    )
    if pairs or any(gauges.values()) or min(seated, failed_head, mirror_control) < 1:
        raise ValueError(
            f"Bench fit error {pairs}, {[(k, v) for k, v in gauges.items() if v]}, controls {seated, failed_head, mirror_control}"
        )
    head_seats = [
        round(common(cylinder(x, y, 2.48, 0.02, 3.5), bracket) / 0.02, 3) for x, y in BRACKET_HOLES
    ]
    if min(head_seats) < 18:
        raise ValueError("Insufficient projected support beneath screw head")
    result = dict(
        frame="isolated bench, millimetres, plate top Z0",
        supplierSha256=ref["stepSha256"],
        centersMm=CENTERS,
        plateMm=[WIDTH, DEPTH, THICKNESS],
        clinchHolesMm=CLINCH_HOLES,
        benchHolesMm=BENCH_HOLES,
        plateVolumeMm3=round(volume(stock), 3),
        plateCalculatedG=round(volume(stock) * 0.00270, 3),
        mountingHoles=12,
        distinctSupplierSolidsPerController=18,
        fixturePositiveIntersections=pairs,
        gaugeCollisions=gauges,
        sameRowEndSeparationMm=end_free,
        sameColumnBracketSeparationMm=nearest_bracket,
        openGapBetweenOpposed35mmGaugesMm=round(end_free - 70, 4),
        flangeProjectedSeatPerBracketMm2=round(seated, 3),
        headProjectedSeatPerFootMm2=head_seats,
        shiftedShankOverlapMm3=failed_head,
        reversedControllerOverlapMm3=mirror_control,
        minimumClinchCenterToOutsideEdgeMm=min(
            min(WIDTH / 2 - abs(x), DEPTH / 2 - abs(y)) for x, y in CLINCH_HOLES
        ),
        screwTipBelowPlateMm=3.5,
        screwProjectionBeyondThickestNutMm=1.25,
        purchasedG={"controllers": 453.2, "brackets": 72.0, "twelveM4x8": 19.32},
        twelveClinchNutConservativeUpperG=round(
            12 * 0.0081 * math.pi * ((8.12 / 2) ** 2 * 2.25 + (5.38 / 2) ** 2 * 1.38), 3
        ),
    )
    step = folder / "bench-plate.step"
    write_step(stock, step)
    raw = step.read_bytes().replace(b"\r\n", b"\n")
    raw = re.sub(
        rb"(FILE_NAME\('Open CASCADE Shape Model',')[-0-9:T]+",
        rb"\g<1>2026-11-20T00:00:00",
        raw,
        count=1,
    )
    raw = b"\n".join(line.rstrip() for line in raw.splitlines()) + b"\n"
    step.write_bytes(raw)
    if abs(volume(read_step(step)) - analytic) > 0.01:
        raise ValueError("Reopened stock mismatch")
    result["plateStepSha256"] = hashlib.sha256(step.read_bytes()).hexdigest()
    (folder / "bench-fit.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ("bracket", "controller", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    review(args.bracket, args.controller, args.output)
