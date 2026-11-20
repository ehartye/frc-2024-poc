"""Reopen the exported native fixture and check interfaces with geometric controls."""

from adsk import core, fusion
import battery_service as battery
from physical_bumper import dimensions, registered_solid
from stage_climber import box, rod


def overlap(a, b):
    return all(min(a[i + 3], b[i + 3]) - max(a[i], b[i]) > 0.0001 for i in range(3))


def common(first, second):
    mgr = fusion.TemporaryBRepManager.get()
    target = mgr.copy(first)
    if not mgr.booleanOperation(target, second, fusion.BooleanTypes.IntersectionBooleanType):
        raise RuntimeError("Intersection calculation failed")
    return round(target.volume * 1000, 5) if target.isValid and target.lumps.count else 0.0


def moved(solid, x=0.0, y=0.0, z=0.0):
    mgr = fusion.TemporaryBRepManager.get()
    target = mgr.copy(solid)
    transform = core.Matrix3D.create()
    transform.translation = core.Vector3D.create(x / 10, y / 10, z / 10)
    if not mgr.transform(target, transform):
        raise ValueError("Service gauge translation failed")
    return target


def interference(gauge, articles, skip=()):
    result = {}
    bounds = dimensions(gauge)
    for name, solid in articles.items():
        if name not in skip and overlap(bounds, dimensions(solid)):
            volume = common(gauge, solid)
            if volume > 0.01:
                result[name] = volume
    return result


def fixture_articles(d):
    return {
        b.name: registered_solid(o, b) for o in d.rootComponent.allOccurrences for b in o.bRepBodies
    }


def inspect(d, r9):
    parts = fixture_articles(d)
    pick = lambda prefix: next(n for n in parts if n.startswith(prefix))
    tray = parts["R7 166 battery-tray"]
    pad = pick("Insulating top pad")
    strap = "R7 203 battery-top-strap"
    case = "R7 144 battery"
    base = battery.BASE_Z
    sweep = box(-291.5, -279, base, -108.5, -201, 1060)
    released = [case, strap, pad]
    for name in parts:
        if name.startswith(
            ("Aspen SO007", "Airfasco MS", "Upper strap spacer", "Upper strap shim")
        ):
            released.append(name)
    report = {"bodyCount": len(parts), "componentCount": d.rootComponent.allOccurrences.count}
    report["maximumCaseRemovalFixture"] = interference(sweep, parts, released)
    report["retainedStrapControlMm3"] = common(sweep, parts[strap])
    report["locatorContactControlsMm3"] = [
        common(moved(sweep, x=x, y=y), tray) for x, y in [(-2, 0), (2, 0), (0, -2), (0, 2)]
    ]
    old_conflicts = {}
    old_clear = {}
    old_released = {144, 203, 204, 209, 210, 211, 216, 217, 218}
    for occ in r9.rootComponent.allOccurrences:
        for body in occ.bRepBodies:
            if (
                occ.component.name == "R7 chassis"
                and body.name.startswith("R7 ")
                and body.name.split()[1].isdigit()
                and int(body.name.split()[1]) in old_released
            ):
                continue
            if overlap(dimensions(sweep), dimensions(body)):
                solid = registered_solid(occ, body)
                if overlap(dimensions(sweep), dimensions(solid)):
                    v = common(sweep, solid)
                    (old_conflicts if v > 0.01 else old_clear)[body.name] = v
    report["maximumCaseRemovalR9"] = old_conflicts
    report["r9NarrowPhaseBodies"] = sorted(old_clear)
    tools = {}
    for y in (-298, -182):
        x = -200
        screw = pick(f"Aspen SO007-1032X34 strap screw {y}")
        nut = pick(f"Airfasco MS21044N3 strap locknut envelope {y}")
        tools[f"top key {y}"] = interference(rod((x, y, 383.52), (x, y, 480), 4.59), parts, [screw])
        tools[f"below flange deep socket {y}"] = interference(
            rod((x, y, 289), (x, y, 372.993), 16.0), parts, [screw, nut]
        )
        near, far = sorted((y, -350 if y == -298 else -135))
        tools[f"side entry {y}"] = interference(box(x - 8, near, 289, x + 8, far, 314), parts, [])
    report["strapToolCollisions"] = tools
    pin = pick("Ganter GN")
    guides = [
        parts["Outer guide with double shear lock bore"],
        parts["Inner sliding tube with lock bore"],
    ]
    rod_gauge = rod((13.9, -19, 305), (151, -19, 305), 9.96)
    handle = rod((53.5, -19, 305), (151, -19, 305), 14.0)
    report["pinRetractedExtraction"] = [
        interference(gauge, parts, [pin]) for gauge in (rod_gauge, handle)
    ]
    report["pinEngagedWithdrawalControlMm3"] = common(moved(parts[pin], x=3), guides[0])
    report["undrilledStationControlsMm3"] = [common(moved(rod_gauge, z=14), g) for g in guides]
    moving = [
        b.name
        for o in d.rootComponent.allOccurrences
        if o.component.name == "Bench bronze nut and carrier"
        for b in o.bRepBodies
    ]
    fixed = [
        pick(p)
        for p in [
            "Outer guide",
            "Machined flanged",
            "Cartridge round",
            "Upper hardened",
            "Thomson SRA3708S",
        ]
    ]
    report["carrierGuideAndShaftChecks"] = {
        str(z): {
            name: hits
            for name in moving
            if (hits := interference(moved(parts[name], z=z), {f: parts[f] for f in fixed}))
        }
        for z in (0.0, 190.0, 380.0)
    }
    # Conservative exterior prism over the entire 380 mm carrier stroke.
    # The screw belongs inside this prism, so test only the guide walls here.
    extents = [dimensions(parts[name]) for name in moving]
    span = [min(b[i] for b in extents) for i in range(3)]
    span.extend(max(b[i] for b in extents) for i in range(3, 6))
    span[5] += 380
    travel_gauge = box(*span)
    report["carrierGuideSweepMm3"] = common(travel_gauge, guides[0])
    report["offsetCarrierGuideControlMm3"] = common(moved(travel_gauge, x=2), guides[0])
    # Move one mating face 0.02 mm into the adjoining solid. The volume
    # change divided by the offset is its supported, projected contact area.
    seat_pairs = [
        ("battery pad to nominal case", pad, case, -0.02),
        ("battery pad to strap", pad, strap, 0.02),
        ("nut flange to carrier", pick("Thomson BN3708S"), pick("Split tapped"), 0.02),
        (
            "lower thrust race to housing",
            pick("EZO F6-14M thrust bearing envelope 117"),
            pick("Machined flanged"),
            0.02,
        ),
        (
            "upper thrust race to housing",
            pick("EZO F6-14M thrust bearing envelope 155"),
            pick("Machined flanged"),
            -0.02,
        ),
        ("shaft to upper abutment", pick("Thomson SRA3708S"), pick("Upper hardened"), -0.02),
        ("retaining nut to lower abutment", pick("Accu HNN-M6"), pick("Lower hardened"), 0.02),
        (
            "housing flange to beam",
            pick("Machined flanged"),
            pick("Crossmember with four rail"),
            0.02,
        ),
        ("lockring to beam", pick("Cartridge round"), pick("Crossmember with four rail"), -0.02),
    ]
    seats = {}
    for label, a, b, dz in seat_pairs:
        seated = common(parts[a], parts[b])
        seats[label] = round((common(moved(parts[a], z=dz), parts[b]) - seated) / abs(dz), 3)
    report["nominalProjectedSeatMm2"] = seats
    for name in [
        "Thomson BN3708S",
        "Thomson SRA3708S",
        "Accu HNN-M6",
        "igus GSM",
        "Split tapped",
        "Insulating top pad",
    ]:
        report.setdefault("registeredBoundsMm", {})[name] = dimensions(parts[pick(name)])
    return report


def verify(report):
    if (
        report["maximumCaseRemovalFixture"]
        or report["maximumCaseRemovalR9"]
        or any(report["strapToolCollisions"].values())
    ):
        raise ValueError("Case removal or service tool obstruction")
    if any(report["pinRetractedExtraction"]) or any(report["carrierGuideAndShaftChecks"].values()):
        raise ValueError("Mast service or carrier obstruction")
    if report["carrierGuideSweepMm3"] > 0.01:
        raise ValueError("Continuous carrier exterior sweep meets guide")
    controls = [
        report["offsetCarrierGuideControlMm3"],
        report["retainedStrapControlMm3"],
        report["pinEngagedWithdrawalControlMm3"],
        *report["undrilledStationControlsMm3"],
        *report["locatorContactControlsMm3"],
    ]
    if min(report["nominalProjectedSeatMm2"].values()) <= 1.0:
        raise ValueError("Missing nominal bearing seat")
    if min(controls) <= 0.01:
        raise ValueError("Required obstructed control did not register")
