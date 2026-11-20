"""Create the ground-supported frame study and check four service paths on R9."""

import json
import sys
from pathlib import Path

from adsk import core, fusion
from physical_bumper import LIB, design, dimensions, native, registered_solid, sha
from passive_service_fit import common, moved, overlap
from stage_climber import rod

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "cad/service-readiness/ground-rails"))
from interface import (
    BORE,
    BOTTOM,
    GRIP,
    GRIP_LENGTH,
    LENGTH,
    OFFSET,
    PARENT,
    SHAFT,
    STATIONS,
    TOP,
    retired,
    station_label,
)


def new_component(design_, name, origin=(0, 0, 0)):
    transform = core.Matrix3D.create()
    transform.translation = core.Vector3D.create(*(v / 10 for v in origin))
    child = design_.rootComponent.occurrences.addNewComponent(transform)
    child.component.name = name
    child.isGrounded = True
    return child.component


def copy_frame(source, target):
    frame = json.loads((ROOT / "cad/assembly-readiness/frame-fit.json").read_bytes())
    if frame["archiveSha256"] != PARENT:
        raise ValueError("Frame evidence revision changed")
    members = set(frame["measurement"]["frameMemberBodies"])
    sleeves = {station_label(x, y) for x, y in STATIONS}
    names = members | sleeves
    seen = set()
    for occ in source.rootComponent.allOccurrences:
        selected = [b for b in occ.bRepBodies if b.name in names]
        if not selected:
            continue
        child = new_component(target, occ.component.name + " ground interfaces")
        base = child.features.baseFeatures.add()
        base.name = "Registered R9 frame solids"
        base.startEdit()
        try:
            for b in selected:
                if b.name in seen:
                    raise ValueError("Duplicate frame member: " + b.name)
                child.bRepBodies.add(registered_solid(occ, b), base).name = b.name
                seen.add(b.name)
        finally:
            base.finishEdit()
        for original in selected:
            b = child.bRepBodies.itemByName(original.name)
            material, color = original.material, original.appearance
            b.material = target.materials.itemByName(material.name) or target.materials.addByCopy(
                material, material.name
            )
            b.appearance = target.appearances.itemByName(
                color.name
            ) or target.appearances.addByCopy(color, color.name)
    if seen != names or len(seen) != 16:
        raise ValueError("Frame and welded sleeve identity mismatch")


def gauge(target):
    component = new_component(target, "R13 removable rail inspection gauge", (0, 450, 70))

    def extrude(diameter, length, operation, name, plane):
        sk = component.sketches.add(plane)
        sk.name = name + " diameter"
        circle = sk.sketchCurves.sketchCircles.addByCenterRadius(
            core.Point3D.create(0, 0, 0), diameter / 20
        )
        if not circle.centerSketchPoint.isFullyConstrained:
            circle.centerSketchPoint.isFixed = True
        dim = sk.sketchDimensions.addDiameterDimension(
            circle, core.Point3D.create(diameter / 10, 0, 0)
        )
        dim.parameter.expression = f"{diameter} mm"
        cmd = component.features.extrudeFeatures.createInput(sk.profiles.item(0), operation)
        cmd.setOneSideExtent(
            fusion.DistanceExtentDefinition.create(core.ValueInput.createByString(f"{length} mm")),
            fusion.ExtentDirections.PositiveExtentDirection,
        )
        feature = component.features.extrudeFeatures.add(cmd)
        feature.name = name
        return feature

    stem = extrude(
        SHAFT,
        LENGTH,
        fusion.FeatureOperations.NewBodyFeatureOperation,
        "Gauge stem",
        component.xYConstructionPlane,
    )
    extrude(
        GRIP,
        GRIP_LENGTH,
        fusion.FeatureOperations.JoinFeatureOperation,
        "Grip and stop",
        stem.endFaces.item(0),
    )
    b = component.bRepBodies.item(0)
    b.name = "Removable 8 mm stepped inspection gauge"
    material = (
        core.Application.get()
        .materialLibraries.itemByName(LIB)
        .materials.itemByName("Aluminum 6061")
    )
    b.material = material


def build(parent, archive):
    if sha(parent) != PARENT:
        raise ValueError("Expected parked R9")
    app = core.Application.get()
    source, target = native(parent), None
    try:
        target = app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        target.name = "R13 ground-supported frame and rail service"
        d = design(target)
        d.designIntent = fusion.DesignIntentTypes.HybridDesignIntentType
        d.designType = fusion.DesignTypes.ParametricDesignType
        copy_frame(design(source), d)
        gauge(d)
        app.activeViewport.fit()
        if not d.exportManager.execute(
            d.exportManager.createFusionArchiveExportOptions(str(archive))
        ):
            raise RuntimeError("Ground frame export")
    finally:
        if target and target.isValid:
            target.close(False)
        source.close(False)


def observations(fixture, whole):
    # Signed station coordinates distinguish physically distinct rail sleeves.
    hosts = {
        b.name: registered_solid(o, b)
        for o in fixture.rootComponent.allOccurrences
        for b in o.bRepBodies
    }
    probe = hosts["Removable 8 mm stepped inspection gauge"]
    if abs(probe.volume * 1000 - 4297.6988) > 0.02 or len(hosts) != 17:
        raise ValueError("Gauge and frame inventory")
    all_bodies = [(o, b) for o in whole.rootComponent.allOccurrences for b in o.bRepBodies]
    confirmed = []
    for occ, body in all_bodies:
        if body.name not in hosts:
            continue
        copy = hosts[body.name]
        original = registered_solid(occ, body)
        # Copied document materials may acquire a new ID. The Fusion physical
        # property IDs and actual density still identify the same stock.
        got = next(
            b
            for o in fixture.rootComponent.allOccurrences
            for b in o.bRepBodies
            if b.name == body.name
        )
        density = lambda b: b.material.materialProperties.itemById("structural_Density").value
        if (
            abs(copy.volume - original.volume) > 0.0001
            or dimensions(copy) != dimensions(original)
            or got.material.name != body.material.name
            or abs(density(got) - density(body)) > 0.01
        ):
            raise ValueError("Parked source geometry or material changed: " + body.name)
        confirmed.append(body.name)
    if len(confirmed) != 16:
        raise ValueError("Missing parent frame member or sleeve")
    selected_count = sum(not retired(o.component.name, b.name) for o, b in all_bodies)
    if len(all_bodies) != 923 or selected_count != 892:
        raise ValueError("Whole robot and ground state inventory")
    stations = []
    for x, y in STATIONS:
        # Each translated cylinder sweeps exactly along its own axis. Start
        # 60 mm above the seated pose and include the complete stroke.
        at_seat = moved(probe, x, y - 450, TOP - LENGTH - 70)
        stem_sweep = rod((x, y, TOP - LENGTH), (x, y, TOP + 60), SHAFT)
        grip_sweep = rod((x, y, TOP), (x, y, TOP + GRIP_LENGTH + 60), GRIP)
        gauges = [stem_sweep, grip_sweep]
        bounds = [dimensions(g) for g in gauges]
        before, after = {}, {}
        for occ, body in all_bodies:
            candidate = dimensions(body)
            if not any(overlap(limits, candidate) for limits in bounds):
                continue
            solid = registered_solid(occ, body)
            # Positive volumes here are individual sweep intersections. The
            # two swept cylinders overlap; do not sum their volumes.
            hits = [common(g, solid) for g in gauges]
            if max(hits) > 0.01:
                key = occ.fullPathName + " / " + body.name
                before[key] = hits
                if not retired(occ.component.name, body.name):
                    after[key] = hits
        sleeve = hosts[station_label(x, y)]
        adverse = common(moved(at_seat, x=OFFSET), sleeve)
        contact = common(moved(at_seat, z=-0.02), sleeve) / 0.02
        nominal = common(at_seat, sleeve)
        expected_bolt = f"MISUMI CB8-100 {x} {y}"
        if (
            not any(name.endswith(" / " + expected_bolt) for name in before)
            or after
            or nominal > 0.01
            or adverse < 30
            or contact < 50
        ):
            raise ValueError(
                f"Rail service fit {x}, {y}: {before}, {after}, {nominal}, {adverse}, {contact}"
            )
        stations.append(
            dict(
                centerMm=[x, y],
                beforeHitsMm3=before,
                groundHitsMm3=after,
                nominalSleeveOverlapMm3=nominal,
                offsetSleeveOverlapMm3=adverse,
                projectedStopSeatMm2=contact,
            )
        )
    return dict(
        fixtureBodies=len(hosts),
        parentBodies=len(all_bodies),
        selectedParkedBodies=selected_count,
        verifiedParkedFrameBodies=sorted(confirmed),
        gaugeVolumeMm3=round(probe.volume * 1000, 4),
        strokeMm=60,
        stationResults=stations,
    )
