"""Build an isolated service fixture from the parked R9 robot's original interfaces."""

import adsk.core as core
import adsk.fusion as fusion

from physical_bumper import LIB, design, dimensions, native, registered_solid, sha
from stage_climber import box
import battery_service
import climb_transmission_fixture
import mast_service

PARENT = "4663d9fa55b5f3eb8e23549f3205a2d29248322543beb3e7a7d8076a23f93f6d"
CHASSIS = "R7 chassis"
FIXED = "R8 fixed climb mast"
SLIDE = "R8 sliding saddle"


def include(component, name, bounds):
    if component == CHASSIS:
        ids = {144, 162, 166, 167, 168, *range(203, 231)}
        return (
            name.startswith("R7 ")
            and name.split()[1].isdigit()
            and int(name.split()[1]) in ids
            or name.endswith("Tab-mounted rectangular rail")
            and abs(bounds[0] + 240) < 0.01
        )
    return component in (FIXED, SLIDE, "R9 mast attachment", "R8 transverse lock")


def copy_hosts(source, target):
    groups = {}
    for occ in source.rootComponent.allOccurrences:
        component = occ.component.name
        if component not in (CHASSIS, FIXED, SLIDE, "R9 mast attachment", "R8 transverse lock"):
            continue
        articles = []
        for body in occ.bRepBodies:
            solid = registered_solid(occ, body)
            bounds = dimensions(solid)
            if include(component, body.name, bounds):
                # The battery is a project-dimensioned gauge for geometry checks.
                if body.name == "R7 144 battery":
                    solid = box(*bounds)
                articles.append((body.name, solid, body.material, body.appearance))
        if not articles:
            continue
        child = target.rootComponent.occurrences.addNewComponent(core.Matrix3D.create())
        comp = child.component
        comp.name = component
        base = comp.features.baseFeatures.add()
        base.name = "Registered parked host geometry"
        base.startEdit()
        try:
            for name, solid, _, _ in articles:
                comp.bRepBodies.add(solid, base).name = name
        finally:
            base.finishEdit()
        if comp.bRepBodies.count != len(articles):
            raise ValueError(f"Host body count changed in {component}")
        for made, (name, _, material, appearance) in zip(comp.bRepBodies, articles):
            made.name = name
            if material:
                made.material = target.materials.itemByName(
                    material.name
                ) or target.materials.addByCopy(material, material.name)
            if appearance:
                made.appearance = target.appearances.itemByName(
                    appearance.name
                ) or target.appearances.addByCopy(appearance, appearance.name)
        groups[component] = child
    if set(groups) != {CHASSIS, FIXED, SLIDE, "R9 mast attachment", "R8 transverse lock"}:
        raise ValueError("Expected five first-party host groups")
    return groups


def build(source_path, output):
    if sha(source_path) != PARENT:
        raise ValueError("Expected released R9 archive")
    app = core.Application.get()
    source = native(source_path)
    target = None
    try:
        target = app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        target.name = "R10 Battery and Mast Service Fixture"
        d = design(target)
        d.designIntent = fusion.DesignIntentTypes.HybridDesignIntentType
        d.designType = fusion.DesignTypes.ParametricDesignType
        hosts = copy_hosts(design(source), d)
        lib = app.materialLibraries.itemByName(LIB).materials
        ch = hosts[CHASSIS]
        battery_service.install(d, ch, lib)
        mast_service.install(d, ch, lib)
        climb_transmission_fixture.install(d, ch, lib)
        for child in hosts.values():
            child.isGrounded = True
        app.activeViewport.fit()
        if not d.exportManager.execute(
            d.exportManager.createFusionArchiveExportOptions(str(output))
        ):
            raise RuntimeError("Fixture export failed")
    finally:
        if target and target.isValid:
            target.close(False)
        source.close(False)
