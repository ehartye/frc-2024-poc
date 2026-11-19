"""Measure a reopened frame in millimetres and cubic millimetres."""
import math
from collections import Counter

import adsk.core
import adsk.fusion


def inspect(root):
    temp = adsk.fusion.TemporaryBRepManager.get()
    positions = {}
    counts = Counter()
    measurements = []
    rail_axes = set()
    sleeve_axes = set()
    bolt_axes = set()
    nut_axes = set()
    maximum_volume_error = 0.
    for occ in root.allOccurrences:
        if occ.component.bRepBodies.count != 1:
            raise ValueError("Each frame article must reopen as one solid")
        name = occ.component.name
        source = occ.component.bRepBodies.item(0)
        body = temp.copy(source)
        if not temp.transform(body,occ.transform2) or not body.isSolid:
            raise ValueError("Could not register frame body")
        box = body.preciseBoundingBox
        bounds = [round(v*10,5) for p in (box.minPoint,box.maxPoint) for v in p.asArray()]
        counts[name] += 1
        cx,cy = (round((bounds[i]+bounds[i+3])/2,3) for i in (0,1))
        positions.setdefault(name,[]).append([cx,cy])
        volume = source.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume*1000
        if name == "Swerve frame - interchangeable drilled rail":
            expected = (38.1*50.8-31.75*44.45)*480-4*math.pi*2.55**2*6.35
            maximum_volume_error = max(maximum_volume_error,abs(volume-expected))
            thicknesses = {}
            for face in body.faces:
                cyl = adsk.core.Cylinder.cast(face.geometry)
                if not cyl or abs(cyl.radius-.255) > 1e-5 or abs(cyl.axis.z)<.99999:
                    continue
                key = (round(cyl.origin.x*10,3),round(cyl.origin.y*10,3))
                b = face.boundingBox
                thicknesses.setdefault(key,set()).add((round(10*b.minPoint.z,3),round(10*b.maxPoint.z,3)))
            if len(thicknesses)!=4 or any(t!={(69.85,73.025),(117.475,120.65)} for t in thicknesses.values()):
                raise ValueError(f"Unexpected rail drill spans: {thicknesses}")
            rail_axes.update(thicknesses)
        elif name == "Swerve frame - bolt crush sleeve":
            radius, flat = 4.75, 3.05
            removed = radius**2*math.acos(flat/radius) - flat*math.sqrt(radius**2-flat**2)
            expected = (math.pi*(radius**2-2.55**2)-removed)*44.45
            bore = next(c for face in body.faces if (c:=adsk.core.Cylinder.cast(face.geometry))
                        and abs(c.radius-.255)<1e-6)
            cx,cy = round(10*bore.origin.x,3), round(10*bore.origin.y,3)
            maximum_volume_error = max(maximum_volume_error,abs(volume-expected))
            if abs(cx)>300:
                to_wall = bounds[3]-cx if cx>-329.85 else cx-bounds[0]
            else:
                rail_y = 329.85 if cy>0 else -329.85
                to_wall = bounds[4]-cy if cy>rail_y else cy-bounds[1]
            if not math.isclose(to_wall, flat, abs_tol=.001):
                raise ValueError("Sleeve flat faces away from the adjacent tube wall")
            if bounds[2::3]!=[73.025,117.475]:
                raise ValueError("Sleeve ends must touch both inner tube walls")
            sleeve_axes.add((cx,cy))
        elif name == "10-32 x 2.75 socket head":
            if bounds[2::3] != [54.4625,129.1385]:
                raise ValueError("Wrong bolt seating or projection")
            bolt_axes.add((cx,cy))
        elif name == "10-32 locknut":
            if bounds[2::3] != [59.2391,65.5637]:
                raise ValueError("Nut does not seat on the lower housing")
            nut_axes.add((cx,cy))
        measurements.append(dict(article=name,boundsMm=bounds,volumeMm3=round(volume,4)))
    expected_axes = {(x,y+delta) for x in (-228.25,228.25) for y in (-329.85,329.85) for delta in (-12.7,12.7)}
    expected_axes.update({(-329.85+delta,y) for y in (-228.25,228.25) for delta in (-12.7,12.7)})
    expected_axes = {(round(x,3),round(y,3)) for x,y in expected_axes}
    if any(group != expected_axes for group in (rail_axes,sleeve_axes,bolt_axes,nut_axes)):
        raise ValueError(f"Unexpected bore, sleeve, bolt or nut axes: {[g ^ expected_axes for g in (rail_axes,sleeve_axes,bolt_axes,nut_axes)]}")
    if maximum_volume_error > .1:
        raise ValueError(f"Reopened rail or sleeve volume mismatch {maximum_volume_error}")
    pedestals = positions.get("Deck support - open socket pedestal", [])
    if sorted(pedestals)!=sorted([[x,y] for x in (-266.7,-139.7) for y in (-80,80)]):
        raise ValueError("Deck seats do not match the original beam attachment stations")
    if sum(counts.values())!=46 or len(counts)!=7:
        raise ValueError(f"Frame inventory mismatch: {counts}")
    return dict(articleCounts=dict(counts),alignedBoltStacks=len(rail_axes),deckBeamSeats=len(pedestals),
                maximumRailOrSleeveVolumeErrorMm3=maximum_volume_error,articles=measurements)
