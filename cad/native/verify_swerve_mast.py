"""Measure the six installed mast articles, their seats and surrounding material."""
import math
from pathlib import Path
import runpy

import adsk.core
import adsk.fusion


def inspect(design):
    spec = runpy.run_path(str(Path(__file__).with_name("swerve_mast_support.py")))
    root = design.rootComponent
    temp = adsk.fusion.TemporaryBRepManager.get()
    assembly = next(o for o in root.occurrences if o.component.name == spec["ASSEMBLY"])
    if any(o.component.name.startswith(spec["LEGACY_PREFIXES"]) for o in root.occurrences):
        raise ValueError("Superseded bridge mounting hardware remains installed")

    def material(occ, body):
        solid = temp.copy(body.nativeObject or body)
        if not temp.transform(solid,occ.transform2):
            raise ValueError("Failed to register native solid")
        return solid

    def volume(left,right):
        result = temp.copy(left)
        if not temp.booleanOperation(result,right,adsk.fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError("Native intersection failed")
        return 1000 * result.volume if result.isValid and result.lumps.count else 0.

    def bounds(b):
        return [v*10 for point in (b.minPoint,b.maxPoint) for v in point.asArray()]

    def shifted(solid,delta):
        result = temp.copy(solid)
        transform = adsk.core.Matrix3D.create()
        transform.translation = adsk.core.Vector3D.create(*(v/10 for v in delta))
        if not temp.transform(result,transform):
            raise ValueError("Failed to move contact gauge")
        return result

    articles = {}
    measurements = []
    for occ in root.allOccurrences:
        if not occ.fullPathName.startswith(assembly.name+"+"):
            continue
        if occ.bRepBodies.count!=1:
            raise ValueError("Mast article must contain one solid")
        body = material(occ,occ.bRepBodies.item(0))
        box = bounds(body.preciseBoundingBox)
        web = occ.component.name==spec["WEB"]
        if not web and occ.component.name!=spec["TUBE"]:
            raise ValueError("Unexpected mast support component")
        expected = 8*(115*4.6+218*62.265) if web else 12*(25.4*50.8-19.05*44.45)
        quantity = 1000*body.volume
        if abs(quantity-expected)>.01:
            raise ValueError(f"Mast article volume mismatch: {quantity} vs {expected}")
        articles[occ.fullPathName] = body,box,web
        measurements.append(dict(article=occ.component.name,boundsMm=box,volumeMm3=quantity))
    if len(articles)!=6 or sum(a[2] for a in articles.values())!=2:
        raise ValueError("Expected two webs and four extensions")
    rail_seats, bridge_seats, web_seats = {},{},{}
    intersections = []
    module_solids = 0
    for occ in root.allOccurrences:
        path = occ.fullPathName
        name = occ.component.name
        module = name.startswith(("Fixed housing - ","MAXSwerve Front","MAXSwerve Rear"))
        if not module and not occ.isVisible:
            continue
        top = root.occurrences.itemByName(path.split("+")[0])
        role = top.component.attributes.itemByName("frcDisplay","role") if top else None
        if role and role.value=="reserved-space":
            continue
        for body in occ.bRepBodies:
            if not body.isSolid or (not module and not body.isVisible):
                continue
            module_solids += int(module)
            native = None
            other = bounds(body.boundingBox)
            for key,(article,box,web) in articles.items():
                if key==path or any(box[k]>other[k+3]+.02 or other[k]>box[k+3]+.02 for k in range(3)):
                    continue
                if native is None:
                    native = material(occ,body)
                overlap = volume(article,native)
                if overlap>.01:
                    intersections.append([key,body.name,overlap])
                sign = 1 if box[1]>0 else -1
                if web and name=="Swerve frame - interchangeable drilled rail":
                    seats,delta = rail_seats,(0,0,-.01)
                elif not web and body.name.startswith("frame fastening bridge-"):
                    seats,delta = bridge_seats,(0,-sign*.01,0)
                elif not web and name==spec["WEB"]:
                    seats,delta = web_seats,(0,sign*.01,0)
                else:
                    continue
                area = (volume(shifted(article,delta),native)-overlap)/.01
                if area>1:
                    if key in seats:
                        raise ValueError("Duplicate seat on a mast article")
                    seats[key] = area
    if module_solids!=1092 or intersections:
        raise ValueError(f"Module count {module_solids}, material intersections {intersections}")
    for seats,count,expected in ((rail_seats,2,920),(bridge_seats,4,443.5475),(web_seats,4,443.5475)):
        if len(seats)!=count or any(not math.isclose(a,expected,abs_tol=.1) for a in seats.values()):
            raise ValueError(f"Missing or incomplete bearing face: {list(seats.values())}")
    return dict(articles=measurements,moduleSolids=module_solids,intersections=intersections,
                railSeatAreasMm2=sorted(rail_seats.values()),bridgeSeatAreasMm2=sorted(bridge_seats.values()),
                extensionToWebAreasMm2=sorted(web_seats.values()))
