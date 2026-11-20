"""Survey the actual bumper-zone steel/aluminum frame, its support and front weld lands."""
import math
from physical_bumper import dimensions, registered_solid
from physical_front import FRONT, REAR
import adsk.core as core
import adsk.fusion as fusion


def hull(pts):
    def half(rows):
        result=[]
        for c in rows:
            while len(result)>1:
                a,b=result[-2:]
                if (b[0]-a[0])*(c[1]-a[1])>(b[1]-a[1])*(c[0]-a[0])+1e-8: break
                result.pop()
            result.append(c)
        return result[:-1]
    ordered=sorted(set(pts))
    return half(ordered)+half(reversed(ordered))


def inspect(d):
    mgr=fusion.TemporaryBRepManager.get()
    entries={}
    for occ in d.rootComponent.allOccurrences:
        for body in occ.bRepBodies:
            s=registered_solid(occ,body)
            entries[body.name]=(s,dimensions(s),occ.component.name)
    family={name for name,(_,_,comp) in entries.items() if comp=='R9 front frame'
            or name.startswith(('R7 592 ','R7 595 ','R7 596 ','R7 597 ')) or name.endswith('Tab-mounted rectangular rail')}
    pts=[]
    for name in family:
        s,bb,_=entries[name]
        if bb[2]<0 or bb[5]>190.5: raise ValueError('Frame survey above bumper zone')
        pts.extend(tuple(round(v*10,5) for v in vertex.geometry.asArray()[:2]) for vertex in s.vertices)
    boundary=hull(pts)
    edges=list(zip(boundary,boundary[1:]+boundary[:1]))
    perimeter=sum(math.dist(a,b) for a,b in edges)
    def distance(x,y):
        return min(((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]))/math.dist(a,b) for a,b in edges)
    offenders=[]
    tight=999.
    for name,(s,bb,comp) in entries.items():
        key=int(name.split()[1]) if name.startswith('R7 ') else -1
        if name in family or comp.startswith(('R9 removable','R9 bumper')) or key in (
                *range(487,503),593,594,*range(598,612)):continue
        margin=min(distance(x,y) for x in [bb[0],bb[3]] for y in [bb[1],bb[4]])
        if margin<-.01:offenders.append((name,margin))
        if key in (20,54):tight=min(tight,margin)
    def contact(a,b,shift):
        t=mgr.copy(a)
        m=core.Matrix3D.create();m.translation=core.Vector3D.create(*(v/10 for v in shift))
        if not mgr.transform(t,m) or not mgr.booleanOperation(t,b,fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Frame face check failed')
        return t.volume*1000/.01 if t.isValid and t.lumps.count else 0.
    front=next(s for name,(s,_,_) in entries.items() if name.startswith('R7 592 '))
    anchor=entries['Inner crossmember 711.2 mm'][0]
    welds=[]
    for name,(s,_,_) in entries.items():
        if name.startswith('Nose spacer'):
            welds.append(dict(spacer=name,innerEndMm2=contact(s,anchor,[-.01,0,0]),outerEndMm2=contact(s,front,[.01,0,0])))
    return dict(frameMemberBodies=sorted(family),hullVerticesMm=boundary,hullPerimeterMm=perimeter,
        enclosingRectanglePerimeterMm=2*(FRONT+REAR+711.2),nominalRectangleMarginMm=3048-2*(FRONT+REAR+711.2),
        forwardCarrierClearanceMm=tight,otherBodyBoxFailures=offenders,
        method='Convex hull of actual fixed straight-edged frame vertices in the bumper zone. All other installed non-bumper body AABB corners tested.',
        noseSpacerContacts=welds)
