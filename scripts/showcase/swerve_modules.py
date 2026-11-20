"""Project-authored display solids, dimensioned from the installed MAXSwerve package.

Dimensions and offsets are mm in the front-left glTF frame, relative to the
wheel center. Meshes are analytic primitives shared by the four stations.
"""
import math


def primitive(model, accessor, kind, tint):
    positions, normals, indices = [], [], []

    def face(points, normal):
        start = len(positions)
        positions.extend(points)
        normals.extend([normal] * 4)
        indices.extend(start + n for n in (0, 1, 2, 0, 2, 3))

    if kind == "box":
        for axis in range(3):
            for sign in (-1, 1):
                normal = [0, 0, 0]
                normal[axis] = sign
                corners = []
                for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    point = normal.copy()
                    point[(axis + 1) % 3] = u * sign
                    point[(axis + 2) % 3] = v
                    corners.append([coord / 2 for coord in point])
                face(corners, normal)
    else:
        # Unit diameter and height. A through bore leaves the carrier visible.
        bore = .40 if kind == "ring" else 0
        for sector in range(48):
            a, b = [math.tau * i / 48 for i in (sector, sector + 1)]
            def p(angle, radius, level):
                return (math.cos(angle)*radius, level, math.sin(angle)*radius)
            nx, nz = math.cos((a+b)/2), math.sin((a+b)/2)
            face([p(a,.5,-.5), p(a,.5,.5), p(b,.5,.5), p(b,.5,-.5)], (nx,0,nz))
            if bore:
                face([p(b,bore,-.5), p(b,bore,.5), p(a,bore,.5), p(a,bore,-.5)], (-nx,0,-nz))
            for y in (-.5, .5):
                cap = [p(a,bore,y), p(b,bore,y), p(b,.5,y), p(a,.5,y)]
                face(cap if y > 0 else cap[::-1], (0,2*y,0))
    material = len(model["materials"])
    model["materials"].append(dict(name=tint[0], pbrMetallicRoughness=dict(
        baseColorFactor=[*tint[1:], 1], metallicFactor=.48, roughnessFactor=.46)))
    attributes = dict(POSITION=accessor(positions,"VEC3",bounds=True), NORMAL=accessor(normals,"VEC3"))
    index = len(model["meshes"])
    model["meshes"].append(dict(primitives=[dict(attributes=attributes, indices=accessor(indices,"SCALAR",5125),
                                                 material=material)]))
    return index


def module_meshes(model, accessor):
    aluminum = ("Machined aluminum", .55, .62, .68)
    graphite = ("Anodized housing", .11, .14, .17)
    return dict(rail=primitive(model, accessor, "box", graphite),
                ring=primitive(model, accessor, "ring", graphite),
                motor=primitive(model, accessor, "cylinder", ("Motor can", .16, .18, .20)),
                cheek=primitive(model, accessor, "box", aluminum),
                hub=primitive(model, accessor, "cylinder", aluminum))


def install_module(model, meshes, station, fixed, steer, roll):
    nodes = model["nodes"]

    def part(parent, name, mesh, center, size, transverse=False):
        node = dict(name=station+" "+name, mesh=meshes[mesh],
                    translation=[v*.001 for v in center], scale=[v*.001 for v in size])
        if transverse:
            node["rotation"] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        nodes[parent].setdefault("children", []).append(len(nodes))
        nodes.append(node)

    for tier, y in (("lower", 34), ("upper", 80)):
        part(fixed, tier+" housing", "ring", (0,y,0), (88.8,12.7,88.8))
        part(fixed, tier+" forward mounting ear", "rail", (58,y,-35), (60,12.7,15))
        part(fixed, tier+" side mounting ear", "rail", (35,y,-58), (15,12.7,60))
    for i, (x, z) in enumerate(((-35,35), (35,35), (-35,-35))):
        part(fixed, "housing spacer "+str(i+1), "hub", (x,57,z), (12,46,12))
    part(fixed, "NEO drive motor", "motor", (0,104,0), (60,58,60))
    part(fixed, "drive end cap", "hub", (0,131,0), (57,4,57))
    part(fixed, "NEO 550 steering motor", "motor", (35,108,-35), (35,48,35))
    part(fixed, "steering end cap", "hub", (35,130,-35), (32,4,32))
    for side, z in (("left",-24), ("right",24)):
        part(steer, side+" fork plate", "cheek", (0,14,z), (48,40,12))
        part(steer, side+" axle boss", "hub", (0,0,z), (20,15,20), True)
    part(steer, "azimuth carrier", "ring", (0,35,0), (79,10,79))
    part(roll, "wheel hub", "hub", (0,0,0), (42,29,42), True)
