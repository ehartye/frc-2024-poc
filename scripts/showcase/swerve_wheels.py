"""Display four measured tire envelopes with the drivetrain's attained joint positions."""
import math

from swerve_modules import install_module, module_meshes

import numpy as np
from scipy.spatial.transform import Rotation


def tire(radius, width):
    # Local axle is +Y in the plant and -Z in glTF. A light witness sector shows roll.
    vertices, normals, colors, indices = [], [], [], []
    for sector in range(64):
        t0,t1 = [math.tau * i / 64 for i in (sector,sector+1)]
        x0,y0 = math.cos(t0),math.sin(t0)
        x1,y1 = math.cos(t1),math.sin(t1)
        shade = (.53,.60,.64) if sector < 3 else (.075,.084,.093)
        def face(points, ns):
            start = len(vertices)
            vertices.extend(points)
            normals.extend(ns)
            colors.extend([shade]*len(points))
            indices.extend(start+k for k in ((0,1,2,0,2,3) if len(points)==4 else (0,1,2)))
        z = width / 2
        a,b = radius*x0,radius*y0
        c,d = radius*x1,radius*y1
        face([(a,b,-z),(c,d,-z),(c,d,z),(a,b,z)],[(x0,y0,0),(x1,y1,0),(x1,y1,0),(x0,y0,0)])
        face([(0,0,z),(a,b,z),(c,d,z)],[(0,0,1)]*3)
        face([(0,0,-z),(c,d,-z),(a,b,-z)],[(0,0,-1)]*3)
    return vertices,normals,colors,indices


def animate_wheels(model, wheels, states, accessor, channel):
    states = np.asarray(states)
    order = ("Front left","Front right","Rear left","Rear right")
    if states.shape[1:] != (8,) or set(wheels) != set(order):
        raise ValueError("Missing attained module coordinates")
    meshes = module_meshes(model, accessor)
    for i,station in enumerate(order):
        wheel = wheels[station]
        xyz,normal,color,index = tire(wheel["diameterMm"]*.0005,wheel["widthMm"]*.001)
        primitive = dict(attributes=dict(POSITION=accessor(xyz,"VEC3",bounds=True),
                                          NORMAL=accessor(normal,"VEC3"),COLOR_0=accessor(color,"VEC3")),
                         indices=accessor(index,"SCALAR",5125))
        mesh = len(model["meshes"])
        model["meshes"].append(dict(primitives=[primitive]))
        axle = len(model["nodes"])
        x,y,z = wheel["centerMm"]
        model["nodes"].extend([dict(name=station+" steering",children=[axle+1],translation=[x*.001,z*.001,-y*.001]),
                               dict(name=station+" tire envelope",mesh=mesh)])
        model["scenes"][0]["nodes"].append(axle)
        # Plant azimuth is +Z (glTF +Y), and roll is +Y (glTF -Z).
        ax,ay,_ = wheel["nativeRollAxis"]
        zero = math.atan2(ax,-ay)
        fixed = len(model["nodes"])
        model["nodes"].append(dict(name=station+" module housing", translation=model["nodes"][axle]["translation"],
                                   rotation=Rotation.from_euler("y",zero).as_quat().tolist()))
        model["scenes"][0]["nodes"].append(fixed)
        install_module(model, meshes, station, fixed, axle, axle+1)
        steer = Rotation.from_euler("y",(zero+states[:,2*i])[:,None]).as_quat()
        roll = Rotation.from_euler("z",-states[:,2*i+1,None]).as_quat()
        for node,rotation in ((axle,steer),(axle+1,roll)):
            for j in range(1,len(rotation)):
                if np.dot(rotation[j-1],rotation[j]) < 0:
                    rotation[j] *= -1
            model["nodes"][node]["rotation"] = rotation[0].tolist()
            channel([node],"rotation",rotation)
