"""Compare elastic center-load bending of two simply supported front members."""
import json

import numpy as np


def section(rectangles):
    """Effective horizontal bending inertia and extreme stress per unit moment."""
    pieces = [(b-a,d-c,(a+b)/2,(c+d)/2) for a,b,c,d in rectangles]
    area = sum(w*h for w,h,_,_ in pieces)
    centroid = sum(w*h*np.array([x,z]) for w,h,x,z in pieces)/area
    second = np.zeros((2,2))
    for w,h,x,z in pieces:
        offset = np.array([x,z])-centroid
        second += w*h*(np.outer(offset,offset)+np.diag([w*w/12,h*h/12]))
    # The stepped skin has a product of inertia. Enforce zero orthogonal
    # bending moment when solving for its longitudinal stress gradient.
    gradient = np.linalg.solve(second,[1.,0.])
    corners = [[x,z] for a,b,c,d in rectangles for x in (a,b) for z in (c,d)]
    return 1/gradient[0],max(abs((corners-centroid)@gradient))


def compare():
    half_span,force,modulus = 352.075,1000.,69000.
    shoulder = half_span-210
    center_i,center_stress = section([(0,3.175,0,127),(-6.35,0,101.6,127)])
    end_i,end_stress = section([(-6.35,3.175,0,127)])
    # Integrate M(x) times the unit-load bending moment on both half spans.
    flex = force/(6*modulus)*(shoulder**3/end_i+(half_span**3-shoulder**3)/center_i)
    stress = force/2*max(shoulder*end_stress,half_span*center_stress)
    depth,height,wall = 38.1,50.8,3.175
    inertia = (height*depth**3-(height-2*wall)*(depth-2*wall)**3)/12
    return dict(loadN=force,spanMm=2*half_span,elasticModulusMPa=modulus,
                fittedPlate=dict(centerEffectiveInertiaMm4=center_i,stressMPa=stress,deflectionMm=flex),
                forwardBoxConcept=dict(depthMm=depth,heightMm=height,wallMm=wall,frontFaceShiftMm=34.925,
                                       inertiaMm4=inertia,stressMPa=force*half_span*depth/(4*inertia),
                                       deflectionMm=force*half_span**3/(6*modulus*inertia)))


if __name__=="__main__":
    print(json.dumps(compare(),indent=2))
