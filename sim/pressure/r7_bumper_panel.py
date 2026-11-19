"""Sweep the installed bumper against the three registered subwoofer front panels."""
import argparse
import json
from pathlib import Path

import numpy as np

from field_route_clearance import phase
from r7_dynamics import BUDGET, verify_binding
from run_axial_shear_lift import digest, load
from speaker_surfaces import SpeakerSurfaces
from speaker_trial_inputs import FRAME


def screen(folders, survey, field):
    if digest(survey) != load(BUDGET)["surveySha256"]:
        raise ValueError("R7 physical survey changed")
    articles = [r for r in load(survey)["rows"] if "noodle" in r["name"] or "bumper wood" in r["name"]]
    if len(articles)!=12:
        raise ValueError("Expected eight full cylinders and four backing boards")
    source = SpeakerSurfaces(field)
    panels = []
    try:
        for idx,(part,packet,rotation,translation) in enumerate(source.parts):
            if part.get("name") != "GE-24335":
                continue
            vertices = source.packets[packet][part.get("mesh",part["key"])+"_v"]@rotation.T+translation
            axes = [v[:2] for v in rotation.T if abs(v[2])<1e-8]
            panels.append((idx,part["key"],vertices[:,:2],np.asarray(axes)))
        if len(panels)!=3:
            raise ValueError("Expected the three installed GE-24335 panels")
        states = []
        pins = []
        for folder in folders:
            receipt=load(folder/"result.json")
            if not verify_binding(receipt):
                raise ValueError("Bumper sweep requires the R7 plant")
            if not states:
                q=receipt["initialQ"]
                states.append(np.array([[receipt["start"],*q[:3]]]))
            states.append(phase(folder)[:,:4])
            pins.append(digest(folder/"trace.jsonl"))
        poses=np.vstack(states)
        dt=np.diff(poses[:,0])
        if np.any(dt<=0) or np.max(dt)>.0010001:
            raise ValueError("Sweep needs contiguous dense chassis intervals")
        pos=(np.c_[poses[:,1:3],np.zeros(len(poses))]-[.658,0.,0.])@FRAME.rotation.T+FRAME.origin_mark
        yaw=poses[:,3]+np.deg2rad(FRAME.yaw_degrees)
        axes=np.stack((np.cos(yaw),np.sin(yaw)),axis=1)
        right=np.stack((-axes[:,1],axes[:,0]),axis=1)
        translation_step=np.linalg.norm(np.diff(pos[:,:2],axis=0),axis=1)
        yaw_step=np.abs(np.diff(yaw))
        worst=None
        material=None
        for article in articles:
            low,high=np.array(article["bboxMm"]).reshape(2,3)/1000
            mid=(low+high)/2
            half=(high-low)[:2]/2+.0015
            center=pos[:,:2]+axes*mid[0]+right*mid[1]
            pad=translation_step+yaw_step*(np.linalg.norm(mid[:2])+np.linalg.norm(half))+1e-8
            for idx,key,vertices,wall_axes in panels:
                # SAT on the two fixed panel axes and two rotating bumper axes.
                # A positive gap on any axis separates both projected volumes.
                gaps=[]
                for a in wall_axes:
                    lo,hi=(vertices@a).min(),(vertices@a).max()
                    extent=half[0]*abs(axes@a)+half[1]*abs(right@a)
                    at=center@a
                    gaps.append(np.maximum(lo-at,at-hi)-extent)
                for a,h in ((axes,half[0]),(right,half[1])):
                    # Project the panel's oriented bounding rectangle. It encloses every triangle.
                    ext=[np.ptp(vertices@axis)/2 for axis in wall_axes]
                    wall_center=sum(axis*((vertices@axis).min()+length) for axis,length in zip(wall_axes,ext))
                    span=sum(length*abs(a@axis) for axis,length in zip(wall_axes,ext))
                    gaps.append(abs(np.sum((center-wall_center)*a,axis=1))-h-span)
                lower=np.maximum.reduce(gaps)[:-1]-pad
                i=int(np.argmin(lower))
                if worst is None or lower[i]<worst["axisGapM"]:
                    worst=dict(axisGapM=float(lower[i]),part=article["name"],panel=key,timeS=poses[i,0])
                if lower[i]>=0 or "noodle" not in article["name"]:
                    continue
                # A point on a field triangle strictly inside the bare finite
                # cylinder witnesses material overlap independently of the padded boxes.
                direction=int(np.argmax(high-low))
                radial=[j for j in range(3) if j!=direction]
                radius=(high[radial[0]]-low[radial[0]])/2
                if not np.isclose(radius,(high[radial[1]]-low[radial[1]])/2) or not np.isclose(
                    np.pi*radius**2*(high-low)[direction]*1e9,article["volumeMm3"],rtol=1e-6):
                    raise ValueError("Cushion is not the registered full cylinder")
                samples=np.tile(mid,(151,1))
                samples[:,direction]=np.linspace(low[direction],high[direction],len(samples))
                rot=np.array([[*axes[i],0],[*right[i],0],[0,0,1]])
                _,_,near=source.nearest(idx,samples@rot+pos[i])
                local=(near-pos[i])@rot.T
                inside=np.minimum(radius-np.linalg.norm(local[:,radial]-mid[radial],axis=1),
                                  (high-low)[direction]/2-abs(local[:,direction]-mid[direction]))
                j=int(inside.argmax())
                if inside[j]>0 and (material is None or inside[j]>material["insideM"]):
                    material=dict(part=article["name"],panel=key,timeS=poses[i,0],insideM=float(inside[j]),
                                  fieldPointM=near[j].tolist(),robotPointM=local[j].tolist())
        return dict(survey=digest(survey),field=source.pins,traces=pins,beginS=poses[0,0],endS=poses[-1,0],
                    intervals=len(dt),clear=worst["axisGapM"]>0,closestBound=worst,materialWitness=material)
    finally:
        source.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("survey","field"):
        p.add_argument(name,type=Path)
    p.add_argument("phases",type=Path,nargs="+")
    a=p.parse_args()
    print(json.dumps(screen(a.phases,a.survey,a.field),indent=2))
