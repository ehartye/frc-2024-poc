"""Refine the low Stage plate candidates using the R7 tires and a height slice of the NOTE."""
import argparse
import json
from pathlib import Path

import numpy as np

from r7_dynamics import BUDGET, LAYOUT
from r7_route_clearance import FRAME
from run_axial_shear_lift import digest, load
from wing_speaker_passage import MAJOR, TUBE


def check(route, survey, field, broad):
    report = load(broad)
    if (report["piece"] != "blue-wing-inner" or digest(survey) != load(BUDGET)["surveySha256"] or
        digest(field) != report["fieldSha256"]):
        raise ValueError("Expected the registered inner-wing R7 screen")
    pairs = report["candidatePairs"]
    stage = "FE-2024/GE-24200_Blue/GE-24210/FE-01021/"
    if not pairs or any(row["field"] not in (stage+"FE-01020/FE-01020-01", stage+"Hook_4_24") or
                        row["robot"] not in ("moving NOTE", "Front left full module azimuth envelope",
                                             "Front right full module azimuth envelope") for row in pairs):
        raise ValueError("This refinement applies only to the observed NOTE, tires and Stage feet")
    field_names = {row["field"] for row in pairs}
    panels = np.array([r["bounds"] for r in load(field)["parts"] if "/".join(r["path"]) in field_names])
    west, ceiling = panels[:,0,0].min(), panels[:,1,2].max()
    traces = []
    poses = []
    for phase in ("pickup","carry","shot"):
        folder = route/("inner-"+phase)
        traces.append(digest(folder/"trace.jsonl"))
        if not poses:
            first = load(folder/"result.json")
            poses.append((first["start"],first["initialQ"]))
        with (folder/"trace.jsonl").open() as stream:
            poses.extend((row.get("time",row.get("t")),row["q"]) for row in map(json.loads,stream))
    if traces != report["traces"]:
        raise ValueError("The inner-wing route changed")
    times = np.array([t for t,q in poses])
    q = np.array([q for t,q in poses])
    step = np.diff(times)
    if np.any(step <= 0) or step.max() > .0010001:
        raise ValueError("Incomplete dense route")
    data = load(survey)["rows"]
    layout = load(LAYOUT)
    chassis = (np.c_[q[:,:2],np.zeros(len(q))]-[.658,0,0]) @ FRAME.rotation.T + FRAME.origin_mark
    yaw = q[:,2]+np.radians(FRAME.yaw_degrees)
    tire_gaps, housing_gaps = {}, {}
    for name in ("Front left","Front right"):
        rows = [row for row in data if row["path"].startswith("MAXSwerve "+name+" ")]
        carrier = [v for v in rows if v["name"].endswith(" carrier")]
        rotating = [v for v in rows if v["name"].endswith(" wheel")]
        if len(carrier) != 92 or len(rotating) != 9 or min(v["bboxMm"][2] for v in carrier)/1000 <= ceiling+1e-5:
            raise ValueError("Unregistered carrier or low carrier hardware")
        # Steering preserves the carrier heights. Enclose all nine rolling bodies
        # in a sphere so the result is independent of steering and wheel roll.
        station = layout["stations"][name]
        center = np.array(station["wheelCenterMm"])/1000
        radii = [np.linalg.norm(np.max(abs(np.array(v["bboxMm"]).reshape(2,3)/1000-center),axis=0)) for v in rotating]
        radius = max(radii)
        lo, hi = np.array(station["installedBoundsMm"]).reshape(2,3)/1000
        # The installed full-module box encloses every fixed housing body at all times.
        corners = np.array([[x,y] for x in (lo[0],hi[0]) for y in (lo[1],hi[1])])
        corner_x = np.c_[np.cos(yaw[:-1]),-np.sin(yaw[:-1])] @ corners.T
        movement = np.abs(np.diff(yaw))*np.linalg.norm(corners,axis=1).max()
        fixed_x = np.maximum(chassis[:-1,0],chassis[1:,0])+corner_x.max(axis=1)+movement
        housing_gaps[name] = float(west-fixed_x.max()-3e-6)
        cx = center[0]*np.cos(yaw)-center[1]*np.sin(yaw)
        # Independently enclose chassis translation, steering and yaw over each solver step.
        xmax = np.maximum(chassis[:-1,0],chassis[1:,0])+cx[:-1]+np.abs(np.diff(yaw))*np.linalg.norm(center[:2])+radius
        tire_gaps[name] = float(west-xmax.max()-3e-6)
    center = (q[:,-7:-4]-[.658,0,0]) @ FRAME.rotation.T + FRAME.origin_mark
    quat = q[:,-4:]
    if not np.allclose(np.linalg.norm(quat,axis=1),1,rtol=0,atol=1e-8):
        raise ValueError("Invalid NOTE orientation")
    vertical = 1-2*(quat[:,1]**2+quat[:,2]**2)
    axis_change = 4*np.arcsin(np.clip(np.minimum(np.linalg.norm(np.diff(quat,axis=0),axis=1),
                                            np.linalg.norm(quat[1:]+quat[:-1],axis=1))/2,0,1))
    # Lowest possible centerline height over the whole interval. Below that plane
    # the horizontal reach of the circular tube shrinks with its vertical offset.
    low_z = np.minimum(center[:-1,2],center[1:,2])-MAJOR*(np.sqrt(np.maximum(0,1-vertical[:-1]**2))+axis_change)
    dz = np.maximum(0,low_z-ceiling)
    tube_reach = np.sqrt(np.maximum(0,TUBE**2-dz**2))
    near = dz <= TUBE+3e-6
    xmax = np.maximum(center[:-1,0],center[1:,0])+MAJOR+tube_reach
    note_gap = float(west-xmax[near].max()-3e-6)
    return dict(clear=min(note_gap,*tire_gaps.values(),*housing_gaps.values())>0, westPlaneM=west, maximumHeightM=ceiling,
                noteGapM=note_gap, tireGapsM=tire_gaps, fixedHousingGapsM=housing_gaps, intervals=len(step), traces=traces,
                broadPhaseSha256=digest(broad), surveySha256=digest(survey), fieldSha256=digest(field))


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("route","survey","field","broad"):
        p.add_argument(key,type=Path)
    print(json.dumps(check(**vars(p.parse_args())),indent=2, allow_nan=False))
