"""Check successive Speaker arrivals with every previously released NOTE still on the field."""
import argparse
import json
from pathlib import Path

import numpy as np

from counter_reset import centers, collection_path, separate_paths
from run_axial_shear_lift import digest, load
from speaker_trial_inputs import MARKS
from swerve_development_route import DEVELOPMENT_STARTS
from swerve_shot_observation import measure
from wing_speaker_passage import MAJOR, TUBE


def clear_by(prior, score, sensor_box):
    lo, hi = sensor_box
    a,b=prior[:-1,1:4],prior[1:,1:4]
    extent=MAJOR+TUBE+1e-7
    gap=np.maximum(lo-np.maximum(a,b)-extent,np.minimum(a,b)-extent-hi).max(axis=1)
    occupied=np.flatnonzero((prior[1:,0]>=score) & (gap<=0))
    if not len(occupied):
        raise ValueError("Prior detection never occupied the registered sensor region")
    return float(prior[occupied[-1]+1,0])


def verify(route, sensor, continuations):
    if not 1 <= len(continuations) < len(MARKS):
        raise ValueError("Supply one collection continuation for each subsequent release")
    faces=np.array([p for row in load(sensor)["sensorCaseEnds"] for p in row["caseEndCornersM"]])
    box=faces.min(axis=0),faces.max(axis=0)
    histories=[]
    records=[]
    last_shot=None
    for i, mark in enumerate(MARKS[:len(continuations)+1]):
        pickup, carry, shot, flight=[route/(mark+"-"+step) for step in ("pickup","carry","shot","field")]
        obs=measure(pickup,carry,shot,flight,sensor,previous_release=last_shot,roster=DEVELOPMENT_STARTS if i else None)
        arrival=obs["firstSimultaneousFullShadowS"]
        field, new, _, _=centers(flight)
        if arrival is None or arrival>120 or load(shot/"result.json")["collectedMarks"] != list(MARKS[:i+1]):
            raise ValueError(f"{mark} has no qualifying distinct paired event by 120 s")
        if i:
            collection=continuations[i-1]
            if i>1:
                group=load(collection/"manifest.json")
                if (group["shots"] != [r["shot"] for r in records] or
                    group["inputs"] != [load(row[-1]/"result.json")["trace"] for row in histories]):
                    raise ValueError("Joint collection omitted or substituted an earlier NOTE")
            for j, history in enumerate(histories):
                history.append(collection/str(j) if i>1 else collection)
        checks=[]
        for entry, history in zip(records,histories):
            prior, end, _=collection_path(entry,history)
            if (end["field"]!=field["field"] or end["nativeSha256"]!=field["nativeSha256"] or
                    abs(end["endS"]-field["endS"])>1e-8 or entry["shot"]==obs["sourceTrace"]):
                raise ValueError("Collection and incoming flight have unequal identities or clock coverage")
            clear=clear_by(prior,entry["scoreS"],box)
            if clear >= field["releaseTimeS"]:
                raise ValueError(f"{entry['piece']} still occupies the counter at the next release")
            gap=separate_paths(prior,new,new[0,0],new[-1,0])
            checks.append(dict(piece=entry["piece"],clearByS=clear,minimumSeparationM=gap,
                               collectionSegments=[load(folder/"result.json")["trace"] for folder in history],
                               checkedThroughS=new[-1,0]))
        histories.append([flight])
        records.append(dict(piece=load(pickup/"result.json")["pieceId"],native=obs["sourceNativeSha256"],
                            shot=obs["sourceTrace"],field=obs["fieldTrace"],releaseS=obs["releaseS"],
                            openingS=obs["opening"]["fullyThroughByS"],pairTimesS=obs["firstFullShadowsS"],
                            scoreS=arrival,priorOccupancy=checks))
        last_shot=shot
    return dict(schema="swerve-shared-counter-sequence/1",sensorSha256=digest(sensor),formalAttempts=0,
                coupledCollections=[dict(sha256=digest(c/"manifest.json"),**load(c/"manifest.json")) for c in continuations[1:]],
                records=records)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("route","sensor","output"):
        parser.add_argument(name,type=Path)
    parser.add_argument("continuations",type=Path,nargs="+")
    a=parser.parse_args()
    result=verify(a.route,a.sensor,a.continuations)
    with a.output.open("x",encoding="utf8",newline="\n") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write("\n")
