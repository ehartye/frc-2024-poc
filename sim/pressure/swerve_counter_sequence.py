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


def verify(route, sensor, continuations, field_segments=None, stop_at_last_score=False):
    field_segments = field_segments or {}
    if not set(field_segments) <= set(MARKS[:len(continuations)+1]):
        raise ValueError("Field segment names must belong to the verified route")
    if not 1 <= len(continuations) < len(MARKS):
        raise ValueError("Supply one collection continuation for each subsequent release")
    faces=np.array([p for row in load(sensor)["sensorCaseEnds"] for p in row["caseEndCornersM"]])
    box=faces.min(axis=0),faces.max(axis=0)
    histories=[]
    records=[]
    last_shot=None
    for i, mark in enumerate(MARKS[:len(continuations)+1]):
        pickup, carry, shot, flight=[route/(mark+"-"+step) for step in ("pickup","carry","shot","field")]
        flight = field_segments.get(mark, flight)
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
        # There is no subsequent handoff after the last required event. Extra
        # integration of either path has no effect on their earlier states.
        terminal_event = stop_at_last_score and i == len(continuations)
        through = arrival if terminal_event else new[-1,0]
        checks=[]
        for entry, history in zip(records,histories):
            prior, end, _=collection_path(entry,history)
            if (end["field"]!=field["field"] or end["nativeSha256"]!=field["nativeSha256"] or
                    (end["endS"]+1e-8 < through if terminal_event else abs(end["endS"]-field["endS"])>1e-8)
                    or entry["shot"]==obs["sourceTrace"]):
                raise ValueError("Collection and incoming flight have unequal identities or clock coverage")
            stop = np.searchsorted(prior[:,0], through, side="left") + 1
            clear=clear_by(prior[:stop],entry["scoreS"],box)
            if clear >= field["releaseTimeS"]:
                raise ValueError(f"{entry['piece']} still occupies the counter at the next release")
            gap=separate_paths(prior,new,new[0,0],through)
            # After the independent field solve, a bounding sphere encloses the
            # whole NOTE at any orientation between successive center samples.
            collected = prior[:stop][prior[:stop,0] >= load(history[0]/"result.json")["endS"]-1e-8]
            eastern_extent = float(collected[:,1].max()+MAJOR+TUBE+3e-6)
            checks.append(dict(piece=entry["piece"],clearByS=clear,minimumSeparationM=gap,
                               collectionEasternExtentM=eastern_extent,
                               collectionSegments=[load(folder/"result.json")["trace"] for folder in history],
                               checkedThroughS=through))
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
    parser.add_argument("--field-segment", action="append", nargs=2, default=[], metavar=("MARK", "DIRECTORY"),
                        help="Use a dense field solve ending before the first possible contact with older NOTES")
    parser.add_argument("--stop-at-last-score", action="store_true",
                        help="Verify concurrent occupancy through the last required paired event")
    a=parser.parse_args()
    segments = {mark: Path(folder) for mark, folder in a.field_segment}
    if len(segments) != len(a.field_segment):
        parser.error("Specify a single field segment per mark")
    result=verify(a.route,a.sensor,a.continuations,segments,a.stop_at_last_score)
    with a.output.open("x",encoding="utf8",newline="\n") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write("\n")
