"""Bind the historical D1 pickup film to its detector and mission evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
LAB = Path("experiments/speaker-cycles")
MEDIA = Path("docs/media")


def read(path):
    return json.loads((ROOT/path).read_bytes())


def ref(path):
    return dict(file=path.as_posix(), sha256=hashlib.sha256((ROOT/path).read_bytes()).hexdigest())


def save(path, content):
    (ROOT/path).write_text(json.dumps(content, indent=2, allow_nan=False)+"\n", encoding="utf8", newline="\n")
    return ref(path)


def register(checker):
    display = MEDIA/"swerve-r7.json"
    source = Path("sim/pressure/evidence/maxswerve-development-shot.json")
    mission = Path("sim/pressure/evidence/maxswerve-six-note-D1.json")
    rec = read(display)
    shot = next(r for r in read(source)["pairedDenseObservations"] if r["attempt"]=="D1")
    first = read(mission)["records"][0]
    if (rec["plantSha256"] != shot["sourceNativeSha256"] or first["fieldTrace"] != shot["fieldTrace"]
            or first["releaseTrace"] != shot["sourceTrace"] or first["carryTrace"] != shot["carryTrace"]
            or first["piece"] != "blue-wing-middle" or first["pairedSensorS"] != rec["simultaneousSensorS"]):
        raise ValueError("D1 film, release and first paired sensor event have different sources")
    film = rec["film"]
    media = ref(Path(rec["assets"]["film"]["path"]))
    if media["sha256"] != rec["assets"]["film"]["sha256"]:
        raise ValueError("First-shot film bytes have changed")
    start, slow, opening, sensor = rec["sourceStartS"], film["slowFromMatchS"], shot["opening"]["fullyThroughByS"], shot["firstSimultaneousFullShadowS"]
    if opening != film["slowUntilMatchS"] or sensor != film["endMatchS"]:
        raise ValueError("Render times differ from field observations")
    slow_end = slow-start+(opening-slow)/film["slowRate"]
    identity = dict(revision=rec["plantSha256"], attempt="D1", piece=first["piece"], clock="match")
    trace = save(LAB/"first-film-states.json", dict(sources=[ref(display),ref(mission)],
                 rows=[dict(**identity,timeS=t) for t in (start,sensor)]))
    events = [
        dict(type="seat",timeS=first["seatedS"],kind="target",target="Pickup",criterion="The middle-wing NOTE seats between the belts."),
        dict(type="release",timeS=shot["releaseS"],kind="target",target="Release",criterion="The NOTE leaves the powered wheels and belts."),
        dict(type="opening",timeS=opening,kind="target",target="Opening",criterion="The full ring clears the registered opening."),
        dict(type="simultaneous-counter",timeS=sensor,kind="score",target="Counter",criterion="The same NOTE shadows both sensor pairs.",
             rule="One simulated paired event after full opening passage."),
    ]
    obs = save(LAB/"first-film-events.json", dict(sources=[ref(source),ref(mission)], events=[
        dict(**identity, **e, qualified=True) for e in events]))
    chapter = dict(id="first-note",label="First middle-wing NOTE",start="/rows/0",end="/rows/1",
                   sync=[dict(runS=t,clipS=c) for t,c in ((start,0),(slow,slow-start),(opening,slow_end),(sensor,slow_end+sensor-opening))],
                   events=[dict(id=e["type"],type=e["type"],timeS=e["timeS"],pointer=f"/events/{i}",
                                outcome="scored" if e["kind"]=="score" else "reached") for i,e in enumerate(events)])
    path = LAB/"first-film-chapters.json"
    save(path,dict(schema="agent-frc/replay-chapters@1",revision=rec["displayRevision"],media=media,trace=trace,
                   observations=obs,chapters=[chapter]))
    verified = subprocess.check_output(["node", str(checker.resolve(strict=True)), str(ROOT/path), str(ROOT)])
    save(MEDIA/"first-film-index.json", json.loads(verified))


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checker",type=Path,help="Agent FRC 0.33.1 scripts/replay-chapters.mjs")
    register(parser.parse_args().checker)
