"""Follow six distinct NOTES in a continuous swerve development run."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from speaker_trial_inputs import MARKS


HERE = Path(__file__).resolve().parent
DEVELOPMENT_STARTS = HERE / "evidence/speaker-pickup-development-starts.csv"


def run_stage(name, *arguments):
    subprocess.run([sys.executable, "-B", str(HERE / (name + ".py")), *map(str, arguments)], check=True)


def receipt(folder):
    return json.loads((folder / "result.json").read_bytes())


def run(archive, field, attempt, output, mass_case=None, outer_feed=None, wall_approach=False, final_carry_speed=.85, final_pickup_speed=None, final_lookahead=.48):
    """Use the nominal first-shot settings on every NOTE, including after a missed score."""
    settings = json.loads((HERE / "evidence/maxswerve-development-shot.json").read_bytes())["controls"]
    command = settings["commonCommand"]
    archive, field = archive.resolve(strict=True), field.resolve(strict=True)
    output.mkdir(parents=True, exist_ok=False)
    records = []
    previous_release = previous_pickup = None
    from swerve_shot_observation import measure

    for mark in MARKS:
        pickup, carry, release, flight = (output / (mark + "-" + k) for k in ("pickup", "carry", "shot", "field"))
        if previous_release is None:
            run_stage("varied_floor_start", "--archive", archive, "--output", pickup, "--carry-output", carry,
                      "--roster", DEVELOPMENT_STARTS, "--attempt", attempt, "--drivetrain", "maxswerve",
                      *(("--r7-mass-case", mass_case) if mass_case else ()), *(("--wall-approach",) if wall_approach else ()))
        else:
            run_stage("swerve_next_note", "--archive", archive, "--output", pickup, "--carry-output", carry,
                      "--roster", DEVELOPMENT_STARTS, "--release", previous_release, "--pickup", previous_pickup,
                      "--next-mark", mark, *(("--carry-speed", final_carry_speed) if mark == "center-low" else ()),
                      *(("--pickup-speed", final_pickup_speed) if mark == "center-low" and final_pickup_speed is not None else ()),
                      *(("--pickup-lookahead", final_lookahead) if mark == "center-low" else ()))
        acquisition = receipt(pickup)
        row = dict(mark=mark, acquisition=acquisition["outcome"], pickupEndS=acquisition["end"])
        records.append(row)
        if acquisition["outcome"] != "SEATED":
            break
        delivered = receipt(carry)
        row.update(carry=delivered["outcome"], carryEndS=delivered["end"])
        if delivered["outcome"] != "AT_SHOOTING_POSE":
            break
        prior = ["--previous-release", previous_release, "--roster", DEVELOPMENT_STARTS] if previous_release else []
        run_stage("wing_note_release", "--archive", archive, "--pickup", pickup, "--delivery", carry,
                  "--output", release, "--dt", settings["dt"], "--grid", settings["grid"], "--segments", 16384,
                  "--duty", command["launcherDutyLimit"], "--pinch-duty", outer_feed if mark=="outer" and outer_feed is not None else command["pinchDuty"],
                  "--target-rpm", command["targetRpm"], "--pitch-trim-deg", command["pitchTrimDeg"],
                  "--height-trim-mm", command["heightTrimMm"], "--align-seconds", command["rampS"],
                  "--first-mark", "middle", "--field-yaw-deg", 245, *prior)
        shot = receipt(release)
        row.update(release=shot["outcome"], shotEndS=shot["end"])
        if shot["outcome"] != "UPWARD_FREE_RELEASE":
            break
        run_stage("run_speaker_field", release, field, flight, "--dt", settings["fieldDt"],
                  "--nodes", settings["fieldContactNodes"], "--first-mark", "middle", "--field-yaw-deg", 245,
                  "--trace-every-step", "--horizon", 5)
        observation = measure(pickup, carry, release, flight, field / "opening-and-counter.json",
                              previous_release=previous_release, roster=DEVELOPMENT_STARTS if prior else None)
        row["observation"] = observation
        previous_pickup, previous_release = pickup, release
    result = dict(attempt=attempt, records=records, formalAttempts=0,
                  scope="Independent Speaker field responses for each continuous robot release. Counter reset, shared collection and route clearance require separate verification.")
    (output / "route.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf8")
    print(json.dumps([(r["mark"], r.get("release"), r.get("observation", {}).get("firstSimultaneousFullShadowS"))
                      for r in records]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("field", type=Path)
    parser.add_argument("attempt", choices=("D1", "D2"))
    parser.add_argument("output", type=Path)
    parser.add_argument("--r7-mass-case", choices=("light","nominal","heavy"))
    parser.add_argument("--wall-approach", action="store_true")
    parser.add_argument("--outer-feed", type=float, help="Outer wing feed duty for a development trial")
    parser.add_argument("--final-pickup-lookahead", type=float, default=.48, help="Unloaded center-low path lookahead in m")
    parser.add_argument("--final-pickup-speed", type=float, help="Unloaded center-low transit speed in m/s")
    parser.add_argument("--final-carry-speed", type=float, default=.85, help="Loaded center-low transit speed in m/s")
    args = parser.parse_args()
    run(args.archive, args.field, args.attempt, args.output, args.r7_mass_case, args.outer_feed, args.wall_approach, args.final_carry_speed, args.final_pickup_speed, args.final_pickup_lookahead)
