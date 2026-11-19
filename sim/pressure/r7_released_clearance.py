"""Check a released NOTE against the following robot and installed field stock."""
import argparse
import json
from pathlib import Path

from released_note_clearance import flight_environment, separation, synchronized_path
from r7_route_clearance import FRAME, DEVELOPMENT_STARTS, inventory, registered_stock
from r7_dynamics import verify_binding
from run_axial_shear_lift import digest, load


def screen(flight, robot, field, survey, manifest):
    state = load(robot/"result.json")
    if not verify_binding(state):
        raise ValueError("Expected the R7 plant")
    installed = load(field)
    if installed["source"] != "42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a":
        raise ValueError("Wrong installed field")
    installed["parts"], _ = registered_stock(installed["parts"])
    result = separation(synchronized_path(flight, robot, FRAME), inventory(survey, manifest))
    return dict(**result, environment=flight_environment(flight, robot, FRAME, installed, DEVELOPMENT_STARTS),
                field=digest(flight/"trace.jsonl"), robot=digest(robot/"trace.jsonl"), installedField=digest(field))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("flight", "robot", "field", "survey", "manifest"):
        p.add_argument(key, type=Path)
    args = p.parse_args()
    print(json.dumps(screen(**vars(args)), indent=2, allow_nan=False))
