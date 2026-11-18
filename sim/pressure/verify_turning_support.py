"""Replay 0.1 s of a turning hold with the recorded controls and optional tread support."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from turning_plant import TurningPlant
from turning_contact import floor_wrench
from verify_turning_lift import digest


def floor_only(plant, q, v):
    data, robot = plant.data, plant.robot
    data.qpos[:] = q
    data.qvel[:] = v
    mujoco.mj_fwdPosition(robot, data)
    mujoco.mj_fwdVelocity(robot, data)
    rotation = data.xmat[plant.body["note"]].reshape(3, 3)
    floor, moment, potential, heat = floor_wrench(
        q[5:8], rotation, v[5:8], rotation @ v[8:11], plant.ring, plant.tube,
        plant.contract, plant.nc["groundMu"], plant.nc["slipRegularizationMps"])
    return dict(force=np.zeros(3), cradleMoment=np.zeros(3), noteMoment=np.zeros(3),
                rotor=np.zeros(2), shearRate=np.zeros(4), normal=np.zeros(2), potential=0., heat=0.,
                floor=floor, floorMoment=moment, floorPotential=potential, floorHeat=heat)


def compare_support(run, stop):
    info = json.loads((run / "result.json").read_bytes())
    if info["inputs"]["stop"] != digest(stop) or info["outcome"] != "HORIZON":
        raise ValueError("The achieved maneuver and source stop must match")
    seed = json.loads(stop.read_bytes())["final"]
    rows = []
    with (run / "trace.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            if row["t"] >= 7 - 1e-9:
                rows.append(row)
            if row["t"] >= 7.1 - 1e-9:
                break
    if len(rows) != round(.1/info["dt"]) + 1 or "command" not in rows[1]:
        raise ValueError("The hold must contain the complete command window")

    results = {}
    for mode in ("intact", "removed"):
        plant = TurningPlant(seed, info["grid"])
        start = rows[0]
        plant.q, plant.v = np.array(start["q"]), np.array(start["v"])
        plant.rotors, plant.shear = np.array(start["rotor"]), np.array(start["shear"])
        plant.bus = start["busV"]
        if mode == "removed":
            plant.contact = lambda q, v, rotors, shear: floor_only(plant, q, v)
        contact = plant.contact(plant.q, plant.v, plant.rotors, plant.shear)
        lowest_normal = float("inf")
        for row in rows[1:]:
            if "command" in row:
                command = row["command"]
            contact, _ = plant.advance(info["dt"], command, contact)
            lowest_normal = min(lowest_normal, min(contact["normal"]))
        start_height = start["q"][7]
        if mode == "intact" and np.max(abs(plant.q - rows[-1]["q"])) > 1e-10:
            raise ValueError("Intact replay diverged")
        free_height = start_height + start["v"][7]*.1 - plant.ac["gravityMps2"]*.1**2/2
        if mode == "removed" and abs(plant.q[7] - free_height) > 1e-10:
            raise ValueError("Unsupported NOTE did not enter free fall")
        results[mode] = dict(changeInNoteHeightM=float(plant.q[7]-start_height), minimumSideNormalN=float(lowest_normal),
                             finalHeightM=float(plant.q[7]), finalVelocityMps=float(plant.v[7]))
    results.update(startS=rows[0]["clock"], durationS=.1, sourceTraceSha256=digest(run/"trace.jsonl"))
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("stop", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare_support(args.run, args.stop), indent=2))
