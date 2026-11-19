"""Estimate the net support force on the moving assembly during an airborne carry."""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from maxswerve import SwervePickup
from run_axial_shear_lift import ROOT, load
from verify_swerve_carry import verify


def measure(pickup, carry):
    receipt = verify(pickup,carry)
    source = load(pickup/"result.json")
    plant = SwervePickup.from_floor_receipt(source,receipt["dt"],receipt["grid"],32)
    model,data = plant.model,plant.data
    groups = [model.body(name).id for name in ("carriage","note")]
    masses = model.body_subtreemass[groups]
    mass = float(sum(masses))
    weight = -mass*model.opt.gravity
    geometry = load(ROOT/"cad/assembly/intake_components.json")["note"]
    previous = None
    peak_horizontal = peak_vertical = 0.
    count = 0
    with (carry/"trace.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            data.qpos[:] = row["q"]
            data.qvel[:] = row["v"]
            mujoco.mj_fwdPosition(model,data)
            mujoco.mj_fwdVelocity(model,data)
            mujoco.mj_subtreeVel(model,data)
            # The free NOTE is outside the carriage subtree. Its belt and wheel
            # forces cancel internally only when both groups enter the balance.
            momentum = masses @ data.subtree_linvel[groups]
            normal_z = data.xmat[groups[1],8]
            half_height = (geometry["majorRadius"]*math.sqrt(max(0.,1-normal_z**2))
                           +geometry["tubeRadius"])/1000
            if data.xpos[groups[1],2]-half_height <= .01:
                previous = None
                continue
            if previous is not None:
                force = (momentum-previous)/receipt["dt"] + weight
                peak_vertical = max(peak_vertical, abs(float(force[2])))
                peak_horizontal = max(peak_horizontal,float(np.linalg.norm(force[:2])))
                count += 1
            previous = momentum
    if count==0 or peak_vertical<=0 or not math.isfinite(peak_vertical+peak_horizontal):
        raise ValueError("No finite airborne load interval")
    return dict(attempt=receipt["attempt"],dt=receipt["dt"],carryTrace=receipt["carryTrace"],
                airborneIntervals=count,movingMassWithNoteKg=mass,weightN=float(weight[2]),
                peakAbsoluteVerticalN=peak_vertical,peakHorizontalN=peak_horizontal)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pickup",type=Path)
    parser.add_argument("carry",type=Path)
    args = parser.parse_args()
    print(json.dumps(measure(args.pickup,args.carry),indent=2,allow_nan=False))
