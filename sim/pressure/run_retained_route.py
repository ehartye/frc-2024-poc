"""Run the native route from a verified, acquired stop using live feedback."""
import argparse
import json
import math
import pathlib

from run_axial_shear_lift import ROOT, EVIDENCE, digest, load
from retained_route import RouteLift, controls
from route_supply import step_route
from native_pressure_lift import LiftDomainError
from signed_pressure_acquisition import DomainExit


def run(archive, ground, stop, output, dt, grid):
    binding_path = EVIDENCE / "native-signed-20261118/native-pressure-binding.json"
    binding = load(binding_path)
    if digest(archive) != binding["nativeArchiveSha256"]:
        raise ValueError("Wrong native Fusion revision")
    acquisition, source = load(ground), load(stop)
    start = acquisition["initial"]
    if (start["timeS"] != 0 or any(start["ledger"].values()) or start["initialEnergyJ"] != 0
            or start["note"]["p"] != [.658,0.] or any(start["note"]["v"])
            or acquisition["index"] is None or acquisition["transportEnd"] != source["initial"]
            or source["sourceTrajectorySha256"] != digest(ground)
            or acquisition["inputs"][binding_path.name] != digest(binding_path)
            or source["bindingSha256"] != digest(binding_path)
            or source["outcome"] != "ACTIVE_STOP_HORIZON" or not source["settledLast100ms"]):
        raise ValueError("Missing continuous floor-acquisition and transport evidence")
    contract = load(EVIDENCE / "native-lift-20261118/native-pressure-lift-contract.json")
    contract["resolution"] = [grid,grid]
    model = RouteLift(source["final"], load(ROOT/"sim/drivetrain/config.json"),
                      load(ROOT/"sim/acquisition/free-planar-config.json")["model"],
                      load(ROOT/"cad/assembly/intake_components.json"),binding,
                      load(ROOT/"sim/actuation/config.json"),contract,shear_stiffness=2500.)
    anchor = source["trace"][-1]["energyJ"]
    join = model.start_mechanical-anchor["kinetic"]-anchor["spring"]
    if abs(join) > (1e-12 if grid==1024 else .002):
        raise ValueError("Lift join energy mismatch")
    initial = model.s["timeS"]
    bus = source["trace"][-1]["busV"]
    ticks = round(.01/dt)
    output.mkdir(parents=True,exist_ok=False)
    status = "HORIZON"
    inputs = {key:digest(path) for key,path in dict(archive=archive,ground=ground,stop=stop,
            route=ROOT/"sim/pressure/retained_route.py",shear=ROOT/"sim/pressure/axial_shear_lift.py",
            supply=ROOT/"sim/pressure/route_supply.py",runner=pathlib.Path(__file__)).items()}
    with (output/"trace.jsonl").open("x",encoding="utf-8") as trace:
        for i in range(580*ticks):
            try:
                if i%ticks == 0:
                    command = controls(model,i//ticks,bus)
                row = step_route(model,dt,command)
            except (LiftDomainError,DomainExit) as error:
                status = str(error)
                break
            if abs(row["energyJ"]["residual"]-anchor["residual"]) > .002:
                status = "ENERGY_RESIDUAL"
            state = row["state"]
            row["minimumFloorClearanceM"] = (state["noteZ"]-model.z0
                    -model.cad["note"]["majorRadius"]/1000*abs(math.sin(state["notePitch"])))
            row["tick"] = i//ticks
            if i%ticks == 0:
                row["control"] = command
            trace.write(json.dumps(row,separators=(",",":"),allow_nan=False)+"\n")
            bus = row["busV"]
            if (i+1)%(ticks*20)==0:
                trace.flush()
                print(f"{state['timeS']-initial:.2f} s, pivot {state['zPivot']:.4f} m, "
                      f"pitch {math.degrees(state['theta']):.2f} deg",flush=True)
            if status != "HORIZON":
                break
    result=dict(outcome=status,dt=dt,grid=grid,startTimeS=initial,endTimeS=model.s["timeS"],
                initialMechanicalDifferenceJ=join,inputs=inputs)
    (output/"result.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result),flush=True)
    return result


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("archive","ground","stop","output"):
        parser.add_argument("--"+name,type=pathlib.Path,required=True)
    parser.add_argument("--dt",type=float,choices=(.0005,.00025,.000125),default=.0005)
    parser.add_argument("--grid",type=int,choices=(512,1024,2048),default=1024)
    run(**vars(parser.parse_args()))
