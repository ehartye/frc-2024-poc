"""Drive the acquired floor NOTE through a finite-power translating turn and full lift."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from run_axial_shear_lift import EVIDENCE, digest, load
from turning_plant import TurningPlant


def run(args):
    ground,stop=load(args.ground),load(args.stop)
    binding=load(EVIDENCE/"native-signed-20261118/native-pressure-binding.json")
    if (digest(args.archive)!=binding["nativeArchiveSha256"] or ground["initial"]["timeS"]!=0
            or ground["initial"]["note"]["p"] != [.658,0.] or any(ground["initial"]["ledger"].values())
            or not ground["index"] or ground["transportEnd"]!=stop["initial"] or not stop["settledLast100ms"]
            or stop["sourceTrajectorySha256"]!=digest(args.ground)
            or stop["bindingSha256"]!=digest(EVIDENCE/"native-signed-20261118/native-pressure-binding.json")):
        raise ValueError("Native identity or continuous acquired floor state mismatch")
    model=TurningPlant(stop["final"],args.grid)
    model.bus=stop["trace"][-1]["busV"]
    contact=model.contact(model.q,model.v,model.rotors,model.shear)
    initial_energy=model.mechanical(contact)
    steps_per_tick=round(.01/args.dt)
    lead=150
    if not math.isclose(steps_per_tick*args.dt,.01):raise ValueError("Inexact control clock")
    signed={"left":1,"right":-1,"straight":0,"braked":0}[args.case]
    speed=0. if args.case=="braked" else args.speed
    work=np.zeros(2);outcome="HORIZON";elapsed=0.;start_q=model.q.copy()
    args.output.mkdir(parents=True,exist_ok=False)
    with (args.output/"trace.jsonl").open("x") as f:
        for i in range(round(args.horizon/args.dt)):
            tick=i//steps_per_tick
            try:
                if i%steps_per_tick==0:
                    cmd=model.command(tick,contact,signed,lead,speed,args.yaw)
                contact,increment=model.advance(args.dt,cmd,contact)
            except (ValueError,ArithmeticError) as e:
                outcome=str(e)
                break
            work+=increment; elapsed=(i+1)*args.dt
            q,v=model.q,model.v
            forward=v[0]*math.cos(q[2])+v[1]*math.sin(q[2])
            residual=model.mechanical(contact)-initial_energy-work[0]+work[1]
            row=dict(t=elapsed,clock=elapsed+stop["final"]["timeS"],q=q.tolist(),v=v.tolist(),
                     rotor=model.rotors.tolist(),shear=model.shear.tolist(),forward=forward,
                     normal=contact["normal"].tolist(),gap=contact["gap"],floorN=contact["floor"][2],
                     centerRobot=contact["centerRobot"].tolist(),normalRobot=contact["normalRobot"].tolist(),
                     substrateDepth=contact["depth"],contactPowerResidual=contact["powerResidual"],
                     residualJ=residual,busV=model.bus,sourceJ=work[0],heatJ=work[1])
            if i%steps_per_tick==0:row["command"]=cmd
            f.write(json.dumps(row,allow_nan=False,separators=(",",":"))+"\n")
            if (i+1)%(20*steps_per_tick)==0:
                f.flush();print(f"{elapsed:.2f}s yaw {q[2]:.3f} speed {forward:.3f} lift {q[3]:.3f} gap {contact['gap']:.5f}",flush=True)
            if not np.isfinite(q).all() or abs(residual)>.01:
                outcome="ENERGY_OR_FINITE_STATE";break
    result=dict(case=args.case,dt=args.dt,grid=args.grid,leadS=lead/100,end=elapsed,outcome=outcome,
                requestedSpeed=speed,requestedYaw=signed*args.yaw,clockStart=stop["final"]["timeS"],
                initialQ=start_q.tolist(),inputs={k:digest(p) for k,p in (
                    ("native",args.archive),("stop",args.stop),("ground",args.ground),
                    ("plant",Path(__file__).with_name("turning_plant.py")),("contact",Path(__file__).with_name("turning_contact.py")),
                    ("runner",Path(__file__)))})
    (args.output/"result.json").write_text(json.dumps(result,indent=2)+"\n")
    print(outcome,elapsed,flush=True)
    return outcome


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("archive","stop","ground","output"):p.add_argument("--"+k,type=Path,required=True)
    p.add_argument("--dt",type=float,choices=(.000125,.00025,.0005),default=.0005)
    p.add_argument("--grid",type=int,choices=(256,512,1024,2048),default=512)
    p.add_argument("--case",choices=("braked","straight","left","right"),required=True)
    p.add_argument("--speed",type=float,default=.32);p.add_argument("--yaw",type=float,default=.24)
    p.add_argument("--horizon",type=float,default=7.3)
    raise SystemExit(0 if run(p.parse_args())=="HORIZON" else 1)
