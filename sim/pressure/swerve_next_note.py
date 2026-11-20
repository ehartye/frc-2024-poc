"""Drive the released swerve robot to a distinct floor NOTE, then return with it."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from field_transit import TransitPath
from maxswerve import LAYOUT, SwervePickup
from mission_runtime import runtime_versions, verify_runtime_chain
from run_axial_shear_lift import EVIDENCE, ROOT, digest, load
from second_wing_pickup import ReturnPlant, advance_pickup, angle_error
from speaker_trial_inputs import FRAME, MARKS, local_offset, read_attempt
from varied_floor_start import carry_to_speaker
from wing_field_frame import piece_id


class SwerveReturn(SwervePickup):
    def controls(self, belt, launcher_duty, feed_enabled):
        if self.approach_complete or hasattr(self,"seated"):
            return super().controls(belt, launcher_duty, feed_enabled)
        _, actuator, pinch, launcher = ReturnPlant.controls(self,belt,launcher_duty,feed_enabled)
        d = self.data
        delta = self.staging_xy - d.qpos[:2]
        gap = np.linalg.norm(delta)
        if self.pickup_path is not None:
            waypoint, remaining = self.pickup_path.target(d.qpos[:2])
            delta = waypoint - d.qpos[:2]
            gap = remaining
        target_heading = self.pickup_heading
        if self.wall_approach and not self.wall_departed:
            local_forward = FRAME.rotation[:2,:2].T[:,0]
            position = FRAME.to_field([*d.qpos[:2],0])
            self.wall_departed = position[0] >= 1.65
            if not self.wall_departed:
                delta = local_forward*(1.78-position[0])
                target_heading = np.deg2rad(180.-FRAME.yaw_degrees)
        target = delta * min(2.3, self.transit_speed/max(np.linalg.norm(delta),1e-9))
        rotation = d.xmat[self.chassis].reshape(3,3)
        forward, side = rotation[:2,:2].T @ target
        angle = angle_error(target_heading,d.qpos[2])
        if gap < .04 and abs(angle) < .035 and d.qpos[3] < .003 and abs(d.qpos[4]) < .015:
            self.approach_complete = True
        turn = np.clip(2.5*angle,-.95,.95)
        return self.module_duties(forward,side,turn), actuator, pinch, launcher


def run(args):
    if not math.isfinite(args.pickup_lookahead) or args.pickup_lookahead <= 0:
        raise ValueError("Pickup lookahead must be positive and finite")
    if args.pickup_speed is not None and (not math.isfinite(args.pickup_speed) or args.pickup_speed <= 0):
        raise ValueError("Pickup transit speed must be positive and finite")
    if not math.isfinite(args.carry_speed) or args.carry_speed <= 0:
        raise ValueError("Carry speed must be positive and finite")
    source = load(args.release/"result.json")
    floor = load(args.pickup/"result.json")
    from r7_dynamics import verify_binding
    mass_binding = verify_binding(source)
    if verify_binding(floor) != mass_binding:
        raise ValueError("R7 mass case changed during the mission")
    verify_runtime_chain([source, floor])
    if (source["firstMark"] != FRAME.first_mark or source["fieldYawDeg"] != FRAME.yaw_degrees or
            source.get("drivetrain") != "maxswerve" or source["outcome"] != "UPWARD_FREE_RELEASE" or
            digest(args.archive) != source["nativeSha256"] or source["rosterSha256"] != digest(args.roster) or
            source["traceSha256"] != digest(args.release/"trace.jsonl") or
            any(floor[k] != source[k] for k in ("firstMark","fieldYawDeg","attempt","rosterSha256","nativeSha256")) or
            floor["layoutSha256"] != digest(LAYOUT) or floor["outcome"] != "SEATED" or
            floor["trace"] != digest(args.pickup/"trace.jsonl") or
            floor["collectedMarks"] != source["collectedMarks"] or floor["pieceId"] != source["pieceId"]):
        raise ValueError("Expected the same swerve robot, piece and field setup at release")
    collected = source["collectedMarks"]
    if args.next_mark in collected or len(set(collected)) != len(collected):
        raise ValueError("A floor NOTE may only be collected once")
    with (args.release/"trace.jsonl").open() as stream:
        for line in stream:
            last_line = line
    last = json.loads(last_line)
    if last["time"] != source["end"] or len(last["q"]) != 24 or len(last["v"]) != 23:
        raise ValueError("Incomplete released swerve state")
    plant = SwerveReturn.from_floor_receipt(floor,args.dt,args.grid,32)
    d = plant.data
    mark = FRAME.to_local(FRAME.mark(args.next_mark))[:2]
    setup = read_attempt(args.roster,source["attempt"])
    physical = mark + local_offset(setup,args.next_mark)
    stiffness = load(EVIDENCE/"native-lift-20261118/native-pressure-lift-contract.json")["floorStiffnessNpm"]
    mass = load(ROOT/"sim/component-launcher/six-drive-config.json")["pieceMassKg"]
    d.qpos[plant.nq:] = [*physical,.0254-mass*9.80665/stiffness,1.,0.,0.,0.]
    d.qvel[plant.nv:] = 0.
    plant.rotors = np.array(last["externalRotors"])
    plant.shear = np.array(last["shearM"])
    plant.bus = last["busV"]
    d.time = last["time"]
    plant.resume(last)
    plant.angle_target = plant.mount + d.qpos[plant.az_pos]
    plant.mark_xy = mark
    route = next((cycle for cycle in load(EVIDENCE/"centerline-route-controls.json")["cycles"]
                  if cycle["piece"] == args.next_mark), None)
    via = route["pickup"]["viaFieldXY"] if route else []
    waypoints = [FRAME.to_local([x,y,0])[:2] for x,y in via]
    direction = mark - (waypoints[-1] if waypoints else d.qpos[:2])
    if source.get("routePolicy") == "straight-wall-approach/1" and args.next_mark == "inner":
        # Face upfield at the wing mark, directly toward the narrow Stage face.
        # A diagonal pickup swings the outboard corner through the upright.
        direction = FRAME.rotation[:2,:2].T[:,0]
    plant.pickup_heading = math.atan2(direction[1],direction[0])
    plant.staging_xy = mark - .85*direction/np.linalg.norm(direction)
    plant.pickup_path = (TransitPath([d.qpos[:2].copy(), *waypoints, plant.staging_xy], args.pickup_lookahead)
                         if waypoints else None)
    plant.wall_approach = source.get("routePolicy") == "straight-wall-approach/1"
    plant.wall_departed = False
    plant.approach_complete = False
    plant.capture_speed = .26
    # Recover time spent backing clear of the Stage and the Speaker before turning.
    plant.transit_speed = args.pickup_speed or (1.30 if plant.wall_approach and args.next_mark in ("center-mid", "center-low") else 1.1)
    result = dict(**mass_binding, schema="maxswerve-next-wing-acquisition/1",drivetrain="maxswerve",runtime=runtime_versions(),
                  layoutSha256=digest(LAYOUT),nativeSha256=source["nativeSha256"],firstReleaseTrace=source["traceSha256"],
                  pieceId=piece_id(args.next_mark),attempt=source["attempt"],rosterSha256=source["rosterSha256"],
                  firstMark=source["firstMark"],fieldYawDeg=source["fieldYawDeg"],start=d.time,
                  targetLocalXY=mark.tolist(),waitingNoteLocalXY=physical.tolist(),initialQ=d.qpos.tolist(),
                  initialV=d.qvel.tolist(),initialRotors=plant.rotors.tolist(),initialShearM=plant.shear.tolist(),
                  initialBusV=plant.bus,initialSourceJ=plant.source_work,traceStride=args.trace_stride)
    if args.pickup_speed is not None:
        result["transitSpeedMps"] = args.pickup_speed
    if plant.wall_approach:
        result["routePolicy"] = source["routePolicy"]
    if route:
        result["viaFieldXY"] = via
        if args.pickup_lookahead != .48:
            result["pickupLookaheadM"] = args.pickup_lookahead
    result.update(advance_pickup(args,plant))
    result["collectedMarks"] = collected+[args.next_mark] if result["outcome"] == "SEATED" else collected
    (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(result["outcome"],result["end"],flush=True)
    if args.carry_output and result["outcome"] == "SEATED":
        carry_to_speaker(args,plant,result,route=route)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("release","pickup","archive","roster","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--next-mark",choices=MARKS,required=True)
    parser.add_argument("--carry-output",type=Path)
    parser.add_argument("--pickup-lookahead", type=float, default=.48, help="Transit path lookahead in m")
    parser.add_argument("--pickup-speed", type=float, help="Unloaded transit speed in m/s")
    parser.add_argument("--carry-speed", type=float, default=.85, help="Loaded transit speed in m/s")
    parser.add_argument("--dt",type=float,choices=(.00025,.0005),default=.0005)
    parser.add_argument("--grid",type=int,choices=(256,512,1024),default=512)
    parser.set_defaults(horizon=18.,trace_stride=1)
    run(parser.parse_args())
