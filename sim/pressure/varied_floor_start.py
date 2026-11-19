"""Run a varied floor start on the native-bound pickup plant."""

import argparse
import json
import math
from pathlib import Path
from mission_runtime import runtime_versions, verify_runtime_chain

from run_axial_shear_lift import EVIDENCE, ROOT, digest, load
from second_wing_pickup import ReturnPlant, advance_pickup
from speaker_trial_inputs import COHORT, FRAME, local_offset, read_attempt
from wing_field_frame import piece_id


def run(args):
    settings = read_attempt(args.roster, args.attempt)
    roster_digest = digest(args.roster)
    runtime = runtime_versions()
    if roster_digest == digest(COHORT):
        verify_runtime_chain([dict(runtime=runtime)])
    binding = load(EVIDENCE / "native-signed-20261118/native-pressure-binding.json")
    native = digest(args.archive)
    if args.carry_output and args.drivetrain != "maxswerve":
        raise ValueError("The continuous holonomic carry requires MAXSwerve")
    mass_case = getattr(args, "r7_mass_case", None)
    if mass_case and args.drivetrain != "maxswerve":
        raise ValueError("R7 mass cases require the four-module plant")
    plant_type = ReturnPlant
    expected_native = binding["nativeArchiveSha256"]
    if args.drivetrain == "maxswerve":
        from maxswerve import LAYOUT, SwervePickup
        from r7_dynamics import BUDGET, binding
        expected_native = load(BUDGET)["archive"] if mass_case else load(LAYOUT)["privateFusionArchiveSha256"]
        plant_type = SwervePickup
        if args.steering != "none":
            raise ValueError("The load and encoder steering variants are differential-drive controllers")
    if native != expected_native:
        raise ValueError("Unexpected native Fusion archive")
    xy = local_offset(settings, "robot")
    yaw = math.radians(settings["robot_heading_deg"])
    nominal = FRAME.to_local(FRAME.mark("middle"))[:2]
    note_xy = nominal + local_offset(settings, "middle")
    floor = load(EVIDENCE / "native-lift-20261118/native-pressure-lift-contract.json")
    mass = load(ROOT / "sim/component-launcher/six-drive-config.json")["pieceMassKg"]
    z = .0254 - mass*9.80665/floor["floorStiffnessNpm"]
    chassis = dict(x=xy[0], y=xy[1], heading=yaw, vx=0., vy=0., yaw_rate=0., wheel=[0., 0.])
    note = dict(p=note_xy.tolist(), v=[0., 0.], yaw=0., spin=0., w=[0., 0.])
    start = dict(timeS=0., chassis=chassis, note=note)
    state = dict(q=[*xy, yaw, 0., 0., *note_xy, z, 1., 0., 0., 0.],
                 v=[0.]*11, rotor=[0.]*4, shear=[0.]*4, busV=12., clock=0.)
    calibration = load(ROOT / "sim/mujoco/evidence/launcher-quadrature-verification-20261118.json")
    plant = plant_type(start, state, args.dt, args.grid, 32,
                        calibration["runs"]["grid-16384"]["parameters"], **({"mass_case": mass_case} if mass_case else {}))
    plant.begin_pickup(starts_lowered=True)
    plant.wall_approach = getattr(args, "wall_approach", False)
    plant.capture_speed = .16
    plant.contact_steering = args.steering == "load"
    plant.encoder_steering = args.steering == "encoder"
    plant.mark_xy = nominal
    initial_q = plant.data.qpos.tolist()
    initial_v = plant.data.qvel.tolist()
    result = advance_pickup(args, plant)
    result.update(schema="varied-first-wing-acquisition/1", attempt=args.attempt, runtime=runtime,
                  rosterSha256=roster_digest, nativeSha256=native,
                  pieceId=piece_id("middle"), collectedMarks=["middle"], firstMark="middle",
                  fieldYawDeg=245., start=0., initialQ=initial_q, initialV=initial_v,
                  initialRobotQ=initial_q[:plant.nq], initialRobotV=initial_v[:plant.nv],
                  initialRotors=state["rotor"], initialBusV=state["busV"],
                  waitingNoteLocalXY=note_xy.tolist(), targetLocalXY=nominal.tolist(),
                  contactSteering=plant.contact_steering, encoderSteering=plant.encoder_steering, captureSpeedMps=plant.capture_speed,
                  traceStride=args.trace_stride)
    if args.drivetrain == "maxswerve":
        result.update(**binding(mass_case), schema="maxswerve-first-wing-acquisition/1", layoutSha256=digest(LAYOUT),
                      collectedMarks=["middle"] if result["outcome"] == "SEATED" else [],
                      drivetrain=args.drivetrain, contactSteering=True, steeringObservation="ideal belt normal load difference",
                      moduleOrder=["FL", "FR", "RL", "RR"],
                      peakDriveAndSteerCurrentA=plant.peak_current.tolist(), peakSlipMps=plant.peak_slip,
                      azimuthQpos=plant.az_pos.tolist(),
                      wheelRollQpos=[int(plant.model.joint("roll_"+tag).qposadr[0]) for tag in ("fl","fr","rl","rr")])
    if plant.wall_approach:
        result["routePolicy"] = "straight-wall-approach/1"
    (args.output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({k:result[k] for k in ("attempt", "outcome", "end", "firstBilateral", "trace")}))

    if args.carry_output and result["outcome"] == "SEATED":
        carry_to_speaker(args, plant, result)


def carry_to_speaker(args, plant, result, *, route=None):
    from types import SimpleNamespace
    from wing_note_delivery import deliver, tail
    carry = SimpleNamespace(output=args.carry_output, first_mark="middle", field_yaw_deg=245.,
                            field_station=[1.475, 5.553075, 180.], transit_speed=.85,
                            turn_rate=.90, turn_accel=.7, lifting_drive_accel=.40,
                            via=[], final_straight=0., lookahead=0., dt=args.dt,
                            horizon=18., grid=args.grid, trace_stride=args.trace_stride)
    if route:
        carry.via = route["delivery"]["viaFieldXY"]
        carry.lookahead = .48
    plant.shoot_position = FRAME.to_local([*carry.field_station[:2], 0.])[:2]
    plant.shoot_heading = math.radians(carry.field_station[2]-carry.field_yaw_deg)
    if getattr(plant, "wall_approach", False) and result["pieceId"] == "blue-wing-inner":
        plant.stage_exit = FRAME.to_local([2.40, FRAME.mark("inner")[1], 0.])[:2]
    plant.field_velocity = plant.data.qvel[:2].copy()
    plant.attempt_setup = (result["attempt"], result["rosterSha256"])
    deliver(carry, plant, tail(args.output, result), result["trace"], result["nativeSha256"], result["collectedMarks"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--carry-output", type=Path, help="Continue the seated swerve robot to the first Speaker station")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--roster", type=Path, default=COHORT)
    parser.add_argument("--attempt", required=True)
    parser.add_argument("--drivetrain", choices=("differential", "maxswerve"), default="differential")
    parser.add_argument("--wall-approach", action="store_true")
    parser.add_argument("--r7-mass-case", choices=("light", "nominal", "heavy"))
    parser.add_argument("--steering", choices=("none", "load", "encoder"), default="none")
    parser.add_argument("--dt", type=float, choices=(.00025, .0005), default=.0005)
    parser.add_argument("--grid", type=int, choices=(256, 512, 1024), default=512)
    parser.set_defaults(horizon=12., trace_stride=1)
    run(parser.parse_args())
