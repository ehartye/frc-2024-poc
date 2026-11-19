"""Run a varied floor start on the native-bound pickup plant."""

import argparse
import json
import math
from pathlib import Path

from run_axial_shear_lift import EVIDENCE, ROOT, digest, load
from second_wing_pickup import ReturnPlant, advance_pickup
from speaker_trial_inputs import COHORT, FRAME, local_offset, read_attempt
from wing_field_frame import piece_id


def run(args):
    settings = read_attempt(args.roster, args.attempt)
    binding = load(EVIDENCE / "native-signed-20261118/native-pressure-binding.json")
    native = digest(args.archive)
    if native != binding["nativeArchiveSha256"]:
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
    plant = ReturnPlant(start, state, args.dt, args.grid, 32,
                        calibration["runs"]["grid-16384"]["parameters"])
    plant.begin_pickup(starts_lowered=True)
    plant.capture_speed = .16
    plant.contact_steering = args.steering == "load"
    plant.encoder_steering = args.steering == "encoder"
    plant.mark_xy = nominal
    initial_q = plant.data.qpos.tolist()
    initial_v = plant.data.qvel.tolist()
    result = advance_pickup(args, plant)
    result.update(schema="varied-first-wing-acquisition/1", attempt=args.attempt,
                  rosterSha256=digest(args.roster), nativeSha256=native,
                  pieceId=piece_id("middle"), collectedMarks=["middle"], firstMark="middle",
                  fieldYawDeg=245., start=0., initialQ=initial_q, initialV=initial_v,
                  initialRobotQ=initial_q[:plant.nq], initialRobotV=initial_v[:plant.nv],
                  initialRotors=state["rotor"], initialBusV=state["busV"],
                  waitingNoteLocalXY=note_xy.tolist(), targetLocalXY=nominal.tolist(),
                  contactSteering=plant.contact_steering, encoderSteering=plant.encoder_steering, captureSpeedMps=plant.capture_speed,
                  traceStride=args.trace_stride)
    (args.output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({k:result[k] for k in ("attempt", "outcome", "end", "firstBilateral", "trace")}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--roster", type=Path, default=COHORT)
    parser.add_argument("--attempt", required=True)
    parser.add_argument("--steering", choices=("none", "load", "encoder"), default="none")
    parser.add_argument("--dt", type=float, choices=(.00025, .0005), default=.0005)
    parser.add_argument("--grid", type=int, choices=(256, 512, 1024), default=512)
    parser.set_defaults(horizon=12., trace_stride=1)
    run(parser.parse_args())
