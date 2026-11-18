"""Independently measure possession and simultaneous driving, yaw and lift."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

CONTRACT = json.loads((Path(__file__).parent / "evidence/turning-lift-20261118/contract.json").read_bytes())


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def overlap(t, forward, yaw, travel, heading, case):
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("Non-increasing lift clock")
    sign = {"left": 1, "right": -1, "straight": 0, "braked": 0}[case]
    moving = forward >= CONTRACT["minForwardMps"]
    if sign:
        moving &= yaw * sign >= CONTRACT["minSignedYawRadps"]
    elif case == "straight":
        moving &= np.abs(yaw) < .01
    else:
        moving = (np.abs(forward) < .01) & (np.abs(yaw) < .01)

    # Count net travel over intervals satisfying both endpoint checks.
    rise = float(np.diff(travel)[moving[1:] & moving[:-1]].sum())
    heading_change = math.degrees(heading[-1] - heading[0])
    passed = (moving.all() and rise >= CONTRACT["minConcurrentNetLiftM"]
              and (not sign or sign * heading_change >= CONTRACT["minHeadingChangeDegrees"]))
    return bool(passed), rise


def assess(folder):
    result = json.loads((folder / "result.json").read_bytes())
    dt = result["dt"]
    keys = ("q", "v", "normal", "floorN", "residualJ", "heatJ", "busV", "sourceJ", "t", "clock",
            "contactPowerResidual", "gap", "centerRobot", "normalRobot")
    columns = {key: [] for key in keys}
    with (folder / "trace.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            for key, values in columns.items():
                values.append(row[key])
    data = {key: np.asarray(values) for key, values in columns.items()}
    if not all(np.isfinite(values).all() for values in data.values()):
        raise ValueError("Nonfinite result")
    q, v, t = data["q"], data["v"], data["t"]
    duration = CONTRACT["leadS"] + CONTRACT["liftAndHoldS"]
    if (dt <= 0 or len(t) != round(duration/dt) or result["outcome"] != "HORIZON"
            or abs(result["end"] - duration) > 1e-10 or result["leadS"] != CONTRACT["leadS"]):
        raise ValueError("Incomplete route")
    if q.shape != (len(t), 12) or v.shape != (len(t), 11) or data["normal"].shape != (len(t), 2):
        raise ValueError("Unexpected physical trace dimensions")
    if (abs(result["clockStart"] - CONTRACT["acquiredAtS"]) > 1e-8
            or np.max(abs(t - np.arange(1, len(t)+1)*dt)) > 1e-9
            or np.max(abs(data["clock"] - t - result["clockStart"])) > 1e-8):
        raise ValueError("Physical clock mismatch")
    if np.max(abs(np.linalg.norm(q[:, 8:12], axis=1) - 1)) > 1e-9:
        raise ValueError("Nonunit NOTE orientation")

    heading = q[:, 2]
    cos_yaw, sin_yaw = np.cos(heading), np.sin(heading)
    w, x, y, z = q[:, 8:12].T
    world_axis = np.column_stack((2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)))

    def robot_frame(a):
        return np.column_stack((cos_yaw*a[:, 0] + sin_yaw*a[:, 1],
                                -sin_yaw*a[:, 0] + cos_yaw*a[:, 1], a[:, 2]))

    center = q[:, 5:8].copy()
    center[:, :2] -= q[:, :2]
    center, axis = robot_frame(center), robot_frame(world_axis)
    gap = q[:, 7] - .0254 - .1524*np.linalg.norm(world_axis[:, :2], axis=1)
    if max(np.max(abs(computed - data[key])) for computed, key in
           ((gap, "gap"), (center, "centerRobot"), (axis, "normalRobot"))) > 1e-9:
        raise ValueError("Reported geometry differs from rigid poses")
    chassis_rates = np.diff(q[:, :3], axis=0)/dt
    rate_errors = abs(chassis_rates - (v[1:, :3] + v[:-1, :3])/2).max(axis=0)
    if np.any(rate_errors > CONTRACT["maxChassisRateErrorPerSecond"]):
        raise ValueError("Chassis poses and reported velocities disagree")
    forward = cos_yaw*v[:, 0] + sin_yaw*v[:, 1]
    load = t >= result["leadS"] - 1e-12
    hold = t >= duration - CONTRACT["holdS"] - 1e-12
    concurrent, rise = overlap(t[load], forward[load], v[load, 2], q[load, 3], heading[load], result["case"])
    airborne = np.flatnonzero(gap > 1e-5)
    floor_return = not len(airborne) or bool(np.any((gap[airborne[0]:] <= 0) | (data["floorN"][airborne[0]:] > 0)))

    target = math.radians(CONTRACT["targetPitchDegrees"])
    desired = [-math.sin(target), 0., math.cos(target)]
    angle_error = math.degrees(np.max(np.arccos(np.clip(axis[hold] @ desired, -1., 1.))))
    pitch_error = math.degrees(np.max(abs(q[hold, 4] - target)))
    clearance = float(gap[hold].min())
    min_normal = float(data["normal"].min())
    pivot_error = np.max(abs(q[hold, 3] - CONTRACT["targetCarriageTravelM"]))
    energy_error = float(abs(data["residualJ"]).max())
    port_error = float(abs(data["contactPowerResidual"]).max())
    integrity = (energy_error <= CONTRACT["maxEnergyResidualJ"] and port_error <= CONTRACT["maxContactPowerResidualW"]
                 and np.all((data["busV"] > 0) & (data["busV"] <= 12.00001))
                 and np.min(np.diff(data["heatJ"])) >= -1e-10)
    passed = (concurrent and not floor_return and min_normal > CONTRACT["minSideNormalNExclusive"]
              and clearance >= CONTRACT["minHoldWholeRingGapM"] and pivot_error <= CONTRACT["maxHoldTravelErrorM"]
              and max(angle_error, pitch_error) <= CONTRACT["maxHoldOrientationErrorDegrees"] and integrity)
    lateral_accel = -sin_yaw*np.gradient(v[:, 0], dt) + cos_yaw*np.gradient(v[:, 1], dt)
    summary = dict(
        case=result["case"], dt=dt, grid=result["grid"], steps=len(t), passed=bool(passed),
        overlap=concurrent, riseDuringOverlapM=rise, minForwardDuringLiftMps=float(forward[load].min()),
        minAbsYawDuringLiftRadps=float(abs(v[load, 2]).min()),
        headingChangeDegrees=math.degrees(heading[load][-1] - heading[load][0]),
        drivePathDuringLiftM=float(np.linalg.norm(np.diff(q[load, :2], axis=0), axis=1).sum()),
        meanForwardDuringLiftMps=float(forward[load].mean()), meanYawDuringLiftRadps=float(v[load, 2].mean()),
        meanLateralAccelerationMps2=float(lateral_accel[load].mean()), finalTravelM=float(q[-1, 3]),
        finalNoteCenterHeightM=float(q[-1, 7]), holdClearanceM=clearance, holdNoteAngleErrorDegrees=angle_error,
        firstAirborneS=float(t[airborne[0]]) if len(airborne) else None, floorReturn=floor_return,
        minSideNormalN=min_normal, maxLateralOffsetMm=1000*float(abs(center[:, 1]).max()),
        maxEnergyDriftJ=energy_error, maxContactPowerResidualW=port_error, maxChassisRateErrors=rate_errors.tolist(),
        traceSha256=digest(folder / "trace.jsonl"), inputs=result["inputs"])
    states = np.column_stack((q[:, :2], q[:, 3], q[:, 5:8], v[:, :2], v[:, 3], v[:, 5:8],
                              q[:, 2], q[:, 4], v[:, 2], v[:, 4], world_axis))
    return summary, states


def compare(coarse, fine):
    first, a = coarse
    second, b = fine
    stride = round(first["dt"]/second["dt"])
    if first["case"] != second["case"] or stride < 1 or len(a)*stride != len(b):
        raise ValueError("Unaligned integrations")
    matching = b[stride-1::stride]
    diff = abs(a - matching).max(axis=0)
    normal_chord = np.linalg.norm(a[:, 16:19] - matching[:, 16:19], axis=1).max()
    return dict(positionM=float(max(diff[:6])), velocityMps=float(max(diff[6:12])),
                angleRad=float(max(diff[12:14])), angularVelocityRadps=float(max(diff[14:16])),
                noteNormalAngleRad=float(2*np.arcsin(min(1., normal_chord/2))),
                withinBounds=bool(max(diff[:6]) <= CONTRACT["finestPairTranslationM"]
                                  and max(diff[6:12]) <= CONTRACT["finestPairLinearVelocityMps"]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    measured, _ = assess(parser.parse_args().run)
    print(json.dumps(measured, indent=2))
    raise SystemExit(0 if measured["passed"] else 1)
