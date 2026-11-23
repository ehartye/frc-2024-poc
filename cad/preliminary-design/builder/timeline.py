"""Joint-space timeline of one scoring cycle, read from the pinned trial traces.

One floor NOTE (the first, "middle", of the fine nominal run) is followed from first approach through
pickup, the loaded lift, feed, free flight into the speaker and the empty return. NOTE poses and the
passive cover angle are recorded simulation output. The cassette angle and the pivot-truck position are
the commanded laws the trials were run with; the laws are evaluated here from the same source modules.

All poses are in the chassis frame, which is the parked STEP frame, in millimetres.
"""
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

import kin

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
# One constant names the evidence tree, so relocating it changes one line.
EVIDENCE = REPO / "2024/preliminary-review-lan/greenfield-retained"
# The release folder the builder writes, and the files carried from the draft layout (inputs/README.md).
RELEASE = REPO / "cad/preliminary-design/releases/1.0"
INPUTS = HERE / "inputs"
CAMPAIGN = EVIDENCE / "counted-reset-power-02"
RESOLUTION = 96
NOTE_ID = "middle"
NEXT_ID = "outer"
LIFT_S = 4.8
LAUNCH_DEG = 122.0


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def pin(path):
    """A source pin: repository-relative path and SHA-256."""
    path = Path(path)
    return dict(file=path.relative_to(REPO).as_posix(), sha256=sha(path))


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def slide_law():
    """The scheduled pivot-truck position against cassette angle (follower-01/stroke.py)."""
    return _module("stroke", EVIDENCE / "follower-01/stroke.py").scheduled


def smooth_table(scheduled, deploy, final, x0):
    """The finite-acceleration slide command of loaded-actuation-01/smooth.py, without SciPy.

    Returns (times, positions in metres). The original uses scipy.ndimage.convolve1d with mode="nearest";
    the kernel is symmetric, so an edge-padded NumPy convolution gives the same samples.
    """
    times = np.arange(-.25, 3.65 + .00025, .00025)
    u = np.clip(times / 3.4, 0, 1)
    b = u * u * (3 - 2 * u)
    a = deploy + (final - deploy) * b
    original = (scheduled(a) + (x0 - scheduled(deploy)) * (1 - b)) * .001
    kernel = np.arange(-320, 321) * .00025
    w = 1 + np.cos(2 * np.pi * kernel / .16)
    w = w / w.sum()
    padded = np.pad(original, 320, mode="edge")
    smooth = np.convolve(padded, w, mode="valid")
    z = np.clip(np.minimum(times, 3.4 - times) / .20, 0, 1)
    h = z ** 3 * (10 - 15 * z + 6 * z * z)
    return times, original + h * (smooth - original)


class Cycle:
    """The traces and commanded laws of one scoring cycle."""

    def __init__(self):
        n = RESOLUTION
        self.files = dict(
            run=CAMPAIGN / f"run-dev-{n}.json",
            plan=CAMPAIGN / "plan.json",
            carry=CAMPAIGN / f"dev-{NOTE_ID}-{n}.json",
            shot=CAMPAIGN / f"split-tail_.25_0-dev-{NOTE_ID}-{n}-6.88.json",
            deploy=CAMPAIGN / f"empty-dev-{NOTE_ID}-{n}.json",
            reset=CAMPAIGN / f"empty-dev-{NEXT_ID}-{n}.json",
            freeze=CAMPAIGN / "freeze.json",
            layout=EVIDENCE / "pickup-lip/layout.json",
        )
        run = read(self.files["run"])
        plan = read(self.files["plan"])
        self.part, self.next_part = run["parts"][0], run["parts"][1]
        self.leg = plan["plan"][0]
        assert self.part["id"] == self.leg["id"] == NOTE_ID and self.next_part["id"] == NEXT_ID
        assert run["passMission"] and run["count"] == 6 and run["freeze"] == sha(self.files["freeze"])
        self.carry = read(self.files["carry"])
        self.shot = read(self.files["shot"])
        self.deploy = np.array(read(self.files["deploy"])["trace"])
        self.reset = np.array(read(self.files["reset"])["trace"])
        assert sha(self.files["carry"]) == self.part["carry"] == self.shot["sourceCarry"]
        assert sha(self.files["shot"]) == self.part["shot"]
        assert sha(self.files["deploy"]) == self.part["emptyMotion"]
        assert sha(self.files["reset"]) == self.next_part["emptyMotion"]
        self.q0 = self.carry["pitchDegrees"]
        self.x0 = read(self.files["layout"])["pickPivot"][0]
        self.scheduled = slide_law()
        self.times, self.table = smooth_table(self.scheduled, self.q0, LAUNCH_DEG, self.x0)
        self.station = np.array([*self.leg["loaded"][-1], 0.0])
        self.travel = sum(self.leg["outT"])
        self.carry_start = self.part["depart"] + self.travel + 1.15
        self.shot_start = self.part["shotStart"]
        assert abs(self.carry_start + self.carry["finish"] - self.shot_start) < 1e-5
        self.gate = 2.82 + self.carry["seatDelay"]
        self.start_lift = self.carry["startLift"]
        self.feed = self.shot["feedTime"]
        self.release = self.shot["release"]["t"]
        self.through = self.shot["opening"]["fullyThroughByS"]
        self.counted = self.part["both"]
        self.trigger = self.feed + 1.0
        self.reset_end = self.next_part["depart"] - self.shot_start
        self.rows = np.array([[*r[:8], r[11]] for r in self.carry["trajectory"]], float)
        self.shot_t = np.array([v["t"] for v in self.shot["trace"]])
        self.shot_q = np.array([v["q"] for v in self.shot["trace"]])

    def arm_law(self, tau):
        """Cassette angle (deg) and pivot X (mm) at time tau after the lift starts."""
        u = float(np.clip(tau / LIFT_S, 0, 1))
        angle = self.q0 + (LAUNCH_DEG - self.q0) * u * u * (3 - 2 * u)
        return angle, 1000 * float(np.interp(3.4 * u, self.times, self.table))

    # Each phase returns joints(t): dict(angle, px, flap_deg, note (4x4) or None), t in seconds from the phase start.

    def deploy_state(self, t):
        """Start pose to the pickup pose while the robot approaches the floor NOTE (first leg only)."""
        e = self.deploy
        angle = float(np.interp(t, e[:, 0], e[:, 1]))
        return dict(angle=angle, px=float(self.scheduled(angle)), flap=math.degrees(float(np.interp(t, e[:, 0], e[:, 2]))))

    def carry_state(self, i):
        """Carry trace row i: pickup, seat and the loaded lift. The NOTE and cover leaf are recorded."""
        row = self.rows[i]
        t = row[0]
        angle, px = self.arm_law(t - self.start_lift)
        return dict(t=t, angle=angle, px=px, flap=math.degrees(row[8]),
                    note=kin.piece(1000 * row[1:4], row[4:8]))

    def shot_state(self, j):
        """Shot trace row j at the firing station: feed, release and flight. The NOTE is recorded."""
        q = self.shot_q[j]
        return dict(t=self.shot_t[j], angle=LAUNCH_DEG, px=kin.PARK_PIVOT_X_MM,
                    flap=math.degrees(self.carry["endFlapRad"]),
                    note=kin.piece(1000 * (q[:3] - self.station), q[3:]))

    def reset_state(self, t):
        """Empty return, t seconds after the shot started: commanded cassette and truck, simulated cover leaf."""
        e = self.reset
        angle, px = self.arm_law(LIFT_S - max(0.0, t - self.trigger))
        return dict(t=t, angle=angle, px=px, flap=math.degrees(float(np.interp(t, e[:, 0], e[:, 2]))))

    def sources(self):
        return {k: pin(v) for k, v in self.files.items()}
