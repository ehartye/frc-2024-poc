"""Quasi-static floor entry, suspended carry, and unpowered floor withdrawal screens."""
import json
import numpy as np
from contact import HERE, SEAT, TUBE, MAJOR

MASS = .2413
GRAVITY = 9.80665
BELT_MU = .6
CARPET_MU = .35
FREE_DRAG = .035


def evaluate(n):
    # As-authored capsule axes and extents are independently bound to the tread
    # survey. Integrate 10^7 N/m^3 over torus material area, in SI units.
    a = (np.arange(n)+.5)*2*np.pi/n
    b = a.copy()
    r = MAJOR + TUBE*np.cos(b)
    px = np.cos(a[:, None])*r
    py = np.sin(a[:, None])*r
    pz = np.broadcast_to(TUBE*np.sin(b), px.shape)
    area = np.broadcast_to(TUBE*r*(2*np.pi/n)**2*1e-6, px.shape)
    keep = (abs(py)>170) & (abs(pz)<6.35)
    px, py, pz, area = (v[keep] for v in (px, py, pz, area))

    def forces(x, lateral=0):
        dx = x+px - np.clip(x+px, 158.95, 324.05)
        dy = py+lateral - np.sign(py)*191.05
        radius = np.hypot(dx, dy)
        planar = 14.62761-radius
        axial = 6.35-abs(pz)
        depth = np.maximum(0, np.minimum(planar, axial))
        normal = 1e7*area*depth*.001
        # A top/bottom face normal is vertical. Count only sidewall patches
        # toward traction and use both faces when bounding floor drag.
        sidewall = normal*(planar <= axial)
        return np.array([normal.sum(), sidewall.sum(), np.sum(sidewall*dx/radius),
                         np.sum(sidewall*dy/radius)])

    def sweep(step):
        # Beyond x=350 all patches clear. Integrate both directions over
        # the same seated-to-free stroke, keeping the worst sampled demand.
        loads = np.array([forces(x) for x in np.arange(SEAT, 350+step, step)])
        return {key: float(np.max(abs(loads[:, 2])+drag*loads[:, 0]))
                for key, drag in (("free", FREE_DRAG), ("braked", BELT_MU))}

    normal, sidewall, axial, lateral = forces(SEAT)
    return {"grid": n, "seatNormalN": round(normal, 5),
            "tractionN": round(BELT_MU*sidewall, 5),
            "lateralAtPoint1MmN": round(abs(forces(SEAT, .1)[3]), 5),
            "weightN": round(MASS*GRAVITY, 5),
            "floorLimitN": round(CARPET_MU*MASS*GRAVITY, 5),
            "sweepStepMm": .25,
            "withdrawalN": {k: round(v, 5) for k,v in sweep(.25).items()},
            "halfStepWithdrawalN": {k: round(v, 5) for k,v in sweep(.125).items()}}


if __name__ == "__main__":
    (HERE/"loads.json").write_bytes((json.dumps([evaluate(n) for n in (256,512,1024)],indent=2)+"\n").encode())
