"""Replay the passive R9 / R13 chassis comparison from registered members."""
import hashlib
import json
from pathlib import Path
import numpy as np
from grillage import Grillage, Section, E, G
from selection import selected_mass

HERE = Path(__file__).resolve().parent
CAD = HERE.parents[1]
MEMBERS = json.loads((HERE / "members.json").read_bytes())
ROWS = {r["name"]: r for r in MEMBERS["members"]}
R13 = json.loads((HERE.parent / "ground-rails/native-fit.json").read_bytes())
WHEELS = json.loads((CAD / "drivetrain/maxswerve-layout.json").read_bytes())["stations"]
TUBE = Section.tube(38.1, 50.8, 3.175)
DECK = Section.tube(25.4, 50.8, 3.175)
BRIDGE = Section.tube(50, 38.35, 3)
LOAD = 100.


def register():
    for key, name in [("parentSha256", "r9-physical-assembly"), ("archiveSha256", "r13-ground-frame")]:
        assert R13[key] == hashlib.sha256((CAD / "student" / (name+".f3d")).read_bytes()).hexdigest()
    assert MEMBERS["archiveSha256"] == R13["parentSha256"]
    actual = json.loads((CAD / "assembly-readiness/mass/actual.json").read_bytes())
    bodies = {r["body"]: r for r in actual["bodies"]}
    assert MEMBERS["bodyCount"] == actual["nativeBodyCount"] == 923
    for name, r in ROWS.items():
        assert bodies[name]["stock"] == r["material"] == "Aluminum 6061"
        assert abs(bodies[name]["volumeCm3"] - r["volumeCm3"]) < .01
    assert set(R13["verifiedParkedFrameBodies"]) <= ROWS.keys()


def build(bridge, connection, holes=True, perimeter=True, soft_weld=False, brace_width=None, brace_x=38, unfastened_end=False):
    g = Grillage()
    # N/mm per bolt and N mm/rad per group's remaining rotation axis/axes.
    interfaces = {"rigid": None, "firm": (1e5, 1e7), "relaxed": (1e4, 1e6),
                  "rocking": (1e4, 0.)}
    bolt = interfaces[connection]
    weld = (1e5, 1e6) if soft_weld else None
    def member(label, axis, ends, fixed, section, cuts=(), stations=()):
        points = sorted({*ends, *stations, *(v for lo,hi,_ in cuts for v in (lo,hi))})
        assert points[0] == ends[0] and points[-1] == ends[1]
        def node(s):
            x,y = (s,fixed) if axis == 0 else (fixed,s)
            return g.node(label,x,y)
        for lo, hi in zip(points, points[1:]):
            local = next((weak for a,b,weak in cuts if a < (lo+hi)/2 < b), section)
            g.beam(node(lo), node(hi), local if holes else section)
        return node
    def attach(a, b):
        g.join(a, b, [g.xy[a]], weld)
    def flanges(stations, diameter, stock, height, wall, multiplicity):
        return [(p-diameter/2, p+diameter/2, stock.drilled(height,wall,diameter*multiplicity))
                for p in stations]
    frame = [r for r in ROWS.values() if r["name"].endswith("Tab-mounted rectangular rail")]
    rear_bounds = next(r["boundsMm"] for r in frame if r["boundsMm"][3] < 0)
    rear_x = (rear_bounds[0]+rear_bounds[3])/2
    transverse_row = next(r for r in ROWS.values() if abs(r["boundsMm"][4]-r["boundsMm"][1]-621.6)<.01)
    box = transverse_row["boundsMm"]
    deck_x = (box[0]+box[3])/2
    cross = member("aft cross tube", 1, (box[1], box[4]), deck_x, DECK, stations=(-80,80))
    tab_stations = [-228.25,228.25]
    tab_cuts = flanges(tab_stations, 5.1, TUBE, 50.8, 3.175, 2)
    rear = member("rear tube", 1, (rear_bounds[1],rear_bounds[4]), rear_x, TUBE,
                  tab_cuts, [-80,80,*tab_stations])
    deck_load = [cross(-80), cross(80)]
    for row in ROWS.values():
        b = row["boundsMm"]
        if abs(b[3]-b[0]-158.4) < .01:
            y = (b[1]+b[4])/2
            runner = member("aft runner", 0, (b[0],b[3]), y, DECK, stations=[-266.7])
            attach(runner(b[0]), rear(y))
            attach(runner(b[3]), cross(y))
            deck_load.append(runner(-266.7))
    wheels = {k:g.node(k,*r["wheelCenterMm"][:2]) for k,r in WHEELS.items()}
    side = {}
    for row in frame:
        b = row["boundsMm"]
        if b == rear_bounds:
            continue
        y = (b[1]+b[4])/2
        sign = int(np.sign(y))
        cuts = tab_cuts + flanges([23,53],13.1,TUBE,50.8,3.175,1)
        rail = member("side tube",0,(b[0],b[3]),y,TUBE,cuts,[deck_x,23,38,53,brace_x,*tab_stations])
        side[sign] = rail
        attach(cross(sign * box[4]), rail(deck_x))
        for x, end in zip(tab_stations, ["Rear","Front"]):
            hub = wheels[end + (" left" if sign == 1 else " right")]
            g.join(rail(x), hub, [(x,y-d) for d in (-12.7,12.7)], bolt)
            if end == "Rear":
                g.join(rear(sign*228.25), hub,
                       [(rear_x+d,sign*228.25) for d in (-12.7,12.7)], bolt)
    if bridge:
        b = ROWS["Crossmember with four rail mounting holes"]["boundsMm"]
        cuts = flanges([-329,329],13.1,BRIDGE,38.35,3,2)
        transverse = member("removable cross tube",1,(b[1],b[4]),(b[0]+b[3])/2,
                            BRIDGE,cuts,[-329,329])
        for sign, rail in side.items():
            # Both fastener stations resolve onto the same tube end, 30 mm apart.
            # Each bolt samples the side tube at its own longitudinal station.
            for x in [23,53]:
                if unfastened_end and sign == 1:
                    continue
                k = None if bolt is None else (bolt[0],bolt[1]/2)
                g.join(rail(x), transverse(sign*329), [(x,sign*329)], k)
    if brace_width:
        # Butt ends on the inside faces of the two drilled drive rails.
        brace = member("lower frame tie",1,(-310.8,310.8),brace_x,
                       Section.tube(brace_width,50.8,3.175))
        for sign, rail in side.items():
            attach(brace(sign*310.8),rail(brace_x))
    if perimeter:
        # Bumper wood, foam and upper machinery contribute no stiffness.
        # Only the continuous lower aluminum backing bars carry load here.
        inner = member("inner nose tube",1,(-355.6,355.6),375.412,TUBE,stations=[-342.9,-160,160,342.9])
        nose = member("outer nose tube",1,(-355.6,355.6),435.412,TUBE,stations=[-342.9,-160,160,342.9])
        for y in [-342.9,-160,160,342.9]:
            spacer = member("nose spacer",0,(394.462,416.362),y,DECK)
            attach(spacer(394.462),inner(y))
            attach(spacer(416.362),nose(y))
        rear_bar = member("rear backing",1,(-355.25,355.25),-351.28125,
                          Section.bar(4.7625,38.1),stations=tab_stations)
        for y in tab_stations:
            attach(rear_bar(y),rear(y))
        for sign, rail in side.items():
            band = member("side backing",0,(-348.9,356.362),sign*352.075,
                          Section.bar(6.35,38.1),stations=tab_stations)
            for x in tab_stations:
                attach(band(x),rail(x))
            attach(band(-348.9),rear_bar(sign*355.25))
            attach(band(356.362),inner(sign*355.6))
    return g, wheels, deck_load


def evaluate(bridge, joint, **changes):
    model, wheels, deck = build(bridge, joint, **changes)
    # Self-equilibrated incremental wheel forces. Three supports fix the datum.
    loads = {n: LOAD*np.sign(n[1]*n[2]) for n in wheels.values()}
    twist = model.solve(loads, list(wheels.values())[:3])
    flat = model.solve({n:-LOAD/len(deck) for n in deck}, list(wheels.values()))
    result = {"joint":joint, "twistEquilibrium": twist is not None, "deckEquilibrium": flat is not None}
    if twist:
        z,reactions = twist
        warp = sum(z[n]*f/LOAD for n,f in loads.items())
        assert max(map(abs,reactions)) < 1e-3 and warp > 0
        torque = LOAD*.520
        result.update(warpMm=warp, twistNmPerDegree=torque/np.degrees(warp/520))
    if flat:
        z,reactions = flat
        assert abs(sum(reactions)-LOAD) < 1e-3
        result.update(deckMeanDropMm=-sum(z[n] for n in deck)/len(deck),
                      wheelReactionsN=dict(zip(wheels,reactions)))
    return result


def beam_controls():
    # Vertical cantilever tip and twist of a shaft coupled to an eccentric force.
    g = Grillage()
    root = g.node("cantilever", 0, 0)
    tip = g.node("cantilever", 420, 0)
    left = g.node("root support", 0, -30)
    right = g.node("root support", 0, 30)
    back = g.node("root support", -30, 0)
    for pad in (left,right,back):
        g.join(pad,root)
    g.beam(root,tip,TUBE)
    arm = g.node("offset load",420,70)
    g.join(tip,arm)
    expected_bend = LOAD*420**3/(3*E*TUBE.inertia)
    expected_twist = LOAD*70**2*420/(G*TUBE.torsion)
    z,_ = g.solve({arm:LOAD}, [left,right,back])
    assert np.isclose(z[tip], expected_bend, rtol=1e-8)
    assert np.isclose(z[arm]-z[tip], expected_twist, rtol=1e-8)
    return dict(cantileverBendMm=z[tip], eccentricTorsionMm=z[arm]-z[tip])


def study():
    register()
    mass = selected_mass()
    cases = {"R9":dict(bridge=True), "R13":dict(bridge=False),
             "lowerTie":dict(bridge=False,brace_width=38.1),
             "R14":dict(bridge="Crossmember with four rail mounting holes" in mass["addedArticles"])}
    report = {}
    for name, params in cases.items():
        report[name] = {mode:evaluate(**params,joint=mode) for mode in ("rigid","firm","relaxed","rocking")}
        for key, changes in [("grossSections",dict(holes=False)),("noPerimeterCredit",dict(perimeter=False)),
                             ("softWelds",dict(soft_weld=True))]:
            report[name][key] = evaluate(**params,joint="relaxed",**changes)
    report["controls"] = beam_controls()
    report["controls"]["oneTieEndUnfastened"] = evaluate(True,"relaxed",unfastened_end=True)
    report["selection"] = mass
    report["sources"] = {"r9": MEMBERS["archiveSha256"], "r13": R13["archiveSha256"]}
    return report


if __name__ == "__main__":
    print(json.dumps(study(),indent=2,allow_nan=False))
