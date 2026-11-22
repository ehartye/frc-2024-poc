"""Risk register of release 004.

Each open risk is closed by a named design decision or carried as a bench test with a pass value and a
fail value. `evidence` names the files in the evidence tree that quantify the risk. `condition` marks a
risk that is also an operating condition no checked case covers. Figures that come from the single-shot
scoring campaign are read from its summary when the release is built.
"""


def _bench(measure, passing, failing, unit):
    return {"measure": measure, "pass": passing, "fail": failing, "unit": unit}


def _pct(part):
    return f"{100 * part:.0f}%"


def risks(operating):
    """The register, given the operating-point record written from the campaign summary."""
    here, trial = operating["atFiringStation"]["frozenStarts"], operating["atTrialStation"]["frozenStarts"]
    everything = operating["allFrozenShots"]
    late = operating["extendedObservation"]
    lane, old = operating["firingStation"], operating["trialStation"]
    counted = here["counted"] / here["shots"]
    six = counted ** 6
    window = operating["deliveryWindow"]

    def span(axis):
        w = window[axis]
        return f"{min(w['delivers']):g} to {max(w['delivers']):g} {w['unit']}"
    static = [
        dict(id="scoring-repeatability", condition="Twenty perturbed starts of the six-NOTE mission",
             risk=f"Six scores in 120 s on 19 of 20 starts is not supported by simulation. From the firing station every "
                  f"one of {here['shots']} perturbed carried NOTES passes through the opening, but {here['counted']} "
                  f"({_pct(counted)}) shadow both speaker beams within 7 s; six in a row at that rate is about {_pct(six)}. "
                  f"Across all {everything['shots']} perturbed shots of the campaign {everything['delivered']} were delivered "
                  f"and {everything['counted']} counted. The count rate did not move with launcher speed (6.56 to 7.68 m/s), "
                  f"the speaker's counter drive speed or its surface friction. Of {late['shots']} uncounted NOTES watched for "
                  f"20 s, {late['countedLate']} counted late, {late['stillInSpeaker']} stayed in the speaker and "
                  f"{late['leftSpeaker']} came back out. The frozen full-mission campaign at the trial station failed three of "
                  f"its first seven starts. What happens to a NOTE inside the speaker rests on assumed field conveyor speeds "
                  f"and friction, so the deciding evidence is a real speaker.",
             evidence="scoring-window-04/summary.json; counted-reset-power-02/run-R02-96.json, run-R03-96.json, run-R06-96.json",
             benchTest=_bench("Starts out of 20 on a full field speaker that score six floor NOTES within 120 s", 19, 18, "starts")),
        dict(id="firing-lane-tolerance", condition="Chassis position and heading error at the firing station",
             risk=f"The cycle trials fired from y = {old['yM']:g} m, where the NOTE cleared the side of the opening by "
                  f"{trial['smallestOpeningMarginMm']:.1f} mm and a 2 degree heading error or a 50 mm lateral error lost every "
                  f"shot. The release moves the firing lane to y = {lane['yM']:g} m ({here['smallestOpeningMarginMm']:.1f} mm "
                  f"smallest margin). There every shot is delivered over heading errors of {span('yaw')}, lateral positions of "
                  f"{span('y')} and stations of {span('x')} from the alliance wall. Chassis tracking to that window is unmeasured.",
             evidence="scoring-window-04/summary.json; cad/preliminary-design/releases/004/operating-point.json",
             benchTest=_bench("Largest heading error at the firing station over 20 autonomous arrivals", 2.0, 3.0, "deg")),
        dict(id="launch-angle-tolerance",
             risk=f"Delivery holds for cassette launch angles of {span('angle')} at the firing station. The released 122 degree "
                  f"setpoint is near the low end of that window. Detailed design should put the launch hard stop at the centre "
                  f"of the window and rerun the lift power and cycle traces at that angle.",
             evidence="scoring-window-04/summary.json",
             benchTest=_bench("Lowest cassette launch angle measured over 20 lifts to the stop", 121.5, 121.0, "deg")),
        dict(id="launch-motor-clearance",
             risk="The whole-cycle check found both launch belt motor cans exactly at the NOTE's outer radius (177.8 mm from "
                  "the cassette centre): the floor NOTE riding over the pickup lip passed them with 0.0004 mm. Revision 004-A "
                  "raises both motors 60 mm along the cassette normal on lengthened shafts; the checked cycle then clears "
                  "them by more than 2.8 mm (15 mm sampled). The taller motor mount is not drawn and the pickup trial does "
                  "not model the motors, so an off-centre NOTE meeting the lengthened shaft is untested.",
             evidence="cad/preliminary-design/releases/004/step-revision.json; cad/preliminary-design/build-004/revise_step.py",
             benchTest=_bench("Smallest gap between a NOTE picked up 50 mm off centre and either launch motor or its shaft", 3.0, 0.0, "mm")),
        dict(id="returning-note", condition="NOTE ejected from the speaker back onto the robot",
             risk="In start R02 the third NOTE left the speaker and entered the roboRIO envelope by up to 25.0 mm. No guard is "
                  "drawn. Detailed design fits a guard over the electrical bay, charged to the unallocated reserve.",
             evidence="shared-counter-03/return-to-robot.json",
             benchTest=_bench("Deepest entry into the control-board envelope of a NOTE dropped from the speaker opening, guard fitted",
                              0, 1, "mm")),
        dict(id="entry-bracket-clearance",
             risk="During discharge the unloaded NOTE passes the right entry bearing bracket with 0.035 mm sampled clearance, "
                  "bounded above 0.015 mm. NOTE tolerance or assembly error consumes it. Detailing relieves the bracket.",
             evidence="cad/preliminary-design/motion-evidence-002/exit-bracket-clearance.json",
             benchTest=_bench("Smallest gap between a nominal NOTE and either entry bearing bracket through discharge", 1.0, 0.0, "mm")),
        dict(id="motor-package-clearance",
             risk="The free NOTE clears the slide motor package by a guaranteed 0.189 mm under a 12 m/s relative-speed bound.",
             evidence="mounted-slide-02/free-piece-clearance.json",
             benchTest=_bench("Smallest gap between the NOTE and the slide motor package through pickup and lift", 1.0, 0.0, "mm")),
        dict(id="belt-indentation",
             risk="The centre intake belt indents the NOTE 7.56 to 7.67 mm against an assumed 8 mm one-sided limit. A 7 mm "
                  "limit rejects both numerical resolutions. No force against indentation has been measured.",
             evidence="shared-counter-03/view-contact-96.json, view-contact-48.json; cad/preliminary-design/compression-limits-002.json",
             benchTest=_bench("One-sided NOTE indentation under the centre intake belt at the seat", 8.0, 8.1, "mm")),
        dict(id="runner-indentation",
             risk="The unloaded NOTE overlaps each hard runner by up to 1.36 mm against a 2 mm planning limit. Runner height "
                  "is adjustable; hard clearance and elastic compression are not yet separated.",
             evidence="shared-counter-03/view-contact-96.json",
             benchTest=_bench("One-sided NOTE indentation on either runner with the NOTE seated", 2.0, 2.1, "mm")),
        dict(id="pickup-tip-height",
             risk="The pickup seats the NOTE with the rigid blade tip 1.5 to 10 mm above the carpet and fails from 12 mm up "
                  "(5 mm nominal). Carpet pile, tread compliance and chassis pitch under load are assumed.",
             evidence="pickup-brackets-04/results.json; pickup-height-control-01/",
             benchTest=_bench("Blade tip height above competition carpet with the cassette deployed and the robot loaded", 10, 12, "mm")),
        dict(id="pickup-lateral-offset", condition="Floor NOTE more than 50 mm off the cassette centre",
             risk="The pickup seats a NOTE up to 50 mm off centre and fails at 100 mm on either side. Driver or vision "
                  "alignment to that band is unmeasured.",
             evidence="pickup-brackets-04/results.json",
             benchTest=_bench("Floor pickups out of 20 that seat within 2.82 s with the NOTE 50 mm off centre", 19, 18, "pickups")),
        dict(id="pickup-grip",
             risk="The pickup seats the NOTE with a nose roller and pad grip coefficient of 0.30 or more and fails at 0.20 "
                  "(0.45 assumed). The grip of the selected tread on a worn NOTE is unmeasured.",
             evidence="pickup-brackets-04/results.json",
             benchTest=_bench("Measured grip coefficient of the nose tread on a competition NOTE", 0.3, 0.2, "coefficient")),
        dict(id="seat-sensor",
             risk="Departure waits for a seat beam held 40 ms. The sensor, its mount and its response to a deformed NOTE are "
                  "not selected.",
             evidence="seat-interlock-01/R08-inner-96.json, blind-R08-inner-96.json",
             benchTest=_bench("Seated NOTES out of 20 confirmed by the seat sensor within 0.2 s of seating", 20, 19, "NOTES")),
        dict(id="cover-leaf-stop",
             risk="The NOTE stayed seated in every lift bracket, but the cover leaf leaves its cam stop by more than 1 degree "
                  "when the lift is shorter than 1.8 s (0.13 degree at the released 4.8 s). The leaf spring and stop are "
                  "assumed values.",
             evidence="retention-brackets-04/results.json",
             benchTest=_bench("Cover leaf overtravel past its cam stop during a 4.8 s loaded lift", 1.0, 2.0, "deg")),
        dict(id="auxiliary-load-during-lift",
             risk="With 39 A of other loads the governing loaded lift misses the pivot motor terminal demand by 0.089 V. The "
                  "budget stages other loads to 25 A during the lift, which leaves 0.448 V. The 004-A motor raise adds about "
                  "1 percent to the lifted moment and is not in that study.",
             evidence="mounted-slide-02/gross-power.json",
             benchTest=_bench("Battery current of all loads other than drive, pivot and slide during the loaded lift", 25, 39, "A")),
        dict(id="weak-battery-shot",
             risk="The weak-pack bus falls to 8.344 V during feed and launch, 0.344 V above the 8 V floor. Pack model: 11.8 V "
                  "open circuit and 27 milliohm.",
             evidence="counted-reset-power-02/run-dev-48.json",
             benchTest=_bench("Lowest bus voltage during a shot on a pack at 11.8 V open circuit", 8.0, 7.9, "V")),
        dict(id="inspection-mass",
             risk="The upper inspection estimate is 56.270 kg against 56.699 kg: 0.429 kg of headroom, with a 3 to 6 kg "
                  "unallocated reserve inside the estimate. The control-board guard, the lengthened launch shafts (0.07 kg), "
                  "the taller launch motor mounts and detailing growth draw on that reserve.",
             evidence="mounted-slide-02/gross-power.json",
             benchTest=_bench("Weighed inspection mass without battery and bumpers", 56.69, 56.70, "kg")),
        dict(id="endgame-clock",
             risk="Six scores followed by the climb on one 135 s clock leaves 0.18 s.",
             evidence="counted-reset-power-02/endgame-clock.json",
             closedBy="Mission rule from the strategic narrative: with 24 s left the robot leaves the scoring lane for the "
                      "stage. The separate 24 s endgame case reaches power cut with 1.99 s remaining. A sixth score before "
                      "the climb is not required."),
        dict(id="climb-centre-of-gravity", condition="Centre of gravity more than 50 mm ahead of nominal",
             risk="After power cut the climb holds 11.5 mm above the carpet with the centre of gravity 40 mm ahead of nominal, "
                  "10.9 mm at 50 mm and 8.4 mm at 60 mm, against a 10 mm allowance. Rearward it passes to 120 mm. The centre "
                  "of gravity is unmeasured.",
             evidence="climb-brackets-04/results.json; counted-reset-power-02/climb-gross-front.json",
             benchTest=_bench("Measured centre of gravity ahead of the nominal position, robot at gross mass", 50, 60, "mm")),
        dict(id="hoist-ramp",
             risk="The climb latches and holds with a 6.5 s hoist ramp. A 5 s ramp leaves 6.9 mm of floor clearance and ramps "
                  "of 4 s and shorter do not latch. The ramp is a controls setting with one passing value tried.",
             evidence="climb-brackets-04/results.json",
             benchTest=_bench("Lowest tire or bumper clearance during a 5 s disabled hold after a 6.5 s hoist ramp", 10, 9, "mm")),
        dict(id="hoist-line-termination",
             risk="Installed splice, drum dead end and bend strength of the 3 mm hoist lines are unqualified. The installed "
                  "line target is 5.73 kN.",
             evidence="guided-hoist-01/return-budget.json, inboard-exchange.json",
             benchTest=_bench("Proof load held by each terminated hoist line without slip or damage", 5.73, 5.7, "kN")),
        dict(id="hoist-line-slack",
             risk="Without take-up up to 227 mm of line pays out after the motor is disconnected. A 0.30 N m passive drum "
                  "bias holds the modeled slack to 0.19 mm; the spring is a purchasing allowance.",
             evidence="guided-hoist-01/return-front.json, winch-front.json",
             benchTest=_bench("Largest slack in either hoist line through full carriage travel, take-up fitted", 5, 10, "mm")),
        dict(id="climb-catch-and-stops",
             risk="Slide stop pins, the pitch lock and the keeper are ideal constraints in the climb trial. Pin capacity, "
                  "catch engagement under load and wear are unproven. With the catch disabled the robot returns to the "
                  "carpet.",
             evidence="climb-brackets-04/results.json; counted-reset-power-02/climb-gross-front.json",
             benchTest=_bench("Chassis drop during a 5 s disabled hold on the stops at 1.5 times robot weight", 0, 1, "mm")),
        dict(id="battery-service-restraint",
             risk="The removable slide link and index pin hold the cassette for hands-on battery service. Thread engagement, "
                  "proof load and operator access are unverified.",
             evidence="guided-hoist-01/truck-lock.json, service-pin.json",
             benchTest=_bench("Horizontal load held by the fitted slide link without movement", 1.29, 1.0, "kN")),
        dict(id="battery-removal-path", condition="Battery lifted out at the service index",
             risk="The straight 370 mm battery lift at the service index is not a checked case: the disconnected pack lead is "
                  "not represented and the lift does not yet pass. The checked service case is the index pin withdrawal.",
             evidence="cad/preliminary-design/build-004/build.py (--battery)",
             benchTest=_bench("Smallest gap around the battery through a straight lift at the service index, leads disconnected",
                              5, 0, "mm")),
        dict(id="slide-motor-cartridge",
             risk="The removable slide motor cartridge and the battery-service slide link are not among the layout solids, so "
                  "their withdrawal paths are not in the checked cycle.",
             evidence="mounted-slide-02/; guided-hoist-01/removable-truck-lock.step",
             benchTest=_bench("Smallest gap around the slide motor cartridge through its full withdrawal", 3, 0, "mm")),
        dict(id="cover-leaf-park-pose",
             risk="The layout STEP draws the cover leaf closed at the parked angle; the cam of the trials holds it open above "
                  "102 degrees.",
             evidence="counted-reset-power-02/empty_return.py",
             closedBy="The cam law governs: every cycle case moves the leaf by the cam and the recorded trace. The detailed "
                      "model draws the leaf in its cam position at park."),
    ]
    assert len({r["id"] for r in static}) == len(static)
    return static
