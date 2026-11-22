"""Risk register of release 004.

Each open risk is closed by a named design decision or carried as a bench test with a pass value and a
fail value. `evidence` names the files in the evidence tree that quantify the risk. `condition` marks a
risk that is also an operating condition no checked case covers.
"""


def _bench(measure, passing, failing, unit):
    return {"measure": measure, "pass": passing, "fail": failing, "unit": unit}


RISKS = [
    dict(id="scoring-repeatability", condition="Twenty perturbed starts of the six-NOTE mission",
         risk="Six scores in 120 s on 19 of 20 starts. In the frozen simulation campaign at the 6.88 m/s command three of "
              "seven perturbed starts failed (R02 scored 2, R03 scored 5 inside the 7 s window, R06 scored 0) and thirteen "
              "were not run. The misses happen inside the speaker, where the model assumes the field conveyor speeds. "
              "A 6.96 m/s command recovered R02 and R03 in single shots and is the first setting to try.",
         evidence="counted-reset-power-02/run-R02-96.json, run-R03-96.json, run-R06-96.json; shared-counter-03/design-decision.json",
         benchTest=_bench("Starts out of 20 on a full field speaker that score six floor NOTES within 120 s", 19, 18, "starts")),
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
    dict(id="floor-pickup-calibration",
         risk="Carpet grip, tread compliance and lip height in the pickup trial are assumed. A high, offset, low-grip "
              "pickup failed on two refinements; the usable tip-height band is unmeasured.",
         evidence="bounded-risk-01/results.json; pickup-height-control-01/",
         benchTest=_bench("Floor pickups out of 20 on competition carpet that seat within 2.82 s at the nominal tip height",
                          19, 18, "pickups")),
    dict(id="seat-sensor",
         risk="Departure waits for a seat beam held 40 ms. The sensor, its mount and its response to a deformed NOTE are "
              "not selected.",
         evidence="seat-interlock-01/R08-inner-96.json, blind-R08-inner-96.json",
         benchTest=_bench("Seated NOTES out of 20 confirmed by the seat sensor within 0.2 s of seating", 20, 19, "NOTES")),
    dict(id="auxiliary-load-during-lift",
         risk="With 39 A of other loads the governing loaded lift misses the pivot motor terminal demand by 0.089 V. The "
              "budget stages other loads to 25 A during the lift, which leaves 0.448 V.",
         evidence="mounted-slide-02/gross-power.json",
         benchTest=_bench("Battery current of all loads other than drive, pivot and slide during the loaded lift", 25, 39, "A")),
    dict(id="weak-battery-shot",
         risk="The weak-pack bus falls to 8.344 V during feed and launch, 0.344 V above the 8 V floor. Pack model: 11.8 V "
              "open circuit and 27 milliohm.",
         evidence="counted-reset-power-02/run-dev-48.json",
         benchTest=_bench("Lowest bus voltage during a shot on a pack at 11.8 V open circuit", 8.0, 7.9, "V")),
    dict(id="inspection-mass",
         risk="The upper inspection estimate is 56.270 kg against 56.699 kg: 0.429 kg of headroom, with a 3 to 6 kg "
              "unallocated reserve inside the estimate. The control-board guard and detailing growth draw on that reserve.",
         evidence="mounted-slide-02/gross-power.json",
         benchTest=_bench("Weighed inspection mass without battery and bumpers", 56.69, 56.70, "kg")),
    dict(id="endgame-clock",
         risk="Six scores followed by the climb on one 135 s clock leaves 0.18 s.",
         evidence="counted-reset-power-02/endgame-clock.json",
         closedBy="Mission rule from the strategic narrative: with 24 s left the robot leaves the scoring lane for the "
                  "stage. The separate 24 s endgame case reaches power cut with 1.99 s remaining. A sixth score before "
                  "the climb is not required."),
    dict(id="climb-floor-clearance", condition="Forward centre of gravity, 40 mm ahead of nominal",
         risk="After power cut the forward-CG climb holds 11.5 mm above the carpet against a 10 mm allowance. The centre "
              "of gravity is unmeasured.",
         evidence="counted-reset-power-02/climb-gross-front.json",
         benchTest=_bench("Lowest tire or bumper clearance during a 5 s disabled hold at the forward CG limit", 10, 0, "mm")),
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
              "catch engagement under load and wear are unproven.",
         evidence="counted-reset-power-02/climb-gross-front.json; guided-hoist-01/return-no-catch.json",
         benchTest=_bench("Chassis drop during a 5 s disabled hold on the stops at 1.5 times robot weight", 0, 1, "mm")),
    dict(id="battery-service-restraint",
         risk="The removable slide link and index pin hold the cassette for hands-on battery service. Thread engagement, "
              "proof load and operator access are unverified.",
         evidence="guided-hoist-01/truck-lock.json, service-pin.json",
         benchTest=_bench("Horizontal load held by the fitted slide link without movement", 1.29, 1.0, "kN")),
    dict(id="cover-leaf-park-pose",
         risk="The layout STEP draws the cover leaf closed at the parked angle; the cam of the trials holds it open above "
              "102 degrees.",
         evidence="counted-reset-power-02/empty_return.py",
         closedBy="The cam law governs: every cycle case moves the leaf by the cam and the recorded trace. The detailed "
                  "model draws the leaf in its cam position at park."),
]
