# Successive floor pickups with Profile A

The decision is whether to retain the current cassette for Detailed design. The acceptance run requires six distinct floor NOTES through the full speaker opening and both counters in 120 seconds on 19 of the 20 frozen starts, with the loaded power and service checks satisfied.

The previous two nominal runs reached all six counters. On the finer run, the field observation originally ended when the piece dropped below 170 mm. Observing the scored piece on the collection floor through the stationary return allowed the design to overlap that return with counting. Both tested collection-floor treatments reproduced the original launch, opening, both counter times and minimum bus voltage. Sixth-score times were 115.629 s and 115.463 s. See ../bounded-lift-01/tail-timing.json and the full fine replay on the LAN.

This successor enforces the outward flap stop during contact-driven pickup and carries belt speeds, moving nose-pad position, flap angle and angular velocity into each following acquisition. The matched coarse first-piece run limited simulated outward overtravel to about 0.13 degrees. The earlier carry allowed more than 12 degrees. Empty return includes the cam, gravity, hinge friction and commanded parent motion; ground traction and sensing still need physical calibration. The shooter logs feed speed and the two independent launcher speeds through field collection. Each run records the source shot and empty-return receipt at the next pickup.

Run mission.py with the project physics interpreter. The two nominal commissioning runs precede the frozen 20-start roster. See run-dev-48.json, run-dev-96.json and current.log for partial progress. A missed seat, uncounted shot, late sixth score or inadequate firing voltage stops that start. Two unsuccessful roster starts rule out the target for this revision. The motor service fixture has a separate drawing and receipt in ../removable-slide-01.

