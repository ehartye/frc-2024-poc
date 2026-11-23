# Loaded cassette actuation

A finite-mass mechanism cannot make the abrupt horizontal velocity change in the original prescribed slide law. At 0.25 ms the numerical peak was 1,009 m/s². The bounded 4.8 s candidate uses a smooth slide curve with unchanged pickup and launch endpoints. The largest geometric change is 7.02 mm. The coarse free-piece mission and the rendering use this corrected trajectory.

The selected design target is a shared 250:1 pitch drive and an independently driven 8 mm/revolution screw with a NEO 550. The NEO 550 has 35 mm case diameter, 44.5 mm case length and a published mass of 142 g. The original STEP places its 13 mm pilot and 3.17 mm exposed shaft along the measured screw axis. The generic mounted motor was 28 mm in diameter. Boring the trial plate to 13.1 mm removes the 162.58 mm³ pilot clash. Both positions examined on the existing 232-body root clear the conservative parked can and terminal envelopes. The straight drivers encounter the back wall and screw bearing block. A removable motor plate installed with its two screws accessible on the bench is the next mounting check. Estimated screw lead/efficiency, screw critical speed, end bearings and positive coupling retention remain open. motor-envelope.json lists every affected stock/tool intersection.

At a 69.013 kg upper running weight, with 2 kg of reserve conditionally distributed to the rotating assembly, the worst synchronized battery load is 114.35 A. The weak pack falls to 8.712 V. The smallest motor terminal margin is 0.456 V. These are separate extrema. A 39 A allowance for other loads keeps the bus above 8 V but produces a negative motor terminal margin; stage loads to remain within 25 A during this motion. The slide motor bound reaches 38.93 A phase, and the pivot needs −29.65 to +31.66 A. A symmetric 35 A pickup-mode pivot limit is required by this screen. The simulated climb uses its own −25/+35 A limits. Bidirectional torque, actual PWM/efficiency, guide friction, unplaced mass, pack condition and thermal duty need validation.

The NEO curve uses the REV-21-1651 data sheet, including 100 A stall current, 0.97 N m stall torque and 11,000 rpm free speed. The assumed screw drive includes 65% efficiency, 15 N guide drag and 2e−5 kg m² combined rotor/spindle inertia. The power calculation independently bounds the unknown distribution of each moving mass group using its registered extents. No kilograms are deducted for replacing the slide motor pending the whole installation.

- slide-demand.json measures the original command and two smoothing windows at four numerical resolutions.
- power.json and retime.json compare 3.4, 4.0, 4.4, 4.8 and 5.2 s with the provisional drive.
- purchased-power.json binds the selected operating point to the manufacturer's curve and the simultaneous drive, pitch and slide loads.
- purchased-motor.json identifies the actual motor STEP and its axial mounting faces, pilot, shaft and hole coordinates.
- ../bounded-lift-01/REVIEW.md records mission timing, sampled motion clearances and the playable review.



The exact purchased motor comparison exceeded its 10 minute budget before producing a per-clock verdict. Its source and terminal receipt remain available. The simpler envelope test finished on the current assembly and exposes the obstructed driver paths at both ends of the screw. The installed purchased motor geometry and its mounting/service sequence remain open.
