# Unadopted spring and opening-cam candidate

Lee Spring LC 032EE 01 M is a concrete music-wire candidate, not an arbitrary foam stiffness. The exact manufacturer India catalog PDF row is now privately pinned and visually verified (2019 metadata), with source hashes/pages in `cassette-opening-source-candidate.json`. This supports historical catalog presence, not actual stock availability. Nominal spring inside diameter is 7.91 mm and fits the 6 mm rod geometrically. This does not verify tolerances, allowable working deflection or fatigue.

Two nominal springs supply 14.6234 N at 7 mm installed length and 5.9534 N at 8.5 mm. These are cassette spring forces, NOT measured NOTE normal forces. Ideal opening work is 0.0154326 J before guide friction, moving inertia and losses. Seat adjustment is not used in this first calculation.

`cassette_opening_cam.py` generates editable original eccentric cam/follower coupons at open/mid/released positions. The 0.75 mm eccentricity gives 1.5 mm displacement. The contact equality is only a unilateral-contact geometry constraint, not an achieved motion command. Three isolated coupon pairs have no material overlap and each STEP reopens with two roots; this is not full-robot fit evidence.

REV SRS V1 REV-41-1097 is a historically documented actuator candidate (2017 drawing), not installed or accepted. Its source stall torque/current must not become continuous available torque. A compatible horn, bearing-supported shaft, actual servo bracket, follower attachment, power module, position detection and positive open-position latch remain to be designed. An eccentric at dead center is NOT a latch: perturbation can release stored spring energy. Opening during transfer still needs actual force handoff; it cannot restore zero travel instantaneously or guarantee retained NOTE support.

Canonical robot geometry and reviewed cassette study remain unchanged. No spring force or cam actuation has been adopted into the simulator.

## Verified component law and power path
`cassette_spring_law.py` exposes the two-spring energy/force and +/-10% rate variation. It leaves NOTE normal and stop reactions unknown. Nominal free-length tolerance remains unspecified; the source approximate wire diameter is not a guaranteed bore tolerance. The historical REV-11-1144 SPM provides6V from robot12V distribution, with9V minimum startup and15A total-output shutdown. No regulator efficiency, two-servo current trace, electrical mount or harness is yet adopted. Four component-law tests check nominal values, energy gradient, parameter limits and unknown contact/stop reactions.

Servo drawing review exposed a concrete interface gap: the2017 drawing provides overall dimensions and R2.25 ear slots, but not all mounting-hole centers/output-axis offset. The matching25T REV-41-1363 horn drawing is pinned, but exact fastening/shaft registration needs native source or separately supported datums. Do not fabricate these dimensions or label the coupon a mounted actuator.
