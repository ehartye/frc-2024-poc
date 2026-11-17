# Unadopted spring and opening-cam candidate

Lee Spring LC 032EE 01 M is a concrete music-wire candidate, not an arbitrary foam stiffness. The manufacturer-indexed catalog table supplies the dimensions/rate recorded in `cassette-opening-source-candidate.json`; direct PDF retrieval returned 403/404, so source-byte pinning and historical catalog verification remain open. Nominal spring inside diameter is 7.91 mm and fits the 6 mm rod geometrically. This does not verify tolerances, allowable working deflection or fatigue.

Two nominal springs supply 14.6234 N at 7 mm installed length and 5.9534 N at 8.5 mm. These are cassette spring forces, NOT measured NOTE normal forces. Ideal opening work is 0.0154326 J before guide friction, moving inertia and losses. Seat adjustment is not used in this first calculation.

`cassette_opening_cam.py` generates editable original eccentric cam/follower coupons at open/mid/released positions. The 0.75 mm eccentricity gives 1.5 mm displacement. The contact equality is only a unilateral-contact geometry constraint, not an achieved motion command. Three isolated coupon pairs have no material overlap and each STEP reopens with two roots; this is not full-robot fit evidence.

REV SRS V1 REV-41-1097 is a historically documented actuator candidate (2017 drawing), not installed or accepted. Its source stall torque/current must not become continuous available torque. A compatible horn, bearing-supported shaft, actual servo bracket, follower attachment, power module, position detection and positive open-position latch remain to be designed. An eccentric at dead center is NOT a latch: perturbation can release stored spring energy. Opening during transfer still needs actual force handoff; it cannot restore zero travel instantaneously or guarantee retained NOTE support.

Canonical robot geometry and reviewed cassette study remain unchanged. No spring force or cam actuation has been adopted into the simulator.
