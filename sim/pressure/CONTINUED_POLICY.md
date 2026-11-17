# Continued early braking (source-only checkpoint)

This additive runner starts at the exact .662 s accepted endpoint of each previously archived early-policy case. It restores the pressure checkpoint without changing physical state, time, rotor momentum or cumulative energy, and reconstructs the controller by replaying all ten archived observations and checking every command. This preserves the stale-observation fault latch. It does not initialize a new policy at .662 s.

Predeclared extension: nominal, stale and pinch-disabled cases for up to 200 ms, or the first unsupported domain query, at 1 ms; nominal 0.5 ms is the refinement control. A 20 ms bounded prefix can be run first for resource checks. Results are written only after exact source-before/after verification; no result or capture is claimed by this source-only commit.

Run one case at a time: `python sim/pressure/run_early_continuation.py artifacts/early-continued-20ms.json nominal .001 .02`. The initial 20 ms costs approximately 25 seconds based on earlier fine-grid runs; no guarantee is made. Full 200 ms may cost several minutes and can stop at the inherited geometric domain. The only live controller receives exact virtual observations. There is no ideal hold, attachment, spring adoption or velocity reset.

## First bounded result

The nominal 1 ms run completed the full .662 to .862 s extension without a domain stop. Its first 20 states and controller events exactly reproduce the separately saved 20 ms run. At .862 s, relative NOTE X is 187.0373 mm, only 9.0373 mm above the inherited 178 mm domain boundary. Chassis speed remains .202561 m/s; NOTE world velocity is -.0151164 m/s and rotor speed18.2479 rad/s. Thus a position close to the185 mm goal is not a stopping, capture or lift-ready result. The new phase energy residual is39.2144 microJ in addition to the inherited1.58363 mJ; this extension has not yet been timestep-refined. The model and policy were not tuned during the run. Further travel requires a separately declared continuation and preservation of any domain stop, not snapping the NOTE to the goal.
