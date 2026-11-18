# Predeclared braking-boundary continuation

Continue the same nominal controller from the source-bound .862 s state. Replay its original ten controller ticks and the following twenty ticks with exact command equality; restore the unchanged plant checkpoint. Current drivetrain commands already request dissipative braking. No momentum reset, new policy, pose snap or geometry extension is introduced.

Predeclared horizon: 50 ms or first unsupported query, 1 ms step; a 20 ms prefix may be used first. The .862 s state has relative NOTE X187.0373 mm, chassis speed0.202561 m/s and only9.0373 mm to the178 mm domain edge. A nearby position is not capture or stopping. Preserve the first domain counterexample and atomic state rejection. A separately run0.5 ms refinement uses the identical start state, not a full-run convergence claim.

Run: `python sim/pressure/run_braking_boundary.py artifacts/new-boundary.json nominal .001 .05`.

## Frozen outcome
Both steps stop at 0.910 s on the original 178 mm experimental guard, not collision. Chassis speed remains 0.161529 m/s. Half-step endpoint relative position differs by 5.88e-8 m; new-phase energy residual decreases from 2.606e-6 to 6.507e-7 J. No stopped/capture claim. The next 138 mm experiment is a separate domain, with rebuilt full-obstacle prefilter.
