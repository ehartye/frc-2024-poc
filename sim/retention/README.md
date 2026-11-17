# Finite contact-patch retention requirements

This is a constructive friction-wrench feasibility calculation, not a rigidly attached NOTE simulation. The source-bound requirement run uses verified nominal patch witnesses alongside synthetic negative controls; neither is a physical retention validation.

Each side's hypothesized normal load is distributed equally among its selected point witnesses. At each point, planar tangential force is bounded by mu times that normal share. A weighted minimum-norm force allocation is constructed for translation and pitch torque. A construction inside every friction circle is a conditional feasible allocation. If a necessary total force or torque bound is exceeded, that force/preload hypothesis is insufficient. If the construction fails but necessary bounds do not, the result is unresolved, NOT a proof of infeasibility. Lateral/roll/yaw imbalance is also unresolved rather than silently discarded.

Desired pitch and lift use explicitly prescribed quintic trajectories only to calculate required forces/torque. Passing a wrench screen would not prove the NOTE follows those trajectories. No pressure distribution, compliance, slip evolution, lateral recentering or physical retention is established. A collapsed point-contact control cannot transmit pitch torque; loss of one side cannot be accepted by this planar model.

## Source-bound requirement cases

Run `python sim/retention/run_requirements.py --output artifacts/a-fresh-retention-requirement-run`.

The saved28cases use30 native-authored belt-face/NOTE-interior witnesses per case (15per side), longitudinal offsets0,9.88835038313412,10,20mm and seven load/motion hypotheses. The exact9.888mm offset is checked against the recorded ground-dwell state; it is not silently reset to center. Uniform-solid-torus inertia and hypothetical10N/side,mu.6 remain assumptions, not measured parameters.

At each offset, the slower1s pitch plus1.2s lift requirement has a constructive allocation at all441 sampled times. The faster.15s pitch exceeds necessary bounds at34samples; the.5s pitch scenario has23Ã¢â‚¬â€œ25 unresolved allocations and is not declared impossible. Lower preload/friction also expose bound violations. Collapsing the patch to points removes pitch torque capability; losing one side produces unresolved lateral/roll/yaw balance, not a planar pass.

This remains a required-wrench analysis. It does NOT execute a retained-piece trajectory or establish finite-slip/contact persistence. Active belt hold current estimates are ideal1:1 reciprocal-DC requirements only. Power-off along-belt static support is not assumed. The real bearing/flange representation andÃ‚Â±5mm offset source checks remain separate design gates.

## Free planar finite-shear follow-up

`python sim/retention/finite_slip.py --output artifacts/a-fresh-finite-slip-run` integrates NOTE translation and pitch, finite elastic/plastic shear patches and optional belt-rotor backdrive. It starts from the recorded ground-dwell longitudinal position and near-zero velocity. It does not constrain the NOTE to follow the prescribed cradle. Source geometry/configuration registration and ground-state lineage are checked.

Under the explicit locked-belt,10N/side,2000N/m-side shear hypothesis, the NOTE stays within the nominal footprint witness domain for the2.2s prescribed motion. The final NOTE pitch is about58.88degrees rather than the cradle's60degrees: no pose reset or perfect attachment. The coast case leaves the chosen footprint domain at.2317s, fast pitch at.0166s, and2N/side preload at.1188s. These are limits of the fixed-footprint model, NOT verified physical drops or safety failures; contact could migrate and needs a richer model beyond this boundary.

Ground support is a frictionless lowest-point penalty approximation with about.165mm maximum nominal penetration. Energy residual decreases from.177mJ to.082mJ to.042mJ at100/50/25microsecond steps. Final pitch differences are not monotonically converged, so trajectory convergence remains open despite improving energy balance. All pressure/stiffness/friction and actuator-hold assumptions remain uncalibrated. Exact bearing/flange and lateral-entry checks remain separate open geometry work.

Do not connect this result to the old aligned handoff by snapping NOTE attitude, velocity or center. The actual final free-body state requires fresh transfer/contact registration. Seven regressions preserve source closure, free motion, backdrive, domain exits, energy/refinement and invalid inputs.
