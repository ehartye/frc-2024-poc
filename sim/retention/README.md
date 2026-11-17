# Finite contact-patch retention requirements

This is a constructive friction-wrench feasibility calculation, not a rigidly attached NOTE simulation. The source-bound requirement run uses verified nominal patch witnesses alongside synthetic negative controls; neither is a physical retention validation.

Each side's hypothesized normal load is distributed equally among its selected point witnesses. At each point, planar tangential force is bounded by mu times that normal share. A weighted minimum-norm force allocation is constructed for translation and pitch torque. A construction inside every friction circle is a conditional feasible allocation. If a necessary total force or torque bound is exceeded, that force/preload hypothesis is insufficient. If the construction fails but necessary bounds do not, the result is unresolved, NOT a proof of infeasibility. Lateral/roll/yaw imbalance is also unresolved rather than silently discarded.

Desired pitch and lift use explicitly prescribed quintic trajectories only to calculate required forces/torque. Passing a wrench screen would not prove the NOTE follows those trajectories. No pressure distribution, compliance, slip evolution, lateral recentering or physical retention is established. A collapsed point-contact control cannot transmit pitch torque; loss of one side cannot be accepted by this planar model.

## Source-bound requirement cases

Run `python sim/retention/run_requirements.py --output artifacts/a-fresh-retention-requirement-run`.

The saved28cases use30 native-authored belt-face/NOTE-interior witnesses per case (15per side), longitudinal offsets0,9.88835038313412,10,20mm and seven load/motion hypotheses. The exact9.888mm offset is checked against the recorded ground-dwell state; it is not silently reset to center. Uniform-solid-torus inertia and hypothetical10N/side,mu.6 remain assumptions, not measured parameters.

At each offset, the slower1s pitch plus1.2s lift requirement has a constructive allocation at all441 sampled times. The faster.15s pitch exceeds necessary bounds at34samples; the.5s pitch scenario has23â€“25 unresolved allocations and is not declared impossible. Lower preload/friction also expose bound violations. Collapsing the patch to points removes pitch torque capability; losing one side produces unresolved lateral/roll/yaw balance, not a planar pass.

This remains a required-wrench analysis. It does NOT execute a retained-piece trajectory or establish finite-slip/contact persistence. Active belt hold current estimates are ideal1:1 reciprocal-DC requirements only. Power-off along-belt static support is not assumed. The real bearing/flange representation andÂ±5mm offset source checks remain separate design gates.
