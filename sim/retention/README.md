# Finite contact-patch retention requirements

This is a constructive friction-wrench feasibility calculation, not a rigidly attached NOTE simulation. Native source-bound patch witnesses are required before running a real geometry case. `wrench.py` currently has synthetic-point regression tests only.

Each side's hypothesized normal load is distributed equally among its selected point witnesses. At each point, planar tangential force is bounded by mu times that normal share. A weighted minimum-norm force allocation is constructed for translation and pitch torque. A construction inside every friction circle is a conditional feasible allocation. If a necessary total force or torque bound is exceeded, that force/preload hypothesis is insufficient. If the construction fails but necessary bounds do not, the result is unresolved, NOT a proof of infeasibility. Lateral/roll/yaw imbalance is also unresolved rather than silently discarded.

Desired pitch and lift use explicitly prescribed quintic trajectories only to calculate required forces/torque. Passing a wrench screen would not prove the NOTE follows those trajectories. No pressure distribution, compliance, slip evolution, lateral recentering or physical retention is established. A collapsed point-contact control cannot transmit pitch torque; loss of one side cannot be accepted by this planar model.
