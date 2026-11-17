# Short eight-motor lift-phase continuation

This is a bounded10ms continuation of the actual indexed acquisition checkpoint, not an achieved lift, ideal attachment, successful retention or scoring cycle. The coordinator's patch/guide APIs and all earlier models remain unchanged.

## State and mass continuity

The original chassis XY/yaw/wheel state, free NOTE XY/yaw state, pinch rotor momentum, clock and cumulative energy ledger are copied exactly. The flat-floor predecessor supplies initial NOTE/pivot Z=.0254m, pitch=0, vertical/pitch rates=0; these are preceding constraints, not an aligned-pose reset. Retention-positive pinch rotors and commands negate the acquisition convention; the acquisition-sign rotor field remains available. No elastic shear history is invented.

Mechanism7kg is partitioned OUT of the old45kg robot hypothesis: base38kg plus4kg translating carriage and3kg rotating assembly. The full `(baseX,zPivot,theta)` actuator mass matrix and bias are combined with base mass, tire forces, finite-contact generalized loads and finite motor torques. Rotor inertia is counted once. Mechanism gravity is referenced to its initial potential. Initial mechanical energy agrees with the predecessor to2.17e-16J. The previous normal-spring energy remains explicitly in the total as a dormant reservoir while the fixed side-normal model does negligible lateral work and witnesses remain supported.

The narrow symmetric XZ scope retains and advances actual tiny lateral/yaw states and energies; it does not reset them. Nonplanar force/torque terms are reported, with a1e-8N/Nm rejection bound. A1e-8m/s or rad/s preflight rejects unsupported nonplanar velocities, and side-normal work is bounded by1e-10W. Absolute chassis heading and relative NOTE yaw must also remain within1e-8rad of the declared longitudinal plane; a stationary yawed state is rejected. These are numerical-domain limits, not physical tolerances. This is not a general asymmetric3D dynamics model; larger nonplanar behavior stops rather than being silently projected away.

## Synchronous mechanics and power

One pure shared-bus query includes four CIM drive motors, two pinch motors, one lift NEO and one pitch NEO. A synchronous explicit midpoint advances chassis, free NOTE, actuator and rotor states together. The old implicit finite-holding step is NOT independently advanced. Its reciprocal electrical function is reused; memoryless friction is evaluated directly at the shared state. Current limits and signed power remain explicit. Unsupported controller voltages and contact/geometry domains reject the step atomically.

Thirty registered belt-face witnesses are re-evaluated at start, midpoint and accepted end using the actual transforms. The normal budget remains1.755N per side, divided across15 points, with one two-component friction-circle budget per side. Tangential forces and rotor reaction torques are equal/opposite; cradle generalized loads use pivot-relative levers, while free-NOTE torques use NOTE-relative levers. No increased preload or infinite locked belt is substituted.

The old flat constraint becomes an explicitly new compliant floor model:16 torus-ring quadrature contacts, total20kN/m spring stiffness and20Ns/m compression-only damping, plus regularized friction using the prior0.4 coefficient. Initial overlap/spring energy is zero. Floor spring energy, damping and friction are in the ledger. This is a constitutive extension, not measured carpet/foam behavior. Quadrature and timestep limits remain provisional.

The guide uses the coordinator's finite geometry and wrench APIs, with actual chassis velocity and reciprocal work. Initial guide gap is10.853mm, so initial guide elastic energy is zero. The guide is still an unadopted, materially UNKNOWN candidate. The archived20-degree disagreement and unsupported guide geometry are retained as UNKNOWN. The new phase does not clear other whole-robot collision geometry or shaft/chain hardware.

Static gravity feedforward is only a finite initial duty command. It does not enforce z or theta, cancel contact loads, or reproduce old quintic pose paths. Source drive/pinch demands are held for this short experiment. Tire normal loads retain the prior prescribed symmetric distribution; dynamic load transfer and end stops are not solved.

## Run

Python with the existing NumPy dependency:

```sh
python -m unittest discover -s tests -p test_coupled_lift_phase.py -v
python sim/integrated/coupled_lift_run.py artifacts/fresh-lift-phase.json
```

The runner refuses existing output, validates all checkpoint/registration/actuator/holding/guide sources, and checks input hashes again after execution. Evidence is new; no historical receipt is relabeled. Every failed step preserves state and cumulative ledger.

## Recorded outcomes

Gravity-feedforward completes10ms. The cradle sags2.308micrometers and pitches-3.338microradians; the free NOTE drops74.650micrometers rather than attaching. Half/quarter-step cradle sag tends toward2.274micrometers. Incremental energy residual above the inherited0.9447mJ residual refines from2.94microjoules to0.756 to0.182. The full cumulative residual is not falsely reported as zero.

With actuators coasting, one1ms step is accepted, then the next proposal stops at UNKNOWN_FINITE_GUIDE_GEOMETRY. That outcome is retained, not tuned into a successful lift or reinterpreted as physical collision. Nine tests cover exact checkpoint continuity, mass/energy partition, eight-motor power, instantaneous work balance, actual retained tiny yaw/lateral state, finite patch and20-degree guide gates, atomic rejection and refinement. Longer continuation, contact robustness, actual actuation calibration and mission-level success remain future work.

The coast diagnostic's rejected guide input is centerRobotXZ=(0.188563416709,0.025391659481)m and pitch=-7.0392e-9rad. The API returns UNKNOWN because its supported attitude interval begins at zero. This is an unsupported negative-pitch input, not a verified guide collision; no clearance inference or domain relaxation is applied.
