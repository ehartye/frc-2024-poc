# Transverse NOTE / cassette / cam coupling fixture

This additive mechanical experiment joins the sampled NOTE pressure law to the independently moving positive-side cassette and cam. It is **not robot acquisition, lift, capture or hardware acceptance**. The negative-side cassette is absent. The NOTE is free only along Y: its X/Z and orientation are ideal fixed constraints. Their reaction forces and moments are reported; work is zero because the constrained velocities are zero, not because reactions vanish. The cradle is fixed.

## Coordinates and energy

The state contains NOTE `y, ydot`, cassette `u, udot` (positive inward, along negative Y), actual cam `theta, thetadot`, time, external actuator work and cam damping heat. No angle command becomes actual position. The cam, component spring and material-area pressure potentials are separate. Actual pressure is evaluated at every RK4 stage and endpoint; the cassette receives its reciprocal generalized force. The component spring's force is never substituted for NOTE normal pressure.

Only normal pressure is included: `mu=0`, zero belt speed and no hidden rotor/electrical state. The interface does not model feed friction, ground, gravity or moving-base reactions. An optional positive-side contact-cap or wrap reaction in constrained directions remains in the reported constraint load. Moving stop impacts reject atomically; static stop reactions can balance force at zero velocity without work. No impulse, clipping or ideal attachment is introduced.

Explicit hypotheses: NOTE mass0.234925kg, cassette translating mass1.5kg, cam output-referred inertia1e-4kg m², pressure density1e7N/m³, cam stiffness1e6N/m and damping10N s/m, external mechanical torque bounded±0.03N m. These are not a calibrated servo electrical/control/thermal model. The source-backed nominal component spring law is reused; actual spring tolerance, material contact and installed moving mass/inertia remain unqualified.

## Initialization and controls

The fixture starts with declared stored spring/pressure/cam energy. It is not a handoff from the ground-acquisition checkpoint. The `holding_input` torque is the isolated no-NOTE cam equilibrium torque at0.2rad; applying it in the presence of the dynamic NOTE does not assert equilibrium or impose cam position.

Five 2ms runs use the same1024×1024 material grid: holding input at0.5/0.25ms steps, torque removed at0.5ms, zero spring preload at0.5/0.25ms. Zero preload means the component spring is uncompressed, not that existing geometric NOTE compression disappears. Pressure still acts. Fixed-grid512/1024 and quarter-cell phase comparisons are diagnostic controls, not a spatial-convergence certificate. Numerical yaw constraint torque from grid phase is explicitly not physical material torque.

The one-side broadphase uses padded AABBs only; retained points are recomputed through the original scalar material law and in original grid order. The absent negative side is omitted **before** querying contact. Filtering it after a two-sided query would wrongly let a nonexistent wall reject a separated one-wall configuration. Unit tests compare every retained positive-side material witness against uncullled scalar evaluation at several phases.

## Reproduce

```
python -m unittest discover -s tests -p 'test_cassette_pressure*.py' -v
python sim/pressure/run_cassette_coupling.py artifacts/fresh-cassette-pressure.json
```

Fresh output paths are required. The runner checks source hashes before and after; its report records the actual mechanical and pressure dependencies plus the source-bound cam geometry inventory. Vendor native CAD is neither needed nor redistributed. Comparison receipts remain unaccepted hypotheses; no current/voltage demand, latch or sustained-holding approval follows from their small energy residual.
