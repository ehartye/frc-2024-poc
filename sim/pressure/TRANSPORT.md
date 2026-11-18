# Finite belt transport query interface

`finite_transport.py` is additive to the reviewed standalone pressure law. It permits shallow NOTE-facing outer end wraps using the exact source capsule gradient and source acquisition tangent, and preserves cavity/deep/medial-tie failures. The same finite-height cap law remains explicit. No existing pressure receipt is relabeled.

`prepare` returns material-area normal contacts and potential, without velocities or a battery. `rates` consumes actual body/cradle velocities, positive-cassette u velocity and two signed rotor velocities. Acquisition-positive rotor speed drives rearward on straight inner faces. Belt speed is the inherited pitch-radius-times-rotor-speed hypothesis; outer-layer strain around wraps is not calibrated. Tangential friction uses a two-component regularized Coulomb law with one per-cell budget. The returned rotor reaction is `-radius * dot(frictionOnNote, acquisitionTangent)`.

Body, cradle, cassette and rotor work plus normal-potential rate and nonnegative slip heat close as one ledger. Equal/opposite forces act at a common penetrating material sample, an explicit compliant-contact convention. There is no tangential stored history, force redistribution, hidden supply, motor state reset, spring equilibrium or opening-actuator claim.

Eight tests check source wrap gradients, unchanged straight pressure, tangent/torque sign, moving-body/cradle/u work, zero-friction/zero-slip controls, pure queries and unsupported/overflow domains. Nine source-bound static/rate fixtures are recorded, not a time trajectory. At512x512 the entry fixture resolves208 contacts:60 wraps and148 straight faces.128x128 misses the wrap feature in that fixture; it must not be used silently for dynamic integration. Work residual is below2e-15 W for the retained static cases, which does not establish physical accuracy or timestep convergence.

```text
python -m unittest discover -s tests -p test_finite_pressure_transport.py -v
python -m unittest discover -s tests -p test_pressure_transport_receipt.py -v
python sim/pressure/run_transport.py artifacts/fresh-transport.json
```

Next: scalar-parity-preserving cached-grid culling, then fresh coupled acquisition from a genuinely no-contact initial state under the new law. The old checkpoint's model-change energy is not reset or imported as continuous state.
