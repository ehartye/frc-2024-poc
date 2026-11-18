# Free planar acquisition experiment (work in progress)

`free_planar.py` is an additive two-motor / shared-bus kernel. It does not load or waive CAD geometry itself. A callback receives robot-relative NOTE center `[x,y]` in metres and must return `status: SUPPORTED` plus one contact at most per belt/motor, or an explicit unsupported-domain status. Contacts carry penetration, orthonormal normal/tangent vectors and the actual contact-point lever in metres. Unsupported midpoint or endpoint geometry stops before advancing to that state.

The rigid flat NOTE has free world XY velocity and yaw rate. Yaw does not change the geometry of a rotationally symmetric torus, but it changes contact-point slip and torque. Prescribed robot translation contributes separate boundary work. Two reciprocal-DC motor surrogates share a resistive source; duty zero is open circuit and does not erase rotor or NOTE momentum. Negative motor electrical power outside this nonregenerative model stops as unknown.

## Explicit hypotheses

Normal force is `k delta (1 + alpha max(-vn,0))`, with compression-only damping continuous at contact onset. It is an exploratory compliance law, not a calibrated foam/belt material. Normal spring energy is `k deltaÂ²/2` and damping loss is `alpha k delta max(-vn,0)Â²`. Tangential force is regularized Coulomb friction, `mu N tanh(slip/vreg)`, with equal rotor reaction and NOTE torque at the selected material contact point. No geometric overlap is silently reclassified as acceptable.

Ground contact holds the flat NOTE at fixed height, with four equal normal-load quadrature points on the major ring. Each point has dissipative regularized Coulomb friction. This approximates sliding/yaw friction, not measured carpet or pressure distribution. Yaw inertia is a declared uniform-solid-torus approximation. No pitching, lift, rolling, deformation, capture sensor or scoring is simulated.

The midpoint integrator carries position, velocity, yaw, rotors, time and energy. Electrical source, winding/source losses, ground/slip/normal damping losses and prescribed-base work are separate. Small energy residual does not establish trajectory convergence, so mirrored response and half-step controls also compare state.

## Synthetic checks so far

Eight kernel tests cover force-free translation/spin, ground dissipation, mirrored lateral response, half-step state/energy convergence, unsupported domain, zero-duty persistent rotor momentum and invalid inputs. In the deliberately synthetic parallel-wall case starting at y=4 mm, the 50 ms endpoint is y=1.600424 mm at dt100 microseconds versus1.600300 mm at50 microseconds. Maximum energy residual reduces from5.78 to1.43 microjoules. These are not results for the actual intake geometry.

Actual source-bound finite belt contact and conservative hard-obstacle gates are pending the designer's immutable geometry contract. The existing Â±5 mm rigid-offset collision evidence remains unchanged and is not a continuous clearance certificate. No actual recentering/capture claim is made from synthetic tests.


## Actual source-bound result

Run from the repository with the existing Python/OCP environment:

```text
python sim/acquisition/run_free_planar.py --obstacles sim/acquisition/free-obstacles.json --output artifacts/free-planar-new.json
python -m unittest discover -s tests -p "test_free*.py" -v
python -m unittest discover -s tests -p test_obstacle_gate.py -v
```

Output must be fresh. The committed `free-planar-evidence.json` contains14 cases, full sampled traces, final valid states, rejected midpoint/endpoint states, separate energy terms, step-sampled force/penetration peaks and complete input hashes. No NOTE state is advanced through an unsupported candidate.

The source geometry uses the exact finite smooth capsule belts, including their round end wraps. Within the strictly coplanar/outside-major-ring domain, an analytic torus-to-capsule witness gives compression and the selected contact-point lever. One resultant per belt remains an explicit pressure-distribution hypothesis.

Conservative obstacles cover711 original assembly parts with744 bounds. Only the two designated contact belts are excluded. Disconnected solids are bounded separately. The cradle weldments use the exact authored pre-fuse stock, carrier, tie and web constituent bounds; native sequential Boolean subtraction leaves zero solids for both actual weldments. The boxes are padded1e-6mm. Track pulley bands retain configured radii/heights; eight track-bearing envelopes use the supplied native outer-size correction at the existing, unseated centers. Pivot bearings retain unresolved source-sized representations and are not mislabeled as selected REV parts.

Known vertical pulley/bearing bands use the exact distance from the flat major ring to a filled finite cylinder; bores stay conservatively filled. Other parts retain boxes. A positive candidate is UNKNOWN, not proof of material collision. AABB-only control remains in the matrix because connected/curved bounds can otherwise obscure useful steering behavior.

| Case | Last valid robot X, mm | Last valid lateral Y, mm | Peak step normal, N |
|---|---:|---:|---:|
| Centered,2kN/m |178.014|0|1.759|
| +5mm,2kN/m |366.308|4.833|2.100|
| +5mm,1kN/m |366.823|4.947|1.094|
| +5mm,4kN/m |337.187|0.521|4.638|
| +5mm,8kN/m |333.416|−0.303|9.521|
| +1mm,2kN/m |333.302|−0.300|2.014|
| +2.5mm,2kN/m |338.719|−0.640|2.138|
| +5mm,zero belt duty |366.519|4.878|2.098|
| +5mm,AABB-only control |378.720|5.000|0|

Centered motion reaches the rear experiment boundary, not capture/retention. Every offset case stops at a conservative pulley-profile candidate. Increased stiffness changes recentering and can produce an opposite-side overshoot; it is not a demonstrated fix. Ground friction and zero-duty controls retain their own results. Penetrations in the refined-contact matrix peak below0.953mm as a compliance hypothesis, not measured material deformation.

At2kN/m, halving100µs to50µs changes the +5mm lateral endpoint by approximately4.8nm. At8kN/m the change is0.144µm. Maximum energy residual is approximately5.2µJ at100µs and1.34µJ or less at50µs. These bounded comparisons do not establish hardware calibration or general convergence.

Pulley angular phase is not integrated/registered to vendor tooth geometry. A later native witness at one clocking can be diagnostic only, not clearance over all rotating-tooth phases. Bearing seating changes are not incorporated; geometry receipts must be regenerated when they are adopted. No contact exemption, acquisition sensor, stopping/hold criterion, lift transition or scoring event is manufactured here.
