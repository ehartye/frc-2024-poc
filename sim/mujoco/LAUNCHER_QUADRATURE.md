# Launcher contact quadrature and continuous-cycle boundary

The earlier [launcher experiment](NATIVE_LAUNCHER_CONTACT.md) rejected shooting prediction because increasing contact resolution changed release. A separate, fixed solver-law hypothesis now passes the declared force, phase, timestep and trajectory checks for staged release, including simultaneous drivetrain demand. **Numerical convergence is not native-mechanism or hardware validation.** Acquisition, moving lift/pitch and transfer remain unconnected.

The native authority is unchanged: the private 734-body Fusion candidate with SHA-256 `5f2a1a4cb98931f31575444e95323c9cb78bab8df6d74f226c34dc53bd62bf96`. No live Fusion interaction or native mutation occurred. Original failed runs remain preserved and keep their original verdicts.

## What changed and why

Prescribed-state probes first separated contact response from trajectory divergence. At the same staged pose, all old variants fit 25 N/wheel at rest, but a 3 m/s tread speed produces roughly 80-350 N/wheel depending on contact resolution. An exact-duplicate control then showed that copying identical contacts changes the original force response; this is not a geometry-refinement effect.

For constant contact impedance `d`, MuJoCo documents a regularizer `R = (1-d)/d * Ahat` and direct reference-acceleration coefficients `k=K/d`, `b=B/d`. Duplicating each contact `q` times while preserving the aggregate solve requires `R_new=q*R` and unchanged reference acceleration. The resulting parameter mapping is:

```text
d_new = d / (d + q*(1-d))
K_new = K * d_new/d
B_new = B * d_new/d
```

This mapping is a derivation from the [documented regularizer](https://mujoco.readthedocs.io/en/stable/computation/#parameters) and [direct solver parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#reference), not a hardware constitutive law. Exact duplicate controls at 2x, 4x and 8x preserve generalized forces within 4.49e-13 N/N m and accelerations within 5.46e-12 in their corresponding units. Without weighting, the same controls change NOTE force components by 8.08-35.62 N.

`native_launcher_quadrature.py` extends that mapping to uniform angular ring samples with `q=segments/128`. Weights are fixed by total sampling resolution, never renormalized by the active contact count. The reference coefficients remain `K=4350.219346 s^-2`, `B=131.912385 s^-1`, `d=0.9`; optional NoSlip is disabled. These are acceleration-reference gains, not spring stiffness in N/m or damping in N s/m. The new hypothesis is recorded separately from the old per-resolution force/damping fits.

## Numerical evidence

Thresholds were recorded before the trajectory experiments. The initial 2048/4096-segment and phase force checks failed. Higher resolution retained exactly the same material hypothesis and thresholds. At 8192/16384 segments, the sampled force comparison passes; the 16384-segment half-cell phase check also passes. The worst normalized error is 0.653 of the allowed threshold: 1% with a 0.1 N force floor and 0.01 N m torque floor. The 8192-segment phase check remains a recorded failure.

The experiments preserve 525 original-law pose/velocity probes, 140 exact-duplicate controls, 600 initial weighted probes and 375 refined weighted probes. These are prescribed states, not acquisition trajectories.

The staged launch starts from rest with finite motor torque, a free six-DOF NOTE and no attachment or state reset after initialization. Four launcher rotors, six-wheel drivetrain and the existing shared resistive supply evolve in one MuJoCo 4.0.1 solve. The fine-grid initial normal load is **about 16.045 N/wheel**, not the old 25 N coarse calibration target; it was not retuned to obtain release.

| Comparison against 16384 segments | Maximum NOTE position difference | Maximum NOTE velocity difference |
|---|---:|---:|
| 8192 segments | 0.0402 mm | 0.000481 m/s |
| Half timestep, 0.05 ms | 0.124 mm | 0.00183 m/s |
| Half-cell ring phase | 0.0182 mm | 0.000344 m/s |

All are below the predeclared 1 mm / 0.01 m/s limits. Last-contact speed is 3.18392 m/s at the fine baseline and 3.18829 m/s at half timestep; last-contact time differs by 0.1 ms. NOTE work residual drops from 0.002709 to 0.001354 J. These are outputs of an uncalibrated model, not predicted hardware shot speeds.

Simultaneous drive demand of 0.15 moves the chassis 20.861 mm during the local launch and gives 3.25359 m/s last-contact speed. It shares supply load with the launcher, reaching 9.32460 V minimum bus voltage. Its independent spatial and timestep comparisons are included in the verifier. Zero launcher drive and zero friction do not produce forward release; no-contact motion matches discrete free fall, and the floating internal-actuation control conserves system COM. The verifier also checks source/model/trace hashes, force projection, friction cones, current limits, electrical identities and post-contact free flight. It rechecks the 27 preserved earlier cases alongside the new runs without changing their conclusions.

The high-resolution checks are slow offline evidence. A 0.14 s fine launch took roughly 53 s of wall time in the recorded run; no real-time or interactive readiness is claimed.

## Native clearance and the missing continuous path

Fresh offline bare-wheel-envelope checks cover 282 stationary-launch poses / 602,248 distance queries and 285 simultaneous-drive poses / 603,857 queries. Minimum containing-envelope gaps are 0.392760 and 0.402071 mm, respectively. These finite witnesses are conditional on configured placement and do not clear the full robot or the intervals between samples. The refined stationary trace still leaves eleven other native hard-part bounds unknown.

`check_native_cycle_handoff.py` now tests whether the preserved retention result can be composed with the launcher start. The identity control passes; the actual splice is rejected. Even after allowing a rigid change of world frame, NOTE position would jump **350.726 mm**, its axis would rotate about **60 degrees**, and the joint sets differ. The missing NOTE gravitational potential change alone is **0.782431 J**. Neither assigning a new pose nor restarting the launcher constitutes a simulated lift/transfer.

The path forward requires one attained acquisition state and one combined mechanism model that retains NOTE/chassis pose, velocity, motor states, time and energy through transport, lift/pitch and feed. Current short straight belt patches cannot cover that route. Native articulation/installed clearance remains unresolved while live-document inspection awaits permission. The historical three-second acquisition result is a horizon diagnostic, not a possession checkpoint; it must not be imported as captured.

## Calibration boundary and reproduction

No NOTE/tread force-compression, unloading hysteresis, slip-versus-load/speed, worn-piece behavior or temperature data were supplied or measured for this model. Actual installed geometry, rotor inertia, wheel mass, motor constants and supply behavior also need independent confirmation. Measure these separately and validate held-out states/trajectories before treating any fitted simulator output as robot performance. A 25 N tuning target, a nominal diameter or Shore hardness cannot substitute for that evidence.

The runner retains the previous native/source hash checks. Use explicit fine resolution and a fresh private output directory:

```powershell
python -B sim/mujoco/native_launcher_quadrature.py --project . --binding sim/mujoco/evidence/native-launcher-binding-20261118.json --native-archive $NativeArchive --contract sim/mujoco/evidence/launcher-quadrature-trajectory-contract-20261118.json --segments 16384 --output $NewRunDirectory
```

Add `--drive-duty 0.15` for simultaneous drive demand. Add `--dt 0.00005` or `--ring-phase-deg 0.010986328125` for the checked refinements. The initial 128-segment default is only a coarse exploratory case; the numerical gate applies to the explicitly reported fine runs.

```powershell
python -B sim/mujoco/verify_launcher_quadrature.py --packet $PrivateSeasonRoot --project . --manifest "$PrivateQuadraturePacket/combined-manifest.json" --binding sim/mujoco/evidence/native-launcher-binding-20261118.json --output $NewVerificationFile
```

The combined manifest references the preserved original packet and the separate quadrature packet under the same private season root. It prevents silently replacing historical producers. `numericalGatePassed` is deliberately separate from `acceptance`, `hardwareValidated` and `continuousCycle`, which remain false. Supplier CAD, raw states and native archives remain private.
