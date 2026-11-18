# Continuous acquisition to the first transport boundary

The inherited pressure model now reaches its declared ground-index condition at **4.440 s**, then attempts transport from that exact state. The NOTE is never assigned a new pose, velocity, attachment or energy reference. The short transport attempt exceeds the 0.010 m/s relative-speed criterion after **17 ms** at all three local timesteps. This is a bounded controller/model result; full loading, native registration, lift/feed and shooting remain incomplete.

## Attained state and numerical evidence

The preserved fresh-rest run ends at 3 s with the NOTE still approaching the goal. Its exact source/configuration pins, physical state, cumulative ledger, controller history, held command and next absolute controller tick are restored. The continuation reaches the 0.180-0.190 m index band with both relative and chassis planar speed at most 0.003 m/s. The continuous dwell begins at 4.337 s; the first eligible 100 Hz transition tick is 4.440 s.

At that tick, relative X is 0.188559790 m, relative longitudinal speed is -0.002679247 m/s and chassis forward speed is 0.002160517 m/s. The complete joined history contains 4,440 accepted 1 ms steps. Its maximum cumulative energy residual is 0.000950965 J. Ground support remains prescribed. This observed index is not a lifted-possession result.

A conservative tiled broadphase accelerates the existing pressure geometry. It retains the same 1024-square material grid, phase, scalar transforms, contact evaluation and summation order. All six existing reference-parity tests pass, including random frames, near-unit transforms, unsupported contacts and extreme coordinates. Ten additional prescribed poses preserve exact dictionaries or exceptions. A 20-step continuation prefix and a separate 100-step resume interval preserve **every physical row and command exactly**. The redundant original process was deliberately stopped only after saving its checkpoint and confirming the 100-step match. A less useful annular-culling attempt is preserved privately and was not adopted.

Typical recorded warm geometry evaluations fall from about 0.285 s to 0.106 s. The resumed 0.94 s trajectory still takes roughly 501 s wall time. This is offline evidence, not a live-control demonstration.

## Transport from the attained index

The new transport controller is frozen before execution: request 0.10 m/s chassis speed using the existing back-EMF feedforward, a 2 V/(m/s) speed correction and 0.2 duty cap. Pinch effort retains the positive-only 0.15 duty/m index correction toward 0.185 m, capped at 0.025, with zero duty coasting. Commands change at the existing 100 Hz tick. State and cumulative energy remain continuous; there is no unscored settling interval.

| Timestep | Time from transition to first speed violation | Chassis travel | NOTE world travel | Relative speed at exit |
|---|---:|---:|---:|---:|
| 1 ms | 17 ms | 0.111420 mm | 0.016199 mm | 0.010007406 m/s |
| 0.5 ms | 17 ms | 0.111416 mm | 0.016185 mm | 0.010003822 m/s |
| 0.25 ms | 17 ms | 0.111415 mm | 0.016179 mm | 0.010003096 m/s |

The two finest runs differ by at most 6.53e-9 m in NOTE relative position and 3.63e-6 m/s in relative velocity over 34 shared states. Their exit times agree to floating-point precision. A separate 20 ms acquisition half-step check gives 4.32e-9 m position and 3.92e-6 m/s relative-velocity differences. **These are local refinements from fixed inherited states.** The complete startup/index trajectory has not been rerun at all required finer timesteps, so this does not issue the full loading-contract verdict or reject the physical robot.

Prescribed-state grids of 512, 1024 and 2048 samples per dimension were compared at first contact, actual index and peak transport reaction. All 1024/2048 checks pass the declared 1% bounds with 1e-4 N force and 1e-6 J potential floors. The first-contact force comparison uses the absolute floor; its maximum tolerance ratio is 0.756. The coarser first-contact comparison fails and remains recorded. These are force/potential probes, not rerun acquisition trajectories.

## Concrete remaining boundaries

An independent steady-translation force balance exposes the current actuator-domain gap. At 0.10 m/s, the inherited ground model requires about 0.921825 N total forward belt traction. The straight-contact pressure model requires about -0.422889 rad/s rotor speed, -0.281704 A and -0.040586 V per pinch motor to maintain co-motion. Terminal power remains positive. Those small reverse commands lie outside the existing positive-drive/coast domain; an actual kernel call rejects -0.0034 duty without changing state. A passive brake is not an ideal zero-speed torque lock. These prescribed-state results do not prove every transient transport attempt must fail.

The next controller experiment needs an explicitly declared signed motor/controller model with current, voltage, source-power and dissipation accounting. Retain this failed positive-only attempt and its attained starting state; do not increase preload or reset the NOTE to make the next attempt pass.

Native registration is also separate. Both experiments reference the same base configuration hash, but the preserved relieved Fusion revision adds geometry overrides. Its running tread face is 176.422393 mm from center versus 176.922393 mm in the historical pressure model: 0.5 mm inward per side, a 1 mm total gap difference. The binding checker rejects automatic native qualification. The native archive remains unchanged at SHA-256 `5f2a1a4cb98931f31575444e95323c9cb78bab8df6d74f226c34dc53bd62bf96`.

A future native-bound fresh-start model must incorporate those overrides and the installed transforms. It must then carry one physical state through transport, lift/pitch and feed into the [separately staged launcher](../mujoco/LAUNCHER_QUADRATURE.md). Contact/material, mass/inertia and electrical parameters remain uncalibrated. No live Fusion interaction, computer control, native mutation or uncertain-audit retry occurred.

## Reproduction

Use the existing Python environment with NumPy and OCP; the historical setup imports CAD helpers even for simulation. All output paths must be new. The unoptimized exact-law continuation starts from the preserved 3 s report already in the repository:

```powershell
python -B sim/pressure/continue_pressure_index.py --project . --contract sim/pressure/evidence/continuity-20261118/pressure-index-contract.json --output $NewIndexDirectory
```

The tested faster resume uses its preserved 3.5 s checkpoint and parity receipt:

```powershell
python -B sim/pressure/continue_pressure_index_resumed.py --project . --contract sim/pressure/evidence/continuity-20261118/pressure-index-resume-contract.json --equivalence sim/pressure/evidence/continuity-20261118/pressure-tile-tests.json --resume "$PrivatePacket/pressure-index-resume-checkpoint.json" --output $NewIndexDirectory
python -B sim/pressure/continue_pressure_transport.py --project . --index "$NewIndexDirectory/receipt.json" --contract sim/pressure/evidence/continuity-20261118/pressure-transport-contract.json --output $NewTransportDirectory --dt 0.00025
python -B sim/pressure/verify_acquisition_continuity.py --project . --packet $PrivatePacket --manifest "$PrivatePacket/manifest.json" --output $NewVerificationFile
```

The private packet preserves original producers, failed/partial attempts and raw states. Public evidence contains scalar reports, contracts and source hashes. `evidenceChecksPassed` remains separate from `acceptance`, `nativeRegistrationAccepted`, `hardwareValidated` and `continuousAcquisitionToLaunch`, which remain false.
