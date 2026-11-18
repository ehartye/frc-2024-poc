# Native-bound signed ground acquisition and transport

This experiment starts a loose NOTE from rest, acquires it with finite forces, and uses signed pinch effort during ground transport. It binds the contact geometry to the preserved 734-body native Fusion candidate. It does not attach the NOTE, import an old captured state, lift it to a launcher, or validate hardware.

The native source remains the private F3D archive with SHA-256 `5f2a1a4cb98931f31575444e95323c9cb78bab8df6d74f226c34dc53bd62bf96`. `bind_native_pressure.py` checks the final native receipt, four belt/tread body volumes and bounds, shaft offsets and 1.5 mm tread. It does not distribute CAD or prove continuous whole-robot clearance. Lifted and pitched configurations remain outside this binding.

## Recorded results

| Experiment | Observed result |
| --- | --- |
| Fresh 1 ms | Index timeout at 6 s; preserved coarse failure |
| Fresh 0.5 / 0.25 ms | Index at 4.450 s; three seconds transport; chassis 0.229108 m, NOTE 0.225563 m |
| Initial pinch-coast stop | Corridor exit after 100.5 / 100.25 ms |
| Clipped negative-duty replay | Relative-speed exit after 187.5 / 187.25 ms |
| Active-pinch stop, 0.5 / 0.25 ms | Two seconds completed, last 100 ms settled; exact attained transport join |

The joined half-step path has 18,900 accepted steps over 9.450 s. Maximum cumulative energy residual is 0.000236 J. Full-startup fine comparison passes the declared bounds: maximum relative-position difference 4.48e-7 m and relative-speed difference 1.56e-4 m/s. Local active-stop refinement gives 4.52e-7 m and 1.87e-5 m/s. Both stop refinements use the same attained half-step checkpoint; a full independent fine-start active-stop replay is not claimed. Local spatial comparisons pass but do not establish full spatial trajectory convergence.

The published launchers additionally catch the inherited `DomainExit` exception when recording rejected step outcomes. The exact executed producers are preserved under `evidence/native-signed-20261118/executed/`; this reporting-only hardening does not alter accepted step equations. Their recorded runs must be checked against those executed bytes. The private packet retains the original names and full traces.

## Model and controller

`SignedPressureAcquisition` mirrors the inherited positive motoring/coast law into negative duty with the same motor constants and 40 A current limit. It rejects unsupported voltage and regeneration. Zero duty coasts; it is not an ideal brake or position lock. One shared supply accounts for finite drive and pinch electrical power, winding losses and contact work.

`NativeSignedPressure` uses the native contact dimensions and the same finite-area pressure law, with a 1024 by 1024 material grid, pressure density 1e7 N/m3, belt friction 0.6 and ground friction 0.4. Those are uncalibrated hypotheses. Pressure depth beyond the tread, unsupported contact geometry and nonplanar wrenches stop the model. NOTE height, roll and pitch remain constrained by the ground-plane model.

The additive NumPy implementation retains the material quadrature. Preflight covers 189 electrical cases and 20 scalar/array geometry cases, including matching domain rejection. Three 20-step prescribed-state dynamic comparisons cover positive, negative and coast commands; these are implementation checks, not acquired states. Exact executed files and contracts are byte-pinned.

Fresh acquisition retains the prior 100 Hz approach controller. Transport uses a 0.1 m/s target with a 0.1 m/s2 ramp, inverse regularized-friction feedforward and bounded position/velocity/current control. Required negative terminal power is replaced by a declared small plugging voltage; battery regeneration is not invented. All actual states still result from motor/contact forces.

The initial stop commands chassis braking and pinch coast. Its failure is preserved. `continue_native_signed_stop.py` instead resumes the exact attained transport endpoint and retains signed pinch regulation while the chassis brakes. Its rolling-brake acceleration estimate is only controller feedforward; it never assigns physical acceleration or velocity.

## Reproduction

Use Python with the project's existing NumPy and transitive OCP dependencies. No new simulator installation is needed. Run from the repository root, using fresh output directories:

```sh
python sim/pressure/verify_signed_native.py --project . --binding sim/pressure/evidence/native-signed-20261118/native-pressure-binding.json --contract sim/pressure/evidence/native-signed-20261118/signed-native-contract.json --output artifacts/native-signed-preflight.json
python sim/pressure/run_native_signed.py --project . --binding sim/pressure/evidence/native-signed-20261118/native-pressure-binding.json --contract sim/pressure/evidence/native-signed-20261118/signed-native-contract.json --output artifacts/native-signed-half-ms --dt .0005
python sim/pressure/continue_native_signed_stop.py --project . --binding sim/pressure/evidence/native-signed-20261118/native-pressure-binding.json --trajectory artifacts/native-signed-half-ms/receipt.json --contract sim/pressure/evidence/native-signed-20261118/native-signed-active-stop-contract.json --output artifacts/native-active-stop --dt .0005
```

Repeat the fresh run at .001 and .00025 s. The full trajectories are offline evidence, not an interactive or real-time robot viewer. Private packets retain full states, commands, failures and hashes; public receipts contain sanitized scalar comparisons. The verifier needs the preserved packet's sources and complete trajectories. Local 512/1024/2048 contact checks do not replace full spatial trajectory refinement.

## Lift and feed boundary

`check_native_lift_join.py` rejects automatic composition with the historical lift class. That class imports fixed contact witnesses and treats inherited normal spring energy as dormant. It cannot replace the current native pressure field while claiming continuity. A valid next model needs native moving-body mass/COM/inertia and transmission transforms, free NOTE vertical/pitch dynamics, explicit floor support, finite lift/pitch effort on the same supply, and preserved contact/energy states. Clearance must be checked along the new motion.

The staged launcher remains a separate experiment. Its shooting result cannot be joined by changing the acquired NOTE's pose, velocities, joint set or energy. Numerical agreement here is neither calibrated physics nor hardware validation, and no completed acquisition-to-launch cycle is claimed.
