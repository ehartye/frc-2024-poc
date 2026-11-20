# A2 complete-cycle mission simulation

## Run

With existing Python, PowerShell, JDK 17 and populated pinned WPILib/Gradle cache:

```powershell
./scripts/run-a2-mission.ps1 -JavaHome '<JDK17 directory>' -GradleHome '<populated Gradle cache>'
```

The fresh output directory contains seven separate scenario CSVs/JSON summaries and an atomic `receipt.json`. No installation, deployment or robot operation is performed. JSON includes no machine paths. Diagnostic logs remain local outputs and should not be published without review.

## What is actually connected

The original `CrescendoMissionSupervisor` commands ideal WPILib differential-drive kinematics, a prescribed A2 telescoping cradle/transfer model, and one reusable `A2ShooterAdapter` for both shots. Shooter RPM and exits come from the actual motor/contact plant, not a timer or injected readiness. Exit velocity and the actual simulated chassis pose determine the recorded world exit.

A preload is followed by drive/align, spinup/release, travel to a known floor NOTE, acquisition/deployment, lift/transfer/index, return/align and a second release. Virtual possession starts only after the floor piece lies within 60 mm of the deployed cradle and completes a prescribed 150 ms seating translation. Indexing requires reaching the shooter-entry coordinate. Stow requires the actual prescribed arm angle. Missing pieces cannot become acquired merely by waiting.

**This is not physical floor capture.** Seating, retention, coordinated telescope/arm motion and transfer are prescribed ideal kinematics. The transfer settles at the guide entry before feed; attitude changes into the 60-degree guide are a hypothesis, not solved contact or rotational dynamics. No gravity-drop, linkage loads, chassis collisions, opponent impact or real sensing are validated. Field and target registration remain provisional. The global trace terminates on a controller/adapter fault; that is not a physical stopping prediction.

## Pinned geometry and motion hypotheses

`mission/cycle-contract-A2.json` and `mission/a2_kinematics.py` are exact original-design snapshots from revision `0b119bf5f054c80a96ad1c1f0b6f180b7e715cf2`. `scripts/generate-mission-geometry.py` produces the Java constants and supports `--check`. The native trajectory is checked against the authoritative Python telescope length law at EVERY emitted CSV row before a receipt is accepted. Height/forward caps include the retention roller, shaft, sleeves and links; 5 mm nominal allowance is not tolerance qualification.

The shot goal is the contract's robot center (0.9834480659573899, 0) m at 180 degrees; actual pose error is retained. Pickup robot goal is (3, 0) m at 180 degrees with the floor NOTE at (2.36, 0, 0.0254) m. The latter is an explicit scenario placement, not detected field truth. Arm rate is 90 degrees/s, transfer speed 0.4 m/s, control period 10 ms, physics integration step 100 microseconds. These are configurable future integration hypotheses, not actuator specifications.

## Continuous reusable shooter

Reusable behavior is explicit opt-in. The original default single-shot API remains unchanged. `departBody()` is permitted only after guide exit. It adds the outgoing body's kinetic plus guide-relative potential energy to a cumulative ledger, clears ONLY the departing body's coordinates, and retains rotor velocity, clock, accumulated motor work and slip loss. A second feed records any introduced feed kinetic energy separately (zero in this mission).

Disabling the opt-in motor opens both electrical branches in `SharedSupply`. It does not set rotor speed to zero and does not implement a brake. There is no bearing/air drag in this bounded model, so unloaded rollers keep spinning while coasting. Tests preserve that power-off counterexample and demonstrate continued piece motion when power is removed mid-release. The default non-reusable adapter still suspends unsupported command changes; its prior behavior is not silently broadened.

The energy identity covers **powered rollers and the straight guide only**, including outgoing-body energy. It excludes drive, ideal lifting/transfer, orientation changes and real losses outside the plant. A tiny residual is internal accounting, not whole-robot energy validation.

## Observed scenario results

- NORMAL (12 V): two actual guide exits, one prescribed/virtual capture, DONE at about 27.3201 s.
- NORMAL_9V: DONE at about 27.3401 s with the permissive 1800-RPM readiness / 1000-RPM loaded floor hypothesis.
- STRICT_12V: completes with 2500/2500 RPM thresholds; STRICT_9V faults on loaded RPM before the first exit.
- Missing NOTE, repeated jam and stale pose feedback do not complete the mission.

This approximately 27.3 s serialized sequence **does not fit a 15 s autonomous period**. No score, scoring throughput or accepted strategy is asserted. Concurrent drive/spinup and other timing changes remain alternatives, not assumed improvements.

## Trace contract

CSV columns: `timeS,state,robotX,robotY,robotHeadingRad,intakeAngleDeg,intakeLengthM,piecePhase,pieceXRobot,pieceYRobot,pieceZRobot,pieceXWorld,pieceYWorld,pieceZWorld,possession,indexed,stowed,shooterRpm,shotCount,fault,event,exitVx,exitVy,exitVz,bodyPitchRad,bodySpinRadPerS,poseAgeS,energyResidualJ,outgoingBodyEnergyJ`.

Coordinates are metres, angles explicitly named degrees or radians. Times use the continuous plant clock including coasting. EXIT rows occur at actual guide crossing times; non-event velocity and body-attitude fields are blank. Spin remains blank/unknown, not zero or stabilized. Body pitch at exit is the declared guide-angle hypothesis. `shotCount` counts actual exits, not scores. `fault` includes adapter faults; terminal dynamics beyond the recorded boundary are not implied. Samples may share a timestamp with a boundary event; timestamps are nondecreasing. Replay should show recorded samples rather than fabricate smoother dynamics.

## Receipt and verification boundary

The runner requires a new output path, reruns the native tests, verifies unique run tokens in both XML and runtime properties, validates compiled class hashes (including the synthetic enum-switch class), pins source/geometry/toolchain artifacts, checks seven scenario outputs, and atomically publishes the receipt only after all checks pass. Stale-token and missing-scenario negative tests prevent relabeling previous evidence. Identical inputs produce identical CSV traces.

Independent review and real hardware calibration remain separate. The repository's older `run.ps1` retains its documented preloaded-slice scope; this additive command runs the mission experiment.
