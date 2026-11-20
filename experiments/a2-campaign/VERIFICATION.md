# Verification

## A2 integrated cycle

Fresh clean-clone verification on Windows, 2026-11-16, at source commit `96ec797`:

- 24 Python tests and the public-content guard passed; the full native Java test suite passed.
- The documented `run-a2.ps1` command completed a fresh offline run. Native run ID: `91bc20f0-891e-4637-bb22-44f750371129`.
- NORMAL completed two releases in 27.320 seconds. NORMAL_9V and STRICT_12V also completed; missing NOTE, jam, stale pose and STRICT_9V reached their expected faults.
- The first and second NORMAL exits had approximately 67.93 and 67.96 mm sampled aperture margins, using actual recorded exits and an assumed fixed 60-degree NOTE attitude. This is not scoring evidence or a continuous clearance certificate.
- Fresh 390-pixel-wide browser checks of NORMAL and STALE_POSE replays reached the expected DONE/FAULT endpoints without JavaScript errors or horizontal overflow.
- Native receipts bind fresh execution, sources, runtime classes and output hashes. The presentation receipt binds that native receipt, replay/evaluation sources and generated artifacts. Interrupted pending runs are not promoted as results.

The roughly 27-second cycle is not a 15-second autonomous routine. Floor acquisition, telescope motion, level retention and transfer remain prescribed hypotheses. Shooter/guide energy accounting does not include drivetrain or prescribed transport energy. Fixed body attitude, spin, aerodynamic effects, full field structure clearance, realistic sensing, contact and impact durability remain unvalidated. No official score, legal inspection or physical acceptance is established.

## Original A2 CAD

Both original stowed and deployed exports reopened as 54 solids. Shared validated kinematics drive CAD, transport geometry and the sweep screen; canonical and runtime snapshot bytes must match. Independent correction review checked 162 native/analytic moving-part bounds and mutation controls. The 1,501-pose screen reports a nominal maximum height of 1.2142 m and forward roller extent of 0.655 m. The approximately 5 mm height and 4.8 mm extension allowances are nominal geometry, not engineering tolerance or strength margins. Reserved linkage/service volumes are not completed hardware.

## Retained legacy regression

The earlier bounded preloaded slice retains its three Java integration tests and all-season regression fixtures. Its historical run ID was `c6174357-de5e-408f-865b-75bb51970500`; its eight-root CAD export is not the new A2 assembly. Those fixtures are shared dependencies, not additional 2024 mission behavior.

Machine-specific logs, native binaries and caches are not published. Re-run to generate complete receipts for your environment.
