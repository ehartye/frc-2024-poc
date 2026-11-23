# Profile A pickup, climb and power experiment

<!-- cad-still -->
[![Preliminary robot with the scoring cassette raised above the blue bumper](../../../../cad/preliminary-design/media/002-preliminary-robot.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Guided cassette at the start of a cycle, with a floor NOTE ahead. Preliminary design release 004, in progress.** The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. This still uses the same robot geometry as the interactive preliminary review. [Still and geometry record](../../../../cad/preliminary-design/media/002-preliminary-robot.json)
<!-- /cad-still -->

The default is the blunt UHMW blade on assembly 83517673. The 19 cases in results.json bind the original input scripts, individual output hashes and numerical outcomes. PLAN.md sets the stopping conditions. Run from this directory with the adjacent review environment's Python (NumPy, SciPy and MuJoCo). The existing private field and parent geometry paths are dependencies. All 19 cases are offline.

| Test | Result | Design action |
| --- | --- | --- |
| Pickup | Raising the deployed stop by 0.75° gives 8.92 mm nominal rigid tip height and 1.076 s seated. In the low case the A blade clears 2.96 mm and seats. The high, +50 mm lateral, low-grip case gives 15.92 mm, hits only the blade and fails on two refinements. Drive-off fails. | Retain Profile A. Leave the fixed stop change unselected. Measure the usable tip-height band and loaded belt speed on actual carpet, at both lateral offsets. |
| Climb | Seven trials. At 68.56 kg, 180 N·m and 30 ms delay, all three CG positions lose suspension. The +40 mm refinement reaches the 180 N·m torque ceiling and drifts 63.73° after cutoff. | Reject this friction brake as the sole disabled hold. Next test a load-bearing mechanical catch seated and confirmed before disabling motor torque. Include a failed-engagement control. |
| Scoring power | Weak-pack permissive requires acceleration released and 100 ms of forecast at least 8.2 V. Both numerical refinements registered both detectors and remained at or above 8.43 V. Bypass reached 5.93 V and missed. Sustained acceleration correctly denied firing. | Select sequencing for the controls prototype. Test actual response time and battery impedance; these runs imposed the drivetrain's electrical demand with the robot at a fixed scoring pose. |

The pickup driver uses assumed electrical constants and equivalent belt inertia, a current cap and a surface speed integrated from the signed contact reactions. Side belts retain their prescribed speed and bounded force law. The simulated NOTE is rigid. Tip clearance includes the imposed pitch/height errors, excluding carpet pile and elastic deformation. The aggregate high-side case tests the combined condition; independent causes have not been apportioned.

The climb model includes a finite friction-loss constraint, a 30 ms linear torque ramp and two ideal hook closures. A fully applied brake can still slip. The raw inherited field `held` tracks brake application; use `passScreen`, physical drift, motor torque and measured ground response for the hold decision.

The weak-pack shot gate also masks renewed acceleration until the NOTE clears the cassette. Gating includes the actual pre-enable settling time in the free-NOTE simulation. The two scored runs gave different detector times. Their convergence is only in outcome and the voltage floor. Long-term hardware performance remains open.

Open the LAN review with `?case=trial-pickup`, `trial-pickup-miss`, `trial-brake`, `trial-shot` or `trial-shot-miss`. Original motion cases remain selectable. The pickup trial changes the cassette's pose as a mounting-height sensitivity. The chassis mesh stays parked.

Use `python replay.py` with the review environment to reproduce the named trials; `python report.py` only rebuilds the derived evidence/view data. Publishing is explicit with `--live`.

