# Current tooling audit and delivery requirements

## CAD delivery

The actual robot CAD must be a Fusion file. Existing Python/OCP generators and STEP exports are preserved as references and test assets; they do not satisfy that final delivery requirement by themselves. A native Fusion assembly, editable original features where appropriate, named components and saved/reopened Fusion archives are required. Supplier geometry and original authored parts must remain clearly distinguished. Do not describe the current STEP-only baseline as a completed Fusion robot.

## Preserved numerical baseline

The unchanged-controller fresh-rest trial completed all 3,000 steps to 3 simulated seconds. Runtime was 3,674.0834 seconds (61.24 minutes), including its current logging/checkpoint workflow. The completed receipt retains the exact 1,503-row partial prefix and original source identity.

First contact occurred at 1.403 s and about 0.23446 m/s, versus the earlier experiment's 0.562 s and 0.81795 m/s. This is approximately 71% lower contact speed at a cost of 0.841 s approach time, with the same controller goal/cap and contact-law assumptions.

The trial ends still moving: relative NOTE X about 201.190 mm and chassis speed 0.02113 m/s. Contact remains active. The conditional friction budget is about 1.9523 N versus 2.3046 N piece weight; this is a necessary holding-budget diagnostic, not a simulated lift or a physical safety verdict. Capture and lift are not established.

Independent review checked the exact receipt hash, 63 source pins, all 3,000 recorded rows and 300 controller events, preserved prefix and a fresh endpoint pressure/energy reconstruction. It did not repeat the hour-long run. Local half-step refinement covers 1.400 to 1.503 s only, not the full startup or entire trial.

## Established-engine comparison

New bespoke simulator expansion is paused for an independent tooling comparison. An isolated official MuJoCo installation and small geometry/contact benchmarks are being evaluated separately; no wholesale migration or calibrated contact accuracy is claimed. Match geometry, boundary conditions, loading and observables before comparing outcomes. Small contact-fixture runtime must not be divided into the whole coupled robot's 61-minute cost as a claimed engine speedup.

The existing pressure law is itself an uncalibrated hypothesis, not ground truth for tuning another engine. Preserve contradictory contacts, force timing, contact-only versus joint-stop reactions and contact-search/timestep sensitivity. Existing work remains available while the actual Fusion CAD and established-engine approach are assessed.

Artifacts: sim/pressure/fresh-rest-controller-3s-evidence.json, fresh-rest-firstcontact-100ms-PARTIAL.json, fresh-local-refinement-comparison.json and FRESH_REST_CONTROLLER.md.
