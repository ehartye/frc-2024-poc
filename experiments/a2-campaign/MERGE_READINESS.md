# Experimental checkpoint qualification

## Acceptance contract

This change is suitable for review as a preserved experimental 2024 checkpoint, not a finished robot simulator, fabrication release, calibrated physics model or successful scoring robot. Merging would not choose three simulators. Maple is a bounded arena option, MuJoCo a separately scoped drivetrain/mechanism research tool, and Synthesis interoperability remains untested. No merge is performed by any verification command.

## Scope and provenance

Qualification baseline: `29f10c4da05d24c9ac33df67d3bb486b9e219234`; target main `88d1391ce6f459cce075c17f25795f4458c80901` is its ancestor. Subsequent readiness edits affect documentation and the public-content guard/tests only, not numerical source, model parameters, meshes or historical evidence. The PR spans many experiments; it is not one executable acquisition-to-score pipeline.

Public assets are original/source-sized project geometry, with exact allowlisted mesh hashes linked to `sim/mujoco/visual/manifest.json` and the original connected STEP. Supplier native CAD, PDFs, Fusion archives, private wiki and machine paths are excluded. Repository MIT and retained upstream notices apply to their respective material; public availability alone is not vendor redistribution permission. The matching Fusion archive is a separately preserved private artifact.

## Fresh local smoke matrix

Executed against the qualification baseline using existing dependencies, without GUI, installation or model changes:

- MuJoCo portable package: **22 Python tests passed** (five modules listed below).
- Current pressure source: **13 tests passed** (6 fast geometry, 1 benchmark receipt, 1 corrected lineage, 2 early policy, 3 historical-verifier tests). These are not every historical experiment or complete trajectory reruns.
- Maple: full Gradle `test --rerun-tasks` passed **both JUnit test classes**: one six-group arena test and one projectile characterization. Source inventory remained stable. Upstream is pinned to `1b1e8ea7278de01d40a5ea14dffbaaf13381512a`.
- Public guard initially rejected three intentional original STL meshes because its binary allowlist had not been updated during portable migration. Readiness correction allows only their exact reviewed hashes. Added negative tests retain rejection of modified assets and arbitrary binary files. No general STL exemption.

Independent reviewers separately replayed the Maple arena and projectile tests, MuJoCo portable headless tests and selected historical studies. Projectile replay differences up to 1.78e-15 m are below its declared 1e-9 timer-epoch tolerance. Airborne target callbacks persist despite an independently obstructing finite-piece path; that is **missing collision support**, not shot acceptance. Trace samples after target removal can be analytic getter output from the retained object, not a registered arena body.

No GitHub Actions checks or GitHub review approvals were present at baseline. Explicit local and independent review evidence is the gate; absence of CI is not a green CI result. No new all-repository test-count claim is made.

## Reproduce without GUI

From `sim/mujoco`, with its documented existing Python environment:

```text
python -B -m unittest test_native_drive test_native_drive_interactive test_native_drive_interactive_v3 test_fusion_visual_display test_fusion_visual_provenance
```

From repository root, run each independently and require a zero exit code:

```text
python -B -m unittest discover -s tests -p test_pressure_fast_geometry.py -v
python -B -m unittest discover -s tests -p test_pressure_benchmark_receipt.py -v
python -B -m unittest discover -s tests -p test_pressure_corrected_lineage.py -v
python -B -m unittest discover -s tests -p test_pressure_early_policy.py -v
python -B -m unittest discover -s tests -p test_pressure_historical.py -v
python -B -m unittest discover -s tests -p test_public_content.py -v
python scripts/check_public_content.py
python sim/maple/verify_run.py --maple-source <clean-pinned-maple-checkout>
```

The Maple runner executes both classes; its `testCount` field refers specifically to the arena class, not aggregate JUnit count. Inspect both XML test reports for aggregate results. It performs no automatic installation. Historical source-bound tests must use `sim/pressure/VERIFYING.md`; broad discovery against changed model versions is not an honest historical validation.

## Remaining product and physical limits (not silently waived by merge)

No accepted collection, retention/lift/launch mission, corrected Maple scoring, airborne obstacle physics, real robot-code interface, full mechanism dynamics or calibrated hardware behavior. MuJoCo visuals do not participate in collision/mass, stay in floor pose and do not animate individual wheels. Its recorded UI test belongs to the audit implementation; portable migration has headless equivalence, not a fresh UI run. Private Fusion CAD and optional native source caches are not distributed here. Root documentation distinguishes historical model generations rather than repinning old receipts.
