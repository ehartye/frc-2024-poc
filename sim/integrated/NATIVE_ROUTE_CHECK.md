# Exact native floor-route audit

The exact colored Fusion archive has now been reopened and checked directly. The loading milestone remains **PARTIAL**: offset NOTE witnesses intersect the original pulley outer-envelope representations, and those contacts have not been registered or modeled as physical pulley contact. This is neither clearance acceptance nor a rejection of the real mechanism.

## Executed evidence

Baseline: `c23ed8447d4cbbbd475103208a66ab1bb314b9cc`, descended from viewer checkpoint `6d6156e`. The earlier `aa9774f` viewer was superseded; the retained final viewer has a separately recorded user-operated GUI check. This audit used CLI and Fusion MCP, with no desktop control.

- Reopened the exact private F3D whose SHA-256 is `24de65755a396d1e371112c65eb9a44a32884c122bf1e4387fd172645ea45423` in a separate audit document. The original document remained unmodified.
- Matched all 732 native bodies against the exact visual manifest, including inherited parent-occurrence source identity. Checked 710 physical/source bodies, including 12 hidden bodies; explicitly excluded four service envelopes and 18 visualization bodies. Visibility never determined physical inclusion.
- Executed 1,443 flat NOTE witnesses: center X658 through X178 mm at 1 mm spacing, Y0 and +/-0.5 mm, Z25.4 mm. The 180–190 mm index band is included. Three translated/origin placement controls checked analytic bounds and volume, self-intersection and separation; deliberately wrong location witnesses were rejected.
- Completed 13,302 candidate body/NOTE Boolean pairs without a failed operation. Positive intersections were repeated with operands reversed. These are finite native-kernel checks, not continuous motion, manufacturing tolerance, deformation or physical force evidence.
- All 481 centered witnesses had only intended belt overlap. Each offset had 25 unresolved pulley-envelope witnesses, X312–336 mm. At X324 mm the largest overlaps were approximately 5.301 mm3 on the negative side and 5.332 mm3 on the positive side. Both are original axial-band outer representations, not supplier tooth/material geometry. Rotor phase is unregistered.
- Fresh portable drivetrain/display tests: **28 passed**. Exact native visual verification: eight meshes, 24 internal bounds checks, and 30 physical ticks exactly equal with and without display. An inspected offscreen image shows the red 1780 robot. This was not a new interactive GUI run.

The sanitized [receipt summary](evidence/native-route-gate-20261118.json) binds the full private receipt and executed script. Private raw native inventories, archive, diagnostic scripts, images and receipts remain outside public source. The prior 13 inherited broad-suite failures have not been repaired or relabeled.

A separate private OCP 8.0.1 source-geometry diagnostic checked the two peak-envelope NOTE centers against the preserved three-solid supplier pulley at one assumed orientation and the configured shaft centers. Both had positive overlap, with maximum approximately 0.353 mm3. Four independent analytic bounds controls checked NOTE placement. Actual installed supplier geometry/phase registration remains unknown; these are hypothetical diagnostic placements, not native assembly acceptance or a calibrated force model. A larger optional phase sweep was cancelled incomplete and supplies no result.

## Reproduce the native witnesses

`../../cad/native/native_route_scan.py` is a Fusion Python entrypoint, not ordinary CPython simulation. Through an already authorized official Fusion MCP session, reopen the exact archive into a separate owned document and mark its root with an `frcAudit/job` attribute. Do not edit or replace the source archive. Prepare `CONFIG` before dispatch with:

- `archive`, `archiveSha256`, `manifest`, `manifestSha256`: exact private inputs and expected digests.
- `documentId`, `job`: the newly inspected audit document and marker; selection must be unique.
- `output`: a fresh private JSON path; existing output is refused.
- `scriptSha256`: SHA-256 of the exact dispatched script bytes.
- `xMm`: integers 658 down through 178, inclusive; `lateralMm`: `[0, -0.5, 0.5]`.

Prefix the script with this independently prepared configuration and execute `run(context)` through `fusion_mcp_execute`. The script performs no native design edits; temporary geometry and private JSON receipts are the outputs. Preserve the dispatch script/configuration, MCP result, partial output and version. An exception, timeout, inventory mismatch, failed Boolean or incomplete output cannot establish clearance. Inspect an uncertain outcome before retrying. The numeric bounds comparison checks native proxy versus temporary-copy registration; it is not an independent kernel equivalence proof.

## Next physical gate

Resolve the pulley witnesses against private source geometry at independently registered shaft transforms and rotor phase, including appropriate intermediate motion and material-contact assumptions. The current source-sized cylindrical bands conservatively omit teeth and bore details; their occupied volume alone cannot establish actual contact. If native material contact exists, the pressure kernel needs an explicit supported contact treatment or a separately reviewed mechanism change. Do not delete the pulley, raise preload or attach the NOTE to pass this gate.

Until that gate is resolved, this branch does not run or claim the planned loading-to-transport experiment. Lift/pitch, overhead reception and shooting remain later integration work. The existing drivetrain is force driven but uncalibrated; displayed mechanisms remain kinematic geometry. Voltage/friction/weight assumptions remain unvalidated and thermal behavior is unmodeled.

## Launch the preserved driver

Using the existing Python/MuJoCo environment and private native export:

```text
python sim/mujoco/fusion_visual_drive.py --manifest <private-export>/manifest.json --fusion-archive <private-export>/source-red1780.f3d --trace <new-private-trace>.jsonl
```

WASD drive/turn, B brakes, C coasts, Q quits. This launches the preserved driver only; it does not provide intake, lift or shooting controls. GUI verification remains operator-owned.
