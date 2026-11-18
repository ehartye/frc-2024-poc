# Drive-over loading: native input gate

Status: **PARTIAL — exact-archive route registration unresolved**. This is the first evidence gate of the loading experiment, not successful acquisition or a physical-design rejection.

The experiment will test fresh floor acquisition, indexing and ground-supported transport with finite contact/motor forces. Lift/pitch transport and the angled output's intended overhead-loading role remain later work. The current pressure kernel is the intended single physics authority; the native viewer will display its accepted states.

## Executed checks

Inspected baseline: `6d6156e684c0b89d50aad7961ddccbe3121b5641`.

- The private native archive SHA-256 is `24de65755a396d1e371112c65eb9a44a32884c122bf1e4387fd172645ea45423`; manifest SHA-256 is `05f8faac0fc1e3554c801c5a83d4dc4b3bdd3fb9a356e9a5a9e0c92a8f04d767`.
- A fresh `verify_fusion_visual_display.py` execution passed eight-mesh loading, 24 internal bounds checks and exact equality of 30 drivetrain ticks with/without the display. The verifier explicitly uses the shared pose helper; it does not independently qualify native NOTE clearance.
- Existing native entry evidence includes 441 poses and 711 targets with no Boolean failures. Its 49 flat, centered samples from NOTE-center X658 to X178 mm report only intended side-belt contacts. That receipt identifies a different native archive (`3067ec17fde1f70630d96de3e7158fd2be2963bad9d160789beeb374cff53327`). It remains useful historical evidence, not clearance evidence for the exact viewer archive.
- All 29 dependency hashes listed in the existing public connected-feed receipt match the inspected checkout. That check establishes source freshness, not equivalence between the private native archives. The public feed receipt itself concerns the inclined intake/handoff configuration and is not a whole-robot floor-entry proof.

No loading trajectory, retention result, new GUI test or live-performance result was produced. Private geometry, paths and raw receipts are not distributed here.

## Unresolved requirement and continuation

Before running the loading experiment, obtain either an independently checked body/pose/material equivalence mapping from the tested native assembly to the exact viewer archive, or fresh native floor-entry witnesses from that archive. Preserve identity, full inventory/exclusions, world transforms, actual NOTE poses, raw contacts, Boolean failures and unsupported geometry.

A complete floor-pose STEP exported from the exact saved/reopened native archive can provide solids for a fresh offline audit, but the export must also have an archive-bound identity and pose receipt. Export alone is not clearance. Preserve all source bodies and classify visual-only geometry explicitly rather than deleting it to clear the route.

Then rerun the route gate. If eligible, proceed to the planned pressure-model experiment; if not, preserve a partial verdict or a qualified collision counterexample. Do not repin older receipts or change contact/preload parameters to bridge this evidence gap.

## Evidence boundaries

Physics, voltage, friction and weight distribution remain model hypotheses awaiting the loading run and calibration. Thermal behavior is unmodeled. The existing visual verification does not close any of those domains. This checkpoint is not a robot-design release.
