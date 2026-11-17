# Original side-pinch cradle and chain elevator candidate

This is editable component CAD, not a fabrication-ready or qualified complete robot. Supplier native CAD remains private. Export using `python cad/assembly/build_intake_candidate.py`; verify exact exported files using the contact audit and verification bundle. Root counts alone are not completion evidence.

## Current route and geometry

A level NOTE enters below the fixed front bumper at robot X178, Z25.4 mm. Two reversible side-pinch tracks hold the outside edges. The cradle rotates to 60 degrees while rising to keep the rigid NOTE floor-tangent, then lifts to handoff at X178, Z474.833 mm. Total vertical travel is 449.433 mm. The carriage is 200 mm above the NOTE, suspending the narrow cradle below high, inboard pinch motors. This is prescribed geometry, not demonstrated acquisition or retention.

The entire shooter moved 58 mm forward to pivot X188 mm. Handoff is shooter guide coordinate -20 mm. All previous release/target/docking results are invalid for this layout until explicitly rerun. The candidate allocates eight NEOs and four drivetrain CIMs, not proof of shared-power adequacy.

Side tracks use nominal 104T RT25 loops with 292.1 mm centers. Remote pinch transfer uses 48T equal-12T loops. Exact 104T availability in the 2024 season remains unverified. Two-stage 12:1 MAXPlanetary envelopes are supported by 2021/2022 drawings, not modeled internal couplers. Lift uses two 236-link #25 chain loops with 16T sprockets, nominal centers 698.5 mm. Rotation uses 114 links, 16T sprockets and 311.15 mm centers. Chain is straight-run/sprocket envelope geometry, not articulated links or tooth engagement.

## Joint correction and structural changes

The initial complete mechanism collided despite a clean NOTE-only screen; its failed evidence remains preserved. The suspended revision removed external collisions in finite samples. Internal auditing then found missing bores and overlapping authored stock. The next revision bores the pivot sprocket, shortens stub shafts, and models each original side frame as a fused proposed weldment with drilled bearing seats. Fabrication, welds, tolerances and strength remain unqualified.

Current guides are at Y +/-250 mm, fixed shooter side rails at +/-300 mm, and authored transverse tubes end at +/-313 mm. The central throat is 430 mm wide. Native rail attachment stations remain at Y +/-228.6 and source rail bolt rows +/-260.35. The central front endrail and lower transverse bridge regions are removed; source crossplate `am-2974-cut-2` is removed entirely. A proposed upper transverse portal is added at guide X101.6, bottom Z917.515 mm. This is a replacement-member proposal, NOT an established load path: its attachments, lower guide anchorage, carriage yoke, cradle torque hub, chain attachment and tensioning remain incomplete. Wider cantilevers and the fixed bumper backing/support require structural and impact assessment.

## Evidence boundaries

The moving/fixed audit retains raw overlaps and explicit motion roles. The dense variant samples at most 1 degree of rotation and 5 mm of lift, not a continuous collision proof. The internal audit checks all moving-part pairs at three key poses. Remaining belt/pulley and chain/sprocket envelope intersections are labeled nominal drive-engagement hypotheses; raw volumes remain in the report and are NOT waived as physical acceptance. Unknown or cross-motion intersections remain explicitly unresolved.

Three-pose full-NOTE contact is separate from mechanism clearance. Intended side-belt penetration does not prove capture, friction or force. The source/STEP verifier requires exactly floor, rotated and handoff, hashes actual files, rejects missing/modified/forged receipts, and guards source changes during export/audit. Export receipts must be regenerated after source changes; old outputs cannot certify a new revision.

Per-part fidelity distinguishes original stock proposals from supplier-sized representations. Partial mass ledgers do not imply whole-robot mass or CoG. Complete fasteners, couplings, chain attachments, tensioners, guarding, harness/service access and structural acceptance remain gates.

Power-off motion/retention behavior and any brake are unmodeled; retention is mission-dependent. Coasting alone is not treated as a safety defect.


## Feed-path correction and connected candidate

The full guide-aligned feed screen rejected the earlier upper bearing carriers even though the three held-NOTE key poses were clear. Bearing centers are now -18.4/+25.4 mm relative to the NOTE plane, with rear shafts -23..34 mm and connected revised side webs. Moving the tracks outward then revealed a rear carrier/gearbox conflict, also retained in evidence.

The selected new candidate uses a 64T REV-21-4064 loop, 165.1 mm centers, shifted +63.5 mm locally. Front pulley stays at +146.05 mm, preserving forward pickup/feed reach; rear pulley moves to -19.05 mm. This puts the held NOTE near the rear end of the straight contact region, so acquisition, alignment variation and traction remain explicit tests. The current belt drawing is dated April 2025; it does not establish exact part availability during the 2024 season.

Each track moves outward 1.75 mm, leaving about 0.878 mm nominal side-belt indentation. This is a geometric hypothesis, not a measured preload or worn-piece allowance. Original pulley representations now use source-bounded outer axial bands instead of a full-radius barrel. The pinned source has THREE solids; native witnesses sum absolute common volumes per solid. Earlier whole-compound zero results are not accepted as material-clearance proof.

`build_connected_candidate.py` combines the lower brackets, upper portal gussets, carriage yokes and detached slide-pad backing/fasteners into a separate export. It preserves earlier frozen artifacts. Pad screw recess is 0.5 mm nominal, not an approved wear interval; pad replacement requires disassembly. Actual chain attachment, torque/retention couplings, motor mounts and remaining shooter joints are still incomplete. Electrical breaker source is surface-only and PDH includes an invalid solid, so old electrical material-volume clearance claims are downgraded.
