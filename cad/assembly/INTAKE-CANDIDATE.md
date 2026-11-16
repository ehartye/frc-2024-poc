# Original side-pinch cradle and chain elevator candidate

This is the next editable component assembly, not a verified complete robot. `python cad/assembly/build_intake_candidate.py` exports427 original/source-sized roots per floor, rotated and handoff pose to `artifacts/intake-component-candidate/`. Each STEP is reopened; the initial three poses have no geometry below ground. Supplier native CAD is not redistributed.

## Mechanism and route

Two reversible side-pinch belt tracks capture/hold the NOTE at its outside edges. Stub shafts and bearings stay beside the NOTE, not through its torus. This avoids an under-floor capture roller. Two-stage12:1 geared NEO proposals separately control cradle pitch and vertical lift; the pinch tracks have two additional direct NEO motors. The whole candidate therefore allocates8NEOs (4shooter,2pinch,1rotation,1lift) and4CIM drivetrain motors, matching the12controller allocation but not proving shared-power adequacy.

The cradle initially holds a level NOTE centered atX178,Z25.4mm. Rotation to60degrees is coordinated with upward motion to remain floor-tangent. Then it lifts to the shooter guide at local coordinate96mm. Total lift is approximately550mm; the418mm post-rotation segment alone was insufficient for lift sizing.

The proposed guides are original25.4mm square aluminum stock with3.175mm walls and adjustable UHMW sliding-pad envelopes. Carriage/drop brackets, stub-bearing carriers and drivetrain representations are editable. Complete fastening, lift-chain attachments, tensioners, rotation-shaft coupling/retention, brakes and guarding remain open. Geometry at intermediate moving poses still needs screening.

## Sourcing and transmission

- NEO family and12T RT25 pulleys use previously pinned source evidence. Side tracks use the currently listed104T REV-21-4104 family size (292.1mm equal12T centers); availability during the2024 season is not yet verified.
- MAXPlanetary uses3:1 and4:1 stages for12:1. Source cartridge drawing dates to2021 and system drawing to2022. The current model is only the documented2-stage envelope, not its internal gears/couplers.
- Lift uses16T halfhex REV-21-2016 #25 sprockets and REV-41-1365 cut-to-length chain.236links give1498.6mm nominal loop length and698.5mm equal-sprocket center distance. Two loops consume2997.2mm before actual installation decisions, leaving50.8mm of a nominal10ft stock length. Two connecting links are needed; the current stock listing includes only one. No chain working-load/masterlink retention acceptance.
- Chain is represented by straight-run envelopes and sprocket envelopes, not actual link articulation. Tooth engagement, polygon effects and attachment end clearances need native/physical checking.

## Evidence and exclusions

`python cad/assembly/check_intake_contact.py` checks the complete rigid NOTE against all included physical shapes at three key poses. Initial results show only the two intended side-belt compression overlaps (~732mm3 each), with no other NOTE intersections in those three poses. This is not continuous motion, capture, friction or contact-force validation. It is not a whole assembly pairwise collision pass.

The opened central throat cuts four named front rail/crossplate/bridge shapes. Fixed front backing support remains included. Lost transverse load paths and the replacement support plate need actual connection, stiffness and impact assessment. The previous unchanged-chassis25-conflict control remains separate.

All supplier representations, incomplete interfaces and missing masses are explicit. No old two-motor/240mm launch or competition qualification transfers to this candidate.
