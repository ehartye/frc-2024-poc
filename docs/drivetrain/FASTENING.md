# Additive rail fastening study

This is a directly source-registered mounting pattern, NOT a selected complete superstructure. Existing drivetrain generator and old optional bridges are unchanged. Designer owns final FRO intake/electrical/bumper layout. The proposed transverse bridges intersect the separately proposed electrical longitudinal tubes at Y±130,Z161..186.4; do not combine unchanged. A full-width front bridge may also obstruct the FRO throat. Keep the proven hole pattern and redesign the upper structure as needed.

## Actual rail evidence

Pinned AM14U4 square STEP was freshly inspected for both inner-rail planar and cylindrical faces. Top flange is Z129.5399989773..132.7149989773mm, thickness3.175. Hole axes are vertical, diameter5.334mm, Y±260.35, X±63.5,±88.9,±114.3,...±342.9 at25.4 pitch. The cylindrical support's arbitrary origin Z is NOT its axial face height; the finite face bounds supply the flange elevations. Top flat spans approximately absY248.412..266.7. Y247.65 is not that hole row and lies just inboard of this flat; the previous whole-rail AABB cannot establish support there.

Source IDs/coordinate transform and48 top-hole records are in fastening.json. Native rails remain in private cache as hash-pinned BREP. Native extraction script: privatecache/frc-drivetrain/inspect-holes.py, from pinned official square STEP. Source geometry is not redistributed.

## Original drilled stack

Two optional bridge stations X101.6 and279.4. Rail fasteners use X63.5/139.7 and241.3/317.5 respectively atY±260.35. Original T-shaped4mm plates seat directly at flange top; wide96mm bar covers absY248.5..270 and40mm tongue extends inward to220. A plain rectangular plate would occupy the CIM envelope; the T-profile avoids it. Tube25.4mm X by50.8mm Z, wall3.175, spansY±244.475. Tube underside136.7149989773, top187.5149989773. Middle tube startsX88.9,18.161mm beyond nominal CIM X70.739; do not rely solely on the~0.341mm vertical gap to CIM tops.

Eight rail screws: AndyMark am-1047 #10-32x0.75in (19.05mm) with am-1042 locknuts. Stack4mm adapter+3.175mm flange+5.953125mm native-reference nut axial thickness leaves5.921875mm nominal beyond nut. Four tube screws: am-1397 #10-32x2.75in (69.85mm), through50.8mm tube+4mm adapter+same nut, leaves9.096875mm. Four original crush sleeves OD9.525, ID5.334, length44.45 fill the internal tube height. All holes5.334; nominal screw diameter4.826 gives0.254mm radial clearance in source rail holes. No washers included or silently assigned thickness. Sleeve material, tube alloy, fastener full-thread length, actual nylon engagement, torque, fatigue and load path remain unverified. Current long-bolt source does not independently establish2024 inventory availability.

Head cylinders radius4,height5 and nut outer radius5.6 are explicit access reserves, not exact manufacturer CAD. Threads are plain shank/hole proxies. Nominal length arithmetic is not bolt preload or accepted threaded engagement. Current nut webpage lacks axial dimensions;5.953125mm is from historical assembled CAD, not a blanket procurement tolerance.

## Run and integrate

    python tests/drivetrain/test_fastening.py
    python cad/drivetrain/build_fastening.py --cache <private-source-cache> --out <new-report.json>

Import build(c) from build_fastening.py with fastening.json to obtain46 original(id,shape,role) parts. Exclude old original-interface parts from build_drivetrain.py. Roles distinguish drilled-tube, drilled-adapter, custom-crush-sleeve, bolt-shank-proxy, head-reserve, nut-thread-proxy. Output STEP contains original geometry only.

Fresh test coverage: source-hole registration, changed/invalid fixed dimension rejection, axial stack arithmetic,46part count, allparts versus four CIM envelope boxes. Fresh native check compares all46 components to both source rails; nominal zero common volume and plate seat distance near numerical zero, rail shank gap0.254mm. A1mm sideways rail-bolt negative produces10.122127mm3 interference. Reopened STEP count46. These are nominal assembly checks, not complete robot/tool access, continuous motion, source tolerance, plate strength, fastener preload or fabrication acceptance.
