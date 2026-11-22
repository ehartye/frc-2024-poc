# 2024 MAXSwerve frame and electronics supports

The floor NOTE enters below the front bumper crossmember. Two longitudinal rails and one rear rail bolt directly between the REV-21-3005 housing tabs. Each front module receives one rail end. Each rear module receives a side rail and one end of the rear rail. A transverse member joins the side rails behind the intake. Two short stringers connect it to the rear rail. Four open pedestals support the retained electronics beams at their existing attachment stations. The side openings in the pedestals provide access to the retained deck locknuts.

| Original frame article | Quantity | Nominal dimensions or interface |
| --- | ---: | --- |
| Drilled rectangular rail | 3 | 480 × 38.1 × 50.8 mm, 3.175 mm wall |
| Through-bolt and locknut | 12 pairs | 10-32 × 2.75 in socket-head envelope, unmodeled threads |
| Crush sleeve | 12 | 9.5 mm stock diameter, one flat 3.05 mm from center, 5.1 mm bore, 44.45 mm length |
| Transverse member | 1 | 621.6 × 25.4 × 50.8 mm, 3.175 mm wall |
| Rear stringer | 2 | 158.4 × 25.4 × 50.8 mm, 3.175 mm wall |
| Deck pedestal | 4 | 56.065 mm overall, 4 mm cap, 17 mm socket opening |

The supplier housing has two mounting tabs per plate. Mounting holes sit at 12.7 mm pitch. The selected holes have a 5.0 mm throat. Several include a 5.1 mm relief extending 2.5 mm from the rail face. The team rails use the outer pair at each attached tab, with 5.1 mm drilled clearances in both rail walls. The 50.8 mm nominal stock height spans the gap between the upper and lower plates. Actual stock tolerances, tab spacing and assembly fit need measurement. The smallest nominal ligament from a drilled rail hole to the side edge is 3.8 mm. A cylindrical sleeve intersected the sidewall by 342.54 mm³ at each bolt. The machined flat leaves 0.125 mm nominal lateral clearance inside the tube. The modeled screw tips project 4.78 mm below the locknuts. [REV module drawing](https://www.revrobotics.com/rev-21-3005/)

The deck load path runs through the two retained beams and four pedestal caps into the lower frame. The root faces of the pedestals and the ends of the crossmember and stringers are designed as welded joints. The solids model their bearing faces. Welding procedure, joint strength, rail end bearing, clamp preload, fatigue, harness access and impact loading need engineering and physical checks. The nominal 46-part assembly weighs an estimated 3.50 kg using 2,700 kg/m³ for the aluminum bodies and 7,850 kg/m³ for the fastener envelopes. This estimate excludes weld metal. The earlier simulator used an assumed total mass of 45 kg.

For scale, a 1 kN vertical point load centered between the bolt stations of one simply supported side rail gives 15.77 MPa nominal bending stress and 0.156 mm elastic deflection. This calculation uses an unperforated midspan section with 183,863 mm⁴ second moment, 456.5 mm span and an assumed 69 GPa elastic modulus. It addresses elastic rail bending in that load case. The assembled frame has additional load paths and local joint stresses.

## Reproduce the geometry and checks

### Lift bridge supports

Two 8 mm aluminum webs carry the intake guide and shooter uprights on the side rails. Each web has a 115 mm landing and a raised forward edge that clears the module plate. Four 12 mm lengths of 25.4 × 50.8 mm rectangular tube extend the original lift bridges out to the webs. Each extension matches the 3.175 mm wall of the bridge. The intended joints are welded. The original bridge tubes and their upright brackets remain installed. The Fusion timeline retires the 44 adapters, sleeves and fastener parts that tied the bridges to the six-wheel frame.

Each web bears on 920 mm² of side rail. All four tube ends bear on 443.55 mm² at each end, measured on the reopened assembly using 0.01 mm intrusion at the weld faces. The six pieces add an estimated 0.667 kg before weld metal. Retiring an estimated 0.290 kg of original hardware makes the net increase 0.377 kg at the assumed material densities. The moving mechanisms maintain at least 36.65 mm separation from these supports throughout independent lift and pitch. The free NOTE maintained at least 122.88 mm sampled separation in two pickup and delivery runs. [Mast evidence](maxswerve-lift-supports.json)

The two swerve carry traces give peak net vertical support forces of 76.79 N and 76.87 N for the modeled moving assembly, including the NOTE. The calculation takes the change in total linear momentum of the carriage subtree and the free NOTE over successive airborne samples and includes their combined weight of 70.95 N. The peaks represent the resultant on the entire moving assembly. Individual joint reactions, actuator and chain preload, local mast moments and bumper loads require separate checks. [Trajectory load calculation](../../sim/pressure/swerve_mast_load.py)

For a separate loading example, treat the 89.4 mm forward projection of one web as a cantilever with a rigid root at the end of the landing. At the front tube center, a 1 kN vertical force gives 17.29 MPa nominal bending stress and 0.021 mm tip deflection. A separate 250 N lateral force gives 33.65 MPa and 0.325 mm. The calculation uses the 8 × 62.265 mm web section and 69 GPa elastic modulus. A load introduced high on an upright also creates a moment. Welds, the rail wall, the step stress concentration and the actual attachment compliance still need qualification.

The Fusion authoring module builds the webs and extensions on the supported chassis. The verifier measures their installed material and checks both ends of each connection. [Authoring module](../native/swerve_mast_support.py) · [Native fit check](../native/verify_swerve_mast.py)

### Lower frame

In the reopened robot, all twelve bolt axes registered through both supplier plates. Of the 24 measured passages, eight included the short relief. A 4.98 mm gauge cleared every passage through its full depth. The 5.20 mm oversize control intersected at least 3.88 mm³ of plate material in every passage. A 0.02 mm intrusion at the fastener and rail faces measured minimum bearing areas of 29.69 and 43.19 mm² respectively. Each of the four pedestal caps seated on approximately 418.22 mm² of the retained beam.

The revised robot retires 83 six-wheel chassis occurrences containing 84 solids. The static check found zero frame intersections above 0.01 mm³. The 188 moving bodies maintained at least 82.30 mm modeled separation from the new frame across independent lift and pitch. Across the two recorded floor-NOTE pickup and carry runs, the smallest sampled NOTE-to-frame bound was 120.97 mm. Harness routing and structural qualification remain open. [Native measurements and test bounds](maxswerve-supported-frame.json)

Run `cad/native/build_swerve_frame.py` from Fusion's Scripts interface with its three sibling authoring and inspection modules available. Choose a new local F3D filename. The script builds the editable tube, drilling, sleeve, fastener and pedestal features, then reopens the archive and measures the result. The screw and locknut solids use the project’s 10-32 exterior dimensions. Nominal geometry omits helical threads and socket-tool tolerances. [Fastener model basis](../electrical/fasteners.py)

The installed-robot check compares the new solids with the four articulated supplier modules and the retained robot. The interval test sweeps the registered moving intake and lift through independent lift and pitch. The separate NOTE check reads each attained pickup and carry sample for the two development runs. [Installed solid check](../native/check_installed_swerve_frame.py) · [Trajectory check](../../sim/pressure/swerve_frame_note_clearance.py)

### Removable bumper prototype

Four editable rails support four drilled 19.05 mm plywood panels. Eight M6 × 25 socket screws engage captive square-flange barrel nuts. Each screw seats on 44.33 mm² of rail, with 7.313–11.250 mm nominal thread engagement. The reopened Fusion assembly retains all eight foam cushions and 1,092 swerve module solids. A 16 mm diameter, 48 mm long tool gauge cleared all eight screw positions. Raising the chassis 200 mm over the supported bumper cleared all twelve wood and foam envelopes, with 14.5 mm vertical separation at the end. Straight upward removal of the bumper would run into the shooter overhang. Cover behavior and physical service time still need a shop check. [Native authoring](../native/swerve_bumper.py) · [Fit and service inspection](../native/verify_swerve_bumper.py)

The modeled lift and cradle cleared the bumper envelopes by at least 2.895 mm throughout 0–450 mm lift and 0–60° pitch. The NOTE cleared them by at least 2.886 mm and 2.702 mm at the sampled poses of the two mirrored pickup and carry runs. Both closest points occurred at the front frame rail. The wood and foam envelopes include a 1.5 mm cover allowance. [Measured clearances](maxswerve-bumper-fit.json) · [NOTE trajectory check](../../sim/pressure/swerve_bumper_note.py)

**Front rail redesign.** A simply supported 704.15 mm span under an assumed 1 kN central horizontal load gives about 39 mm elastic deflection for the fitted stepped plate. The model omits plywood composite action, joint stiffness and yielding. This result rules out relying on the plate alone for that case. A 38.1 × 50.8 × 3.175 mm box tube projects forward, moves the inner wood face by 34.925 mm and gives 0.91 mm deflection under the same assumptions. The initial placement and both recorded NOTE paths supported that location. The R7 assembly below includes the corner connections and revised panels. [Bending calculation](bumper_front_bending.py)

### Installed front crossmember

R7 replaces the stepped front plate with a 711.2 mm length of 38.1 × 50.8 × 3.175 mm rectangular tube. Two drilled landings on top carry the original removable screws. The side rails extend 6.35 mm and seat directly on the rear wall of the tube. Each front corner has 241.935 mm² of nominal bearing face. The front plywood, cushions and red cover advance 34.925 mm. The side plywood and cushions grow by the same amount. The reopened design locates all sixteen white team digits on their four cover faces. [Editable Fusion build](../native/swerve_front_tube.py)

The eight tool approaches and the 200 mm chassis lift for bumper removal cleared the assembled robot. The modeled mechanism clears the installed bumper by at least **9.245 mm** through its independent 450 mm lift and 60° pitch range. The free NOTE cleared the front crossmember by at least **3.747 mm** and **3.635 mm** at the sampled pickup and carry poses on the two mirrored starts. The revised rails use an estimated 0.523 kg less aluminum at 2,700 kg/m³. The added plywood and foam volumes are 169.0 cm³ and 442.4 cm³. As-built mass, cover mass, weld strength and chassis attachment compliance still need measurement. [Reopened geometry and simulation results](maxswerve-front-crossmember.json)

The starting-size survey clips the installed solids to the first 190.5 mm above the carpet after excluding removable bumper parts, cover graphics and marked study geometry. A rectangle enclosing the resulting material measures 754.00 × 711.20 mm, with perimeter 2930.40 mm. Including moving bodies in this survey makes the rectangle a conservative upper bound on the fixed frame. The surveyed robot height is 968.32 mm.

**Starting configuration needs revision.** The modeled lift motor envelope projects 71.25 mm beyond the enclosing rectangle. The slotted launcher carrier projects 56.28 mm. Both exceed the 6.35 mm minor-protrusion allowance. Check the exact motor installation and repackage the outboard parts or verify a legal stowed pose. [Native size survey](../native/swerve_starting_size.py) · [2024 manual, R101–R104](https://firstfrc.blob.core.windows.net/frc2024/Manual/HTML/2024GameManual.htm#R101)

### Public R7 motion replay

The showcase displays team-authored solids from the reopened R7 archive. The recorded D1 run used the earlier articulated swerve archive. Both use the robot-floor datum and the cradle pivot at (178, 0, 25.4) mm. The replay carries the measured lift, pitch, eight wheel coordinates and NOTE pose through the whole-ring opening event. Chassis and mechanism coordinates hold at release during the last segment of NOTE flight. The count occurs later in the field simulation.

Fusion compares the two archives in world coordinates before the animation is built. Each visible moving article must match by occurrence path, body and motion group. The solid bounds and the volume of their symmetric difference are checked for each pair. The public mesh export selects team-authored parts before tessellation. Four cylindrical tire envelopes use the measured centers, outside diameters, tread widths and roll axes. A light sector makes each tire's rotation visible.

To regenerate, use `export_swerve.extract(display_archive, display_sha, folder)` and `compare_swerve_motion.compare(display_archive, plant_archive, display_sha, plant_sha)` inside Fusion with `scripts/showcase` on the module path. Write the comparison result to a private JSON file. Resolve the archive digests from [the R7 build record](maxswerve-front-crossmember.json) and [the articulated plant layout](maxswerve-layout.json). Build the public display and attach the D1 carry, release and field traces using the verified [first-shot record](../../sim/pressure/evidence/maxswerve-development-shot.json).

```text
python scripts/showcase/build_model.py <folder>/manifest.json <folder>/public-floor.glb --public
python scripts/showcase/register_swerve_replay.py <folder> <comparison.json> <registration.json>
python scripts/showcase/build_replay.py <folder>/public-floor.glb docs/detailed-design/archive/draft-001/media/swerve-r7.glb sim/pressure/evidence/maxswerve-development-shot.json <carry> <shot> <field> --registered-motion <registration.json>
```

The [public model receipt](../../docs/detailed-design/archive/draft-001/media/swerve-r7.json) identifies the CAD revisions, trace digests and displayed geometry. R7 shot clearance and mission repeatability require their own checks.
