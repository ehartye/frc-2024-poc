# 2024 MAXSwerve frame and electronics supports

The front of the frame stays open for floor pickup. Two longitudinal rails and one rear rail bolt directly between the REV-21-3005 housing tabs. Each front module receives one rail end. Each rear module receives a side rail and one end of the rear rail. A transverse member joins the side rails behind the intake. Two short stringers connect it to the rear rail. Four open pedestals support the retained electronics beams at their existing attachment stations. The side openings in the pedestals provide access to the retained deck locknuts.

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

In the reopened robot, all twelve bolt axes registered through both supplier plates. Of the 24 measured passages, eight included the short relief. A 4.98 mm gauge cleared every passage through its full depth. The 5.20 mm oversize control intersected at least 3.88 mm³ of plate material in every passage. A 0.02 mm intrusion at the fastener and rail faces measured minimum bearing areas of 29.69 and 43.19 mm² respectively. Each of the four pedestal caps seated on approximately 418.22 mm² of the retained beam.

The revised robot retires 83 six-wheel chassis occurrences containing 84 solids. The static check found zero frame intersections above 0.01 mm³. The 188 moving bodies maintained at least 82.30 mm modeled separation from the new frame across independent lift and pitch. Across the two recorded floor-NOTE pickup and carry runs, the smallest sampled NOTE-to-frame bound was 120.97 mm. Front lift and bumper connections, harness routing and loaded structural checks are outstanding. [Native measurements and test bounds](maxswerve-supported-frame.json)

Run `cad/native/build_swerve_frame.py` from Fusion's Scripts interface with its three sibling authoring and inspection modules available. Choose a new local F3D filename. The script builds the editable tube, drilling, sleeve, fastener and pedestal features, then reopens the archive and measures the result. The screw and locknut solids use the project’s 10-32 exterior dimensions. Nominal geometry omits helical threads and socket-tool tolerances. [Fastener model basis](../electrical/fasteners.py)

The installed-robot check compares the new solids with the four articulated supplier modules and the retained robot. The interval test sweeps the registered moving intake and lift through independent lift and pitch. The separate NOTE check reads each attained pickup and carry sample for the two development runs. [Installed solid check](../native/check_installed_swerve_frame.py) · [Trajectory check](../../sim/pressure/swerve_frame_note_clearance.py)
