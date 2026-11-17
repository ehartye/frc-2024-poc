# Original MAXPlanetary output-coupling candidate

This additive study does not replace the whole-robot gearbox envelope or qualify a drivetrain. It uses the private, pinned 2022 manufacturer assembly's 35-solid output subset to check an original rounded hex shaft and nominal blind-retention screw.

The selected shaft enters the 11.1125 mm socket by 10.5 mm, leaving 0.6125 mm nominal floor clearance. Correct clocking has no native material intersections; a 30-degree clocking error intersects about 73.89 cubic millimeters. A nominal #10-32 by 3/8-inch screw crosses the manufacturer's 5 mm clearance hole and engages the proposed pilot bore over 7.6425 mm. The declared thread-zone overlap is intentional simplified thread geometry, not an interference waiver for other parts or proof of thread strength.

The original-only export has three reopened roots. The source receipt records per-solid checks, dependency hashes and before/after stability. Supplier geometry is not redistributed. Run `python cad/components/gearbox_output_coupling.py --native-output <private-output-subset.step>` with the pinned source subset.

Remaining gates: select the actual screw and material; validate tolerances, pilot/tap specification, torque and engagement; design sprocket/collar retention and gearbox mounts; screen installation tools and whole-robot motion. Access to the retention screw is assumed before gearbox stage assembly. This is a nominal interface candidate, not fabrication or load acceptance.

Primary source: https://www.revrobotics.com/content/cad/MAXPlanetary%202-Stage.STEP
Output drawing: https://www.revrobotics.com/content/docs/REV-21-2104-DR.pdf
