# Additive lower support anchorage candidate

`lower_support_mounts.apply(parts)` deliberately operates on a copy of the current whole-robot part list. It drills the four original posts and two authored bridge tubes and adds four original L brackets plus sixteen selected screw/head/nut stacks (52 added solids). It does not silently change the canonical export. The additional hardware STEP alone is not a replacement whole-robot assembly: the mating posts and bridge bores must also come from `apply`.

Each 3.175 mm bracket mates to a 3.175 mm tube wall. Nominal #10-32 x 0.75 inch am-1047 screws and am-1042 nuts follow the existing sourced fastener representations. Two leg bolts connect each guide/shooter post; two foot bolts connect each bracket to its transverse bridge stub. Lower guide brackets point outward and shooter brackets inward. Bracket stock is an original proposed aluminum weldment/formed-angle geometry, material grade and manufacturing choice unresolved.

The initial foot-hole placement was rejected because nuts intersected tube sidewalls. Upper guide screw heads also hit the floor-position carriage plate. The rejected native receipt is preserved. Revised foot holes run along the bridge axis, and leg bolts sit lower. The native positive check covers added hardware against the modified floor assembly and against moving components at 15 sampled poses. The undrilled-post negative control must intersect the nominal screw. This is not a continuous motion or load qualification.

Install the internal nuts through open tube ends before upper portal/side-rail closure. Actual wrench reach, sequence, tool clearance, preload, bracket bending, local wall bearing, edge-distance suitability and impact loads remain unresolved. A geometric absence of interference does not establish any of them. Upper portal attachments, carriage yoke, chain attachment/tensioning and keyed torque transfer are separate open interfaces.

Run `python cad/assembly/lower_support_mounts.py`. The receipt binds current assembly inputs, this script and the exported/reopened 52-root hardware STEP. These are original representations, not redistributed supplier native CAD.
