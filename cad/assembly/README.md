# 2024 integrated component CAD candidate

<!-- cad-still -->
[![Preliminary design of the robot](../preliminary-design/media/guided-cassette-preliminary.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Project CAD · Preliminary design of the robot · Guided cassette 1.0 · effective.** The robot is at the start of a cycle, with a floor NOTE ahead. The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. The still shows the layout solids of the preliminary design review in its opening pose. [Open the design review](https://ehartye.github.io/frc-2024-poc/preliminary-design/)
<!-- /cad-still -->

Run `python cad/assembly/build_robot.py`. Output: `artifacts/whole-robot/original-component-assembly.step`, `a2-layout.png`, and `report.json`.

This joins the source-sized AM14U4 six-wheel drivetrain and electrical package with four original shooter cartridge representations. It replaces the old 710 mm frame and 600 mm track with measured 719.074 by 711.2 mm chassis and 579.60768 mm track. Fixed backing/noodle candidates follow the new perimeter: nominal19.05mm backing,127mm height,two63.5mm noodles,top185.5mm plus1.5mm cloth allowance. Flat floor-tangent NOTE underpass has only6.2mm nominal margin;5degree pitched NOTE fails. This is not bumper support/corner construction or rules acceptance. No broad bumper gap is proposed.

The shooter uses the independently checked 45.72 mm tread seat and nominal 48.26 mm finished radius. Drilled bearing plates, drilled motor sliders, slotted carrier bridges and smooth belt pitch-band representations are editable original geometry. Supplier native CAD is not exported. The wheel shell is a simplified representation, not the native extrusion; belt teeth and screw threads are omitted.

## Unfinished interfaces, visible rather than hidden

- Shooter-to-frame anchorage remains unfinished. Nominal bridge spacers, slider clamp screw/head/nut envelopes and an integral jack-lug proposal now show an adjustment load path. They are not source-qualified fastener assemblies: nut/head dimensions, tapped engagement, travel, installation and torque remain to be checked. No plate is accepted as load-bearing merely because it is drawn.
- The old telescoping intake/transfer is green and named UNRESOLVED. It is retained as spatial context, not completed component CAD or a functioning intake.
- The proposed rear crossmember was excluded because it intersected the electrical deck; source-derived front/middle mounting adapters are a separate ongoing task.
- Selected tread rivet fails the nominal bare-stack grip range. No attachment or safe rotational-speed acceptance.
- No whole-assembly collision, sweep, structure, electrical harness or service-access acceptance.
- Frame changes invalidate any old frame/perimeter-based screen unless explicitly rerun. No previous simulation acceptance transfers automatically.

## Mass bookkeeping

The report separates supplier-known masses (with envelope-center placement assumptions) from estimated mass of OUR drilled aluminum plates using CAD volume and assumed 2700 kg/m3. Supplier bounding boxes are never filled with aluminum density. Whole-robot CoG remains null until missing hardware and COTS masses are represented. Partial centroids are not stability predictions.

## Rejected controls retained

Independent review of4989180 found each bearing-carrier bridge bolt crossed an undrilled bridge (about58.08mm3 of metal). The current revision drills registered5.2mm clearance holes; actual common-volume tests check all8 bolts and require a1mm displaced-bolt negative control to collide. This is geometric registration, not fastener/load acceptance.

The initial91-pose full-torus intake study rejects unchanged front endrail/crossplate and both transverse support tubes in25poses. A separate `--open-throat` study cuts a400mm-wide central throat in those four items only, retaining the baseline result. This is a proposed frame modification with lost transverse load paths, not permission to remove a bumper segment or proof of structure. The mechanism, modified-native interfaces and continuous swept clearance remain gates.

## Current intake architecture experiment

A level floor NOTE passes below the fixed front bumper, stages atX178mm, rotates forward-up to60degrees while remaining floor-tangent, then lifts to join the shooter at local guide coordinate96mm. The revised screen includes a full-width original3.175mm rigid front backing-support plate immediately behind the plywood, so an empty chassis cut is not mistaken for bumper support. Plate end attachment, stiffness, impact loads and rules inspection remain unresolved. This plate is a structural proposal, not a qualified substitute for the original endrail.

The sampled rigid-NOTE route is92poses (not continuous) and excludes shooter cartridge/web contact until the final handoff study. It does not prove pickup, floor traction, retention or a workable elevator. Actual actuator, carriage, rotating cradle and bearing selection follows this spatial test; the old green telescoping hardware remains rejected/unresolved context, not the new mechanism.
