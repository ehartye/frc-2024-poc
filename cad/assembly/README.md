# 2024 integrated component CAD candidate

Run `python cad/assembly/build_robot.py`. Output: `artifacts/whole-robot/original-component-assembly.step`, `a2-layout.png`, and `report.json`.

This joins the source-sized AM14U4 six-wheel drivetrain and electrical package with four original shooter cartridge representations. It replaces the old 710 mm frame and 600 mm track with measured 719.074 by 711.2 mm chassis and 579.60768 mm track. Bumper envelopes follow the new perimeter with provisional 75 mm thickness; this is not a bumper construction or rules acceptance.

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
