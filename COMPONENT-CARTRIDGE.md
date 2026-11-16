# 2024 real-component increment: wheel support cartridge

This is a bounded, original interface study supporting the A2 launcher architecture. It is not a complete shooter or a drop-in replacement for the abstract roller. The existing A2 mission remains unchanged.

## What is new, versus reused evidence

Earlier work already imported the AndyMark am-3608 bare wheel and separately explored REV MAXSpline wheel/hub/support stacks for 2026. This increment combines the actual am-3608 with two mirrored REV-21-1916 rounded-hex bearings and REV-21-2900 collars, an original 140 mm rounded-hex shaft profile, two bored 80 x 80 x 3.175 mm plates and two original inner-race spacers. It checks the actual wheel/shaft clocking and both support interfaces together. It does not transplant a team's mechanism or reuse the unrelated 2026 contact result.

The generator downloads only pinned manufacturer STEP files into an explicitly supplied private cache. Source hashes and numeric native observations are retained in `cad/components/cartridge-evidence.json`. Exported STEP contains only the five original shaft/plate/spacer solids, reopened after writing. Vendor wheel/bearing/collar solids are not redistributed. Public access to manufacturer CAD is not treated as a license to republish it.

## Actual result

At the selected zero-degree hex phase, tested native wheel/shaft and mirrored bearing/collar/plate/spacer pairs have zero common volume. A deliberate 30-degree shaft phase overlaps the actual wheel by 146.351 mm3. This discriminating control verifies the check sees a real orientation error; zero common volume does not establish tolerance, bearing preload, contact area, fit or strength.

The bare wheel's radius is 46.99 mm, not the A2 model's 50.8 mm tread radius. The drawing's .501-.506 inch hex bore range also differs from the simplified .500 inch CAD. Bare wheel inertia is approximately 0.000199744 kg m2 at explicitly assumed uniform aluminum density 2700 kg/m3. The report adds the original shaft/spacer inertias separately and excludes stationary plates from rotating inertia. Neither result replaces the simulation's roller inertia until tread, drive components and contact distribution are resolved.

Five original components plus the bare wheel account for approximately 0.319596 kg. This is a partial mass ledger, not total assembly mass or robot center of gravity. Bearing, collar, motor, transmission, tread, fastener and electrical masses remain excluded. Symmetric partial placement must not be mistaken for a full robot balance assessment.

## Interfaces and next bounded build

- Bearing seats: original nominal 28.6 mm plate bore; 5 mm mounting holes on 50.8 mm spacing. No manufacturing tolerances or shaft finish specified. Outer-race retention and selected fastener preload are unresolved.
- Shaft: nominal rounded-hex profile, 12.7 mm flats clipped by 13.75 mm corner diameter. Material/stock, end machining, runout and fit need a real selected stock specification.
- Drive: only 19.68125 mm nominal axial allowance beyond each collar. Select and place the NEO-to-hex transmission, exact ratio, pulley/gearbox, shaft retention and service tool before claiming an installed motor. Direct coupling is not assumed.
- Contact: select actual tread and attachment, measure loaded radius/compression and recompute inertia. The bare wheel is not a compliant shooter roller.
- Mounting: connect original plates to the robot with verified load paths and tool access. The current two mounting holes do not imply adequate strength.
- Electrical: place the selected controller, phase/sensor connections, wire routing and strain relief; model supply/current/heat with actual component settings. No hardware wiring or operation is performed here.

## Reproduce

`python cad/components/build_cartridge.py --vendor-cache PATH_TO_PRIVATE_CACHE --output artifacts/component-cartridge`

Requires OCP. Source byte changes fail closed. Native imports and common-volume checks are executed, not fabricated. Ordinary regression tests cover source binding, original solid validity, parameter-driven plate geometry and recorded negative-control behavior; they are not a new download replay. Material/load, speed, thermal, fatigue, electrical safety and fabrication acceptance remain open.

Sources: [AndyMark Performance Wheels](https://andymark.com/products/performance-wheels), [REV bearings](https://www.revrobotics.com/ION-Bearings/), [REV shaft guidance](https://docs.revrobotics.com/ion-build/motion/shafts-spacers-collars). Exact CAD URLs/hashes are in the editable configuration. Historical 2024 event applicability is not established by a current catalog.

## Tool enhancement findings

A reusable component pipeline needs source-pinned native interfaces, explicit unit/datums, original-only export filtering, positive interference controls, both mirrored supports and partial mass/inertia ledgers. Replacing a rendered box with a catalog name is not integration. Retain separate physical interface, contact, fabrication and electrical gates. Current cache inputs do not include tolerances or license grants; fail rather than silently assume them.

## Motor/transmission packaging experiment

`cad/components/a2-transmission.json` selects a provisional 1:1 belt path: NEO V1.1, REV-21-2200 12T keyed driver, REV-21-2201 12T half-hex driven pulley, and REV-21-4040 40T RT25 belt. Equal pulleys and a 6.35 mm pitch give 88.9 mm ideal shaft spacing. This is a historical motor/model continuity candidate, not a purchase recommendation; the vendor lists V1.1 as discontinued.

The driven pulley is centered at Z60 on the cartridge axis; the motor axis is X88.9 and its native face datum is placed at Z45. Driver pulley center is Z60. This is an editable nominal placement, not a mounted or tensioned assembly. The driven pulley occupies approximately Z50.653..69.347, leaving only 0.334 mm nominal collar gap and 0.653 mm shaft-end margin. These narrow allowances require revision or a tolerance/retention design, not acceptance.

Reproduce with `python cad/components/check_transmission.py --vendor-cache <private-cache> --output artifacts/transmission/report.json`. The checked-in receipt pins both configs and all six vendor inputs. Every one of the 78 pair combinations is tested, including the motor, pulleys and complete prior cartridge. A 30-degree driven-pulley clocking is deliberately rejected. Zero motor/driver interference does **not** establish key engagement: a separate key, screw/washer stack and axial retention are still missing.

Official [NEO mechanical data](https://docs.revrobotics.com/brushless/neo/v1.1), [pulley data](https://www.revrobotics.com/RT25-Pulleys/) and [belt data](https://www.revrobotics.com/RT25-Belts-1/2in-Width/) give masses of 425, 15, 11 and 11 g, respectively. Their 462 g subtotal is not the complete cartridge mass and supplies neither rotor inertia nor installed center of gravity. Do not apply uniform aluminum density to the motor. No A2 simulation parameters change here.

Next gate: source-backed motor mounting pattern and #10-32 stack, explicit key/retention, adjustable belt tension, bearing outer-race retention, guard/service space, actual tread and its attachment. Belt teeth/contact, tension loads, tolerances, balance, shaft bending and operating-speed acceptance remain unverified. Vendor solids stay private; this increment exports none.
