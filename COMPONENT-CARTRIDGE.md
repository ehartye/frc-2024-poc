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

### Key and mount refinement

The preceding clearance-only placement did not establish key engagement. Adding an original nominal 2x2x14 mm key at the motor's +Y slot exposed 23.578 mm3 of interference with the originally unclocked driver. Rotating the driver 90 degrees aligns its slot with the key; both motor/key and driver/key common volumes are then checked alongside the rest of the assembly. The unclocked case remains a negative control. This is geometric slot occupancy, not a shear, tolerance or torque-transfer qualification.

An original 72x72x3.175 mm motor plate now has a 19.3 mm pilot opening and four cardinal 5.2 mm clearance holes on a 50.8 mm circle. The [V1.1 drawing](https://revrobotics.com/content/docs/REV-21-1650-V1.1-DR.pdf) distinguishes **5.5 mm front mounting-hole depth** from **12.7 mm shaft-end tapped-hole depth**. The product page's broad 0.5-inch mounting-depth statement must not override that feature-specific drawing. A provisional 5/16-inch screw through this plate and an assumed 1 mm washer projects 3.7625 mm, leaving 1.7375 mm nominal depth reserve. Thread strength, actual screw/washer tolerances and torque remain open; a 1/2-inch screw is an explicit failed depth control. Plate-to-robot mounting and belt-tension adjustment remain separate unfinished interfaces.

The replay also exports `original-motor-interfaces.step` beside its report, containing only the original plate and key envelopes. Keep generated files in `artifacts/`; do not commit vendor geometry. Tread, axial pulley retention and belt tension remain unresolved; these refinements do not validate an operational shooter.

The corrected key has 0 mm nearest-surface distance to the motor slot and 0.025 mm to the driver slot in pinned nominal CAD. This explicitly leaves clearance rather than claiming loaded flank contact; backlash, torque take-up and tolerance extremes are not simulated.


### Service-interface experiment, not assembly acceptance

Original tread packaging uses [gray am-2611a](https://andymark.com/products/gray-grippy-tread-1-in-wide-10-ft-long): nominal 25.4 mm width, 2.54 mm thickness, 60A. Finished nominal radius is 49.53 mm, not the A2 placeholder 50.8 mm. The page links an older green HAL-12E datasheet; equivalence to the gray product is unestablished, so its density/friction are not transferred. Seam, rivets, adhesive, compression, balance and mass remain absent.

Two axial spacer envelopes and an end washer explore motor-pulley retention. **This load path is rejected:** the inner spacer terminates on the stationary motor pilot. Zero native common volume does not prevent rubbing or inappropriate clamp loading. The generated geometry is retained as an explicit negative lesson, not a valid retention assembly. Use compatible shaft-retaining clips or a verified rotating shoulder before closing this gate. The shaft-end screw itself and threads are not modeled. Driven-shaft end retention is also unfinished.

A +/-1 mm motor-axis travel allocation is checked against the nominal tread at both endpoints. Required equal-pulley pitch length becomes 252..256 mm versus a nominal 254 mm belt. This does not authorize stretching the belt or establish tension: slot hardware, carrier attachment, clamping, jackscrew, stiffness and vendor tension limits are still needed. The complete nominal assembly has 171 common-volume pair checks; the current export includes six original interface/envelope solids only. This is a packaging experiment with an explicitly rejected retention load path, not a mounted, fastened operating assembly.


### Shaft-compatible retention candidate

The active experiment replaces the rejected spacer/washer stack with two privately imported [REV-29-2598 push-on rings](https://www.revrobotics.com/rev-21-2907/), one on either side of the driver pulley. The vendor identifies a 7.5 mm nominal bore for press-fit use on an 8 mm keyed shaft. The [drawing](https://www.revrobotics.com/content/docs/REV-21-2907-DR.pdf) gives 1.6 mm axial thickness. Nominal inner-ring lower Z49.0528 leaves 0.5528 mm from the stationary pilot top at Z48.5. Neither ring bears on the pilot. The proposed axial load path is pulley face -> ring -> gripping motor shaft, rather than stationary housing.

Native ring/shaft common volumes of approximately 0.8704 and 0.8342 mm3 are explicitly classified as intentional undeformed press-fit grip. They are not suppressed as numerical noise, nor does the model calculate installation deformation or pull-off force. All other pair checks still require near-zero volume. Existing original 14 mm key remains a custom-cut candidate, not the vendor's 20 mm key. Ring orientation, holding force, repeated replacement, axial running clearance and pulley-face contact under load remain unverified. Driven-shaft retention, support attachment and real belt-tension hardware remain open. Earlier rejected spacer evidence remains in git history and the named rejected geometry function; it is excluded from the active original-only export. Active export now contains three original solids: plate, key and tread envelope. Native rings remain private.


### Driven-shaft axial retention candidate

The transmission experiment now explicitly overrides the original shaft length to 160 mm and adds a native REV-21-2900-PK4 clamp collar centered at Z73, beyond the driven pulley. The original cartridge baseline remains unchanged and its mass ledger must not be reused for this longer shaft. The outer collar occupies Z69.825..76.175, entirely supported by the shaft ending at Z80. Nominal shaft beyond collar is 3.825 mm; outer pulley-to-collar gap is 0.4778 mm. With the existing inner collar's 0.33405 mm gap, nominal pulley axial travel is approximately 0.81185 mm. This travel is not accepted belt alignment or a manufacturing tolerance.

Proposed outward load path is pulley face -> external clamp collar -> shaft; inward load path is pulley face -> existing inner collar -> shaft. Existing bearings support the shaft, but collar clamp holding force, torque, axial clearance, shaft stock tolerances and screw access remain unverified. The prior 140 mm shaft lacked length for full external collar engagement and is not silently reused. All native pairs are rerun; only the two previously declared motor push-on-ring press fits may overlap. This advances retention packaging, not complete mounting or a qualified load path. Belt tension hardware and tread attachment remain unfinished.
