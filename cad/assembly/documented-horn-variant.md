# Manufacturer-documented retained horn variant

This is a new physical variant, not a waiver for the older1363 horn. It retains SRS V1 REV-41-1097 and selects horn V2 REV-41-1828 plus the exact REV-41-1359 M3x8 screw. The pinned2021â€“22 manufacturer BOM/assembly identifies all three. HornV2 is not ServoV2; no new-servo depth or current limit is borrowed. The guide's step prose calls the screw a T-slot screw while its parts/BOM identify1359; that inconsistency is recorded and the actual1359 CAD supplies the head shape.

The2020 drawing explicitly distinguishes M3x0.5 tapped holes from M3clearance holes. It provides40mmOD,8.5mmcenter counterbore and6061 material. Source-native1359 has an8mm shank and2mm-high hex head, replacing the previous conservative3mm head assumption. Native files remain private and hash-pinned.

## Real interfaces
The actual source servo shaft end plane is localX130.5. The registered horn's nominal bore-bottom face meets that plane, instead of choosing position merely to suppress volume overlap. The source center screw seats at the actual counterbore floorX131.75; its head remains about0.2496mm below the horn frontX134. This is nominal source geometry, not a tolerance-qualified margin. Manufacturer use of this exact pair and screw length resolves the sourcing/nominal-retention choice; pullout strength, torque and locking remain open.

A larger original flange uses the new source's actual tapped-hole pattern, with M3clearance holes, sufficient radial head-bearing allowance and a hollow neck. The cam and follower positions remain unchanged. Both actual attachment screws and actual center screw clear the authored adapter. The source horn and servo have zero common volume at the selected nominal pose; a0.5mm axial misplacement produces7.3368mm3, retained as a negative control. This does not certify spline-tooth loads or erase previous variant discrepancies.

Raw source center-screw/servo overlap7.8199mm3 and attachment-screw/horn overlap7.5595mm3 each remain recorded. Simplified major envelopes and tapped/spline surfaces are not a thread-contact mechanics model. Do not silently classify every overlap as acceptable, or claim that zero volume alone proves installation.

The full robot screen uses source-sized conservative representations at five cam angles and both separated/contact-boundary cassette positions in three robot poses. Original-only export excludes every supplier shape and hardware representation. No canonical adoption, dynamic retention, fatigue, locking, impact or continuous-sweep acceptance is claimed. The powered-holding/no-positive-latch6V/SPM/work contract continues unchanged.

A pocketed coarse horn representation failed containment by1.55458mm3 and is retained as a negative control. A filled disc/boss simplification also misses this outer blend material. External screening therefore uses an enclosing source AABB with0.001mm numerical padding and source-containment checks; native interface checks still use the actual private source geometry. No clearance result from the rejected under-enclosing representation is promoted.
