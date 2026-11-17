# Additive flanged-bearing seating correction

The current intake bearing envelope is undersized. The pinned REV-21-1916 source has a31.115mm flange diameter and7.9375mm total axial envelope, rather than the old28.575mm by6.35mm cylinder. The manufacturer's [June2022 drawing](https://www.revrobotics.com/content/docs/REV-21-1916-DR.pdf) identifies a13.75mm round bore for a rounded-hex shaft. The new original stepped representation keeps that round bore; it is not a hex-keyed torque coupling.

The actual annular carrier spans localZ−4.7625 to−1.5875mm. Two native seating arrangements have zero material overlap and effectively zero seat gap:
- Lower flange below carrier: flip bearing180degrees aboutX and shift its center−2.38125mm from the old nominal center.
- Upper flange above carrier: do not flip; shift center−3.96875mm.

The unraised lower arrangement leaves only0.65mm nominal floor clearance and insufficient lower-stub extension. A tested alternative raises the lower carrier2mm while maintaining its flange-seat relation. The bearing center then moves−0.38125mm relative to the OLD lower center. Ground clearance becomes2.65mm and the existing shaft end extends0.25mm beyond the bearing's lowest face. These small nominal allowances require tolerance, carpet/deflection, retention and installation review. No shaft or carrier relocation has been silently adopted.

Wrong axial seating by1mm intersects native material by117.17 cubic millimeters. An oversized rounded-hex shaft intersects by11.52 cubic millimeters. Correct-size shaft checks at0/30degrees both clear, which is expected for the round bore and is not keyed torque-transfer proof.

`bearing_seat_candidate.py` exports only three original study parts: carrier, stepped bearing representation and test shaft. It does not redistribute supplier CAD. The stepped representation omits detailed grooves, fillets and internals and is not automatically a conservative material bound.

Next gate: integrate a selected orientation and raised lower carrier into an additive full-assembly candidate, check actual retained NOTE pose and full motion/path, then finish axial retention. Canonical f2be99b geometry and current contact models remain unchanged pending those checks.
