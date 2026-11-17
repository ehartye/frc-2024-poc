# First actual servo mounting placement: rejected

The privately pinned REV-41-1097 STEP supplies four ear-slot axes at source X +/-5 mm, Y +/-24.75 mm, radius2.25 mm, nominal underside plane Z=-29.25 mm. The original drilled mounting frame and four M4 shank proxies have zero native material overlap. This verifies nominal hole registration only; screw heads/nuts, bearing pressure and tolerance are not complete.

Source -Z output becomes cradle +X. The selected body position puts the output near local X40,Y196.25,Z140 mm. A full-body source envelope and original follower are now tested with the actual robot and moving cassette, rather than floating coupons. Six endpoint checks retain11 raw contacts in each floor state and8 in each raised state. In particular, the authored follower crosses the fixed backbone by480 mm3. The servo frame also hits the pinch motor/plate. This placement is rejected; do not adopt it or dismiss the source-envelope conflicts without native witnesses.

The follower/base overlap with the moving weldment represents an unfinished proposed fabrication joint, not a demonstrated fastener/load path. Servo-to-horn-to-cam coupling and bracket-to-backbone support also remain open. Next revise lateral follower routing and actuator position while preserving native mounting axes; do not remove the fixed backbone merely for clearance.

## Runtime constraint
For this cam orientation, nonpenetration imposes u <=0.75*(1-cos(theta)) mm. Equality holds only while follower contact is active. NOTE reaction may hold the cassette at smaller u, separated from the cam. Contact reaction must be nonnegative with complementarity; spring, guide friction, inertia, stops and NOTE forces determine actualu. Servo angle cannot resetu. Dead center is not a latch; powered holding, command loss and returned spring energy require explicit dynamics.

Only positive-side candidate is studied. No full continuous sweep, moving added-part mutual audit, opposite-side qualification, installed SPM or physical holding acceptance is claimed. Canonical f2be99b remains unchanged.
