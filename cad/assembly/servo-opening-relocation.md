# Positive-side powered opening relocation

This additive candidate replaces no canonical geometry. The original rejected `f1015f7` placement and receipt remain unchanged.

## Actual changes
The SRS V1 output axis is moved to local X130,Y196.25,Z185 mm. A first exploratory X130/Z115 route cleared the raised robot but hit the floor chassis crossmember and fastening bridge; that route is rejected. Raising the servo above that member and routing the follower past the backbone end removes the previous 480 mm3 backbone intersection without cutting away the backbone.

The original mounting frame is supported by a welded-stock candidate from the fixed outer anchor. Its contact with the frame and anchor is a proposed weld interface, not a completed strength or fabrication approval. The follower foot seats on the moving side beam; no volume overlap is silently called a weld.

An original two-hole flange, hollow axial neck and eccentric cam couple to REV-41-1363. Manufacturer STEP hole radius1.25 mm and opposite source centers (+/-4,+/-6.92820323) mm locate M2 shank proxies. The cam and follower are axially separated from the horn disc. One millimetre of axial relief eliminates actual native case-cap overlap. The remaining native shaft/horn overlap is preserved as UNKNOWN: the supplier horn model uses a simplified circular spline bore and does not establish 25-tooth engagement, insertion depth, shaft screw retention or loaded fit. It is not exempted as acceptable contact.

## Evidence and limits
The full robot screen samples five cam angles, both zero travel and the contact-boundary travel, in floor, rotated and handoff poses. These are separated-follower and active-boundary geometric samples, not a dynamic trajectory or continuous swept-volume proof. The native interface screen checks each supplier solid independently against every new authored interface part, and every pair of authored parts without exclusions. Both scripts pin all imported geometry/verdict helpers.

The native servo and horn stay private inputs; only original mounting frame, support, follower and flange/neck/cam geometry may be exported. Source URLs and SHA256 values are in the native receipt. No source geometry is redistributed.

Still open: shaft/horn spline seating and center screw, M4 and M2 heads/nuts/locking and tool access, support/foot weld sizing and fatigue, bracket deflection, opposite-side routing, continuous sweep, and installed regulator/wiring. No installed/load-qualified or canonical-adoption claim is made.

## Dynamics and power contract
The cam imposes only `0 <= u <= .75*(1-cos(theta))` mm. It does not command u: spring, NOTE, stops, inertia and nonnegative contact reaction determine actual displacement. Separation is allowed. No positive latch is claimed; holding is powered and command/power loss must preserve mechanical state and release stored spring energy through the simulated mechanism.

REV-11-1144 supplies 6V; SRS V1 is not connected directly to12V. The source snapshot documents2A stall current per V1 and15A aggregate regulator shutdown, not active3A per-port limiting or a continuous stall rating. Ideal nominal two-spring opening work remains0.0154326J, not total electrical energy. Regulator loss, both servos, idle/holding draw, current protection and servo work must enter the shared battery ledger before runtime adoption.

## Exploratory friction
Official horn CAD was recovered at the manufacturer `/content/cad/REV-41-1363.STEP` route after the product page had disappeared. The installed OCP API uses `TopoDS.Face`, not `Face_s`. Chassis envelopes, not only adjacent mechanism geometry, drove the height change. A zero external-interference report did not eliminate the separate native spline-fit uncertainty.
