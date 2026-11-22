# Preliminary design · retained cassette and shared-lift comparison

**Current decision.** The active greenfield layout, frozen scoring campaign, revised PARK route and rejected inward climb are documented in the [current preliminary review](endgame-route-01/REVIEW.md). The earlier comparisons below preserve their original scope.


Open the [retained cassette](http://192.168.1.129:8764/) or the [passive cover and shared lift](http://192.168.1.129:8764/?concept=shared). Both use the same chassis coordinates. The first is the working baseline. The second tests an architecture with one fewer powered lift and a passive NOTE keeper.

The baseline has 126 closed solids. The replaceable rear pad seats on its backing. Each bracket contacts the backing across 389.321 mm² and the crossbar across 170.921 mm². Each end of the crossbar bears on 80 mm² of a cassette side beam. The model includes two backing screws and two through bolts with nuts. Weld sizing, joint strength and durability remain open. See [stop-support.json](stop-support.json).

The baseline pin and upper bridge kept a freely moving 0.25 kg rigid ring in the cassette at five powerless angles on both refinements. The largest translation was 26.43 mm. Removing the pin and bridge allowed more than one metre of downhill travel at both extreme angles. The powered center-belt approach failed to seat the same size ring. The animation with the imposed tipping motion is a geometry check. The separate free-ring trial is the acquisition discriminator. See [free-hold.json](free-hold.json) and [powered floor trial](pickup-64-0.0005-1-0-0-0.75.json).

The alternative's windowed 2 mm polycarbonate sheet weighs an estimated 314 g before mounts. It leaves 4.2 mm above the nominal seated foam. A flap obstructs a 50 mm mouthward translation in its resting position and clears the same translated ring when folded up. The inward admission cycle, spring, cam, hinge durability, deflection and release with a real NOTE remain open. The earlier full panel estimate was 389 g. The relieved side rails remove two 1,536 mm³ clashes introduced by sharing the space with the lift yoke. See [shared-view-check.json](shared-view-check.json).

The shared-lift alternative uses the cassette as the climb arm. Its crosspiece moves from a candidate chain-contact angle of 70.43° to 165°. At an assumed 68.56 kg running mass the maximum static shoulder moment due to robot weight alone is 171.15 N·m. The geometrical motion has no positive overlap in the scoped comparisons. The single-support loaded posture can put an actual bumper vertex 28.73 mm below the carpet at one interior CG assumption. The CG is unmeasured. A gated hook, passive support after cutoff, verified loaded attitude, slider restraint and rated paths from the hook to both sills are required for a climb. See [shared-arm.json](shared-arm.json) and [loaded-pose.json](loaded-pose.json).

The empty horizontal robot reached the archived Blue PARK pose in 10.53 s from the recorded Blue 1 position with 24 s remaining. The path clears conservative envelopes of all five peers and the stage legs. The direct path collides with two legs and a peer. Overhead, fixturing and match conditions remain assumptions. See [park-rehearsal.json](park-rehearsal.json).

The browser inspection tests cover all baseline parts through paginated leaders, model rotation, component selection and both review URLs. All season geometry and experimental inputs stay in the private project review. Detailed design remains paused.
