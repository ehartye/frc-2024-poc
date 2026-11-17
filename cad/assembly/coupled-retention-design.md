# Coupled acquisition to retention: geometry and physical design gate

The exact source-bound modeled approach ends at robot-relative NOTE X228.67604936032027 mm, or +50.67604936032026 mm from the cradle origin. This state is not capture and its recorded velocities/rotors remain nonzero.

## Findings

The nominal straight belt contact face spans cradle X -19.05 to146.05 mm. At the recorded longitudinal offset the potential midheight NOTE contact footprint spans33.0322 to68.3199 mm, well within that face. Thirty material-classified points (15 per side sharing one side preload budget) remain valid through15 conditional carried poses.

That does NOT make the pose usable for lifting. The full rigid NOTE intersects the fixed front bumper backing, foam noodle representations and original backing support during10/20/30-degree pitch. At10 degrees the backing common volume is approximately178353 mm3, far beyond a numerical grazing contact. Indexing the NOTE farther inward while remaining floor-flat must precede pitch. A lower position target is a controller/design candidate, not permission to reset the piece or assume it reaches that target.

Separately, the end-state optimistic static friction bound is2.106256 N versus2.304563 N weight. The model cannot lift even when all friction is allocated vertically. Moving the piece longitudinally does not itself repair that force deficit.

## Physical concepts: compliant mounting, not arbitrary foam stiffness

Explore two spring-biased backing rollers per side, mounted behind the straight inward belt run, on one guided transverse yoke with adjustable preload and hard travel stops. The rollers should push on the BACK of the belt, not touch bare NOTE, and the existing drive pulleys should remain registered. The yoke must transmit spring reaction into the original cradle stock through real brackets, rather than unsupported floating rollers. A compression spring's measured installed force/travel and belt-tension reaction determine available NOTE normal force; do not substitute a larger numerical foam stiffness or presume spring force equals NOTE force.

The current nominal loop cavity is only about22.255 mm wide between its two inner belt surfaces. A provisional16 mm backing-roller envelope has geometric room, but its exact shaft, bearing, yoke and travel are NOT selected or qualified. Its placement must follow the subsequently verified inward indexing window, not the presently blocked +50.676 mm pitch state. The roller-support yoke must stay above floor clearance, and all new structure must be swept with the complete NOTE and robot.

A supported under-NOTE cradle is the alternative if friction-only retention remains fragile. At floor pickup, the NOTE is already tangent to the ground, so an added underside rail cannot simply appear under it: a ramp/rolling transition with modeled floor contact and drive work is required. That is a larger intake-path revision, not a free support force.

## Timing-belt compatibility gate and preferred fallback

The timing belt has an internal toothed interface. A small smooth backing roller must not be assumed compatible or substituted for a matching idler. The current source-sized 12T pulley has a26.4 mm flange diameter, larger than the22.255 mm loop cavity. The provisional16 mm roller is therefore only a packaging probe, not a selected valid component.

The preferred fallback is a spring-biased COMPLETE belt cassette: both registered pulleys, bearings, motor and transfer drive move together on a short guided transverse carrier with adjustable stops. This preserves the timing-belt path instead of pressing an unverified roller onto teeth. The cassette must be separated from the rigid cradle/pivot structure using actual guided mounts, with measured preload and drive/structural reaction accounted for. This requires new mounting CAD; it is not implemented by changing the current NOTE stiffness or moving the existing fused weldment indiscriminately.

## Next gates in order
1. Find an inward indexing window that clears the complete front backing/support throughout pitch. Continue the actual coupled state to that window; do not teleport or silently reuse old patch/preload data.
2. Design and source the spring/yoke/backing-roller interfaces, with controlled installed preload, travel stops and measurable calibration targets. Preserve friction/foam/wear uncertainty.
3. Recompute the finite contact wrench including gravity, commanded acceleration and torque demands, and actual belt drive/braking behavior. A static force sum is necessary but insufficient.
4. Screen new hardware/NOTE motion and resolve shaft retention; keep94ab67d bearing seat candidate separate until deliberately reviewed/adopted.

These are design hypotheses and finite native geometry witnesses, not a fabricated or calibrated retention mechanism. Current canonical CAD and runtime force parameters are unchanged.

## Sampled inward-index comparison

A separate90-case full represented-assembly screen compared offsets0,5,10,20,30 and40 mm at15 prescribed pitch/lift poses. Only0 mm was free of unintended represented contact throughout that sample set. Even+5 mm hit the rigid front backing support at10 degrees. The original support occupies a3.175 mm plate just inside the front frame perimeter; it must not be waived as foam or deleted without restoring the backing load path.

Consequently, there is not yet a broad accepted indexing corridor. X178 mm can be a provisional continued-control target, not a position reset or guarantee. A usable design needs a tolerance-aware corridor, actual low-speed state convergence and possibly a deliberate support/pitch-path revision. Rearward offsets also change ground clearance during pitch and cannot be assumed free. Force capacity remains independently insufficient at the present calibration hypothesis.

## Compare deliberate compliance with a translated cradle

Rigid common volume is a geometric contact witness, not a safety verdict and not proof that compliant foam cannot pass. At10 degrees, offset+5 mm produces879.19 mm3 of support occupancy; offset+10 produces6894.01 mm3 at the support and2990.93 mm3 at the backing. The much larger +50.676 mm case must not be used to dismiss the smaller indexed contact or to calibrate its force.

A deliberately rounded guide/contact surface at the early-pitch backing interface is worth testing before adding another whole-cradle axis. It needs an actual guide shape and attachment, contact location/normal, finite foam compression/friction law, pitch-actuator work and force sensitivity. Merely tagging the existing overlap as intended does not implement any of these. The current raw plate/backing edges are not an accepted guiding surface, and occupancy volume is not deformation energy or measured strain.

The alternate is a short longitudinal cradle slide: translate backward while the NOTE is floor-supported, pitch/lift through a larger clearance corridor, then return forward for launcher registration. It adds a real actuator, guides, stops, moving load path and registration state. Its command cannot carry the NOTE as an ideal attachment; ground drag, belt forces and remaining velocities still determine piece motion. If the NOTE is offset from the rotation pivot, the lift trajectory also needs to account for that offset to avoid ground penetration. Neither alternative is adopted yet.

Current preference: compare the small indexed-state contact against an explicitly rounded, bounded compliant guide first, keeping a translating cradle as the geometry-clean but mechanically more complex alternative. Preload capacity and source-compatible spring cassette mounts remain independent gates.
