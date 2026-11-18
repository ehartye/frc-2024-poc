# Native track/tread candidate and retention limit

Status: **experimental native geometry; no integrated loading or shooting acceptance**.

The original colored robot and its driver are preserved. This separate candidate moves both intake tracks outward by 1 mm, rebuilding 34 affected shafts, pulleys, bearings, supports, pivot ends and drive parts. A 1.5 mm outer tread is added to each belt. This changes the nominal inner contact face from 176.922393 to 176.422393 mm from the centerline. The earlier 1 mm and unrelieved 1.5 mm candidates are retained privately. The final version adds a 0.5 mm return-run clearance pocket in each original weldment and moves each pivot shaft inner end from 203.5 to 206.2 mm.

The modification is original geometry, not a selected purchased belt or fabrication release. Tread stiffness, friction, adhesion, wear and density remain hypotheses. Native tread material mass is not calibrated. The saved F3D contains editable BRep components/BaseFeatures, not a reconstructed parametric feature history or a rigged robot.

## Native verification

All three variants were saved as native Fusion archives through the official MCP, separate from the original document. The final 734-body archive was independently reopened and every body checked for identity, solid validity, volume and bounds. The first 32-part replacement preserved all 700 unrelated bodies exactly; the second iteration changed only the two tread bodies and preserved the other 732. The final four-part relief iteration preserved its other 730 bodies with zero bounds/volume changes. No new feature errors were reported. The sanitized [receipt](evidence/track-tread-candidate-20261118.json) records exact source/archive/script hashes.

The complete floor route was checked at X658 through X178 mm every 1 mm, Y0 and Â±0.5 mm, Z25.4 mm. All 1,443 poses passed 11,948 native Boolean pairs with only explicitly identified belt/tread overlap. Analytic torus placement/volume, self-intersection, separated-object and reversed-operand controls passed. All 712 physical/source bodies participated, including hidden physical bodies. Four service envelopes and 18 explicitly visual bodies were excluded. Finite samples do not prove continuous full-robot clearance.

For the nominal flat NOTE within Â±0.5 mm lateral offset, an independent all-X separation bound places the pulley flanges at least 0.575283 mm outside the NOTE and the middle band at least 0.75 mm outside. A fresh private supplier-CAD check bounds the three axial bands by radii 13.159795, 11.834522 and 13.159788 mm, below the candidate envelopes of 13.2/12/13.2 mm; the source's full axial extent is covered. Rotation about the configured shaft cannot escape these radial bounds. Thus an unknown tooth phase no longer controls nominal clearance for this candidate. This is conditional on the specified shaft/axial placement and does not certify an actual installed part, tolerance, tilted NOTE or foam deformation.

The unrelieved candidate introduced eight belt/tread overlaps with the side weldments and pivot ends, up to 2862.430 mm³. It is rejected as an assembly candidate. The final relief revision has zero new or increased overlaps above 0.01 mm³ in 72 native broadphase pairs involving changed parts. Existing fits elsewhere are not thereby qualified. The pocket leaves about 3.997 mm of straight wall locally; reduced section and pivot engagement still require load/strength review.

## Force-based experiment

`sim/pressure/gravity_release.py` couples the existing finite-area pressure/friction law to one free six-DOF NOTE in MuJoCo 4.0.1. MuJoCo alone integrates its position and orientation. The fixed cradle and stationary belts supply reaction forces; no NOTE attachment or pose reset is used. The NOTE starts staged at X185 mm in a raised, stationary fixture, without a floor. This isolated release experiment does not simulate acquisition, lift motion, motors, drivetrain or shooting.

At the staged pose and a 2048Ã—2048 surface grid, the 1 mm tread gives a generous friction magnitude bound of 1.951900 N, below the 2.304563 N NOTE weight. The 1.5 mm tread raises that bound to 4.517818 N. A friction bound above weight is not proof of retention.

The 1.5 mm candidate still slips and drops 50 mm under the existing regularized Coulomb law. At a 256Ã—256 grid, steps of 0.2 and 0.1 ms give drop times 0.6770 and 0.6771 s; maximum energy-balance errors fall from 0.2445 to 0.1223 mJ. The 1 ms run gave 0.570 s and excessive rotational sensitivity; it is retained as an unconverged diagnostic, not used as the decisive timing result. At 512×512 and 0.2 ms, the drop occurs at 0.6744 s, a 2.6 ms grid difference. The no-contact control matches the semi-implicit Euler free-fall position/velocity to below 1e-12. The 512-grid run took 281.7 seconds for 0.6744 simulated seconds; this is not a real-time result.

The law uses `tanh(slip/scale)` and has no static sticking state. The result demonstrates this model's inability to support the proposed retention claim; it does not establish that a real compliant tread must drop a NOTE. Raising preload until a visual appears to hold would not resolve that uncertainty.

## Decision and remaining work

Keep the new native candidate for review because it removes the nominal pulley-envelope boundary. Do not promote it into a lifted driving/shooting demonstration yet. The next necessary evidence is a measured contact/retention model, including static sticking or a mechanically supported capture design, followed by the existing floor acquisition/transport contract and actual lift/launcher integration. Preserve the single physics authority when composing those stages. The isolated MuJoCo fixture does not replace or run alongside the planar drivetrain integrator in a robot simulation.

Physics, voltage, friction, weight distribution and tread material remain uncalibrated; thermal behavior is unmodeled. The existing interactive driver still shows static mechanisms and does not pick up or shoot. Historical A2 exits do not validate this candidate. No new GUI verification or merge was performed.

## Reproduction

Use the existing OCP and MuJoCo/NumPy environments; no new stack is needed. Outputs must be fresh private directories. The private delivery packet contains all three F3Ds, native execution/registration scripts and receipts, unsuccessful attempts, supplier containment evidence and complete physics traces. Do not commit those assets or raw inventories.

```powershell
python -B cad/assembly/build_track_tread_candidate.py --source-manifest <private-original-part-manifest.json> --output <fresh-private-cad-directory> --tread-mm 1.5 --relieve-return-run
<mujoco-python> -B sim/pressure/gravity_release.py --project . --output <fresh-private-release-directory> --tread-mm 1.5 --resolution 256 --dt .0001 --duration .8
<mujoco-python> -B sim/pressure/gravity_release.py --project . --output <fresh-private-freefall-control> --tread-mm 1.5 --no-contact --dt .001 --duration .2
```

To inspect native geometry, open the private `track-offset-tread-relieved-candidate.f3d` in Fusion as a separate document. To run the existing colored driver, use its preserved private launcher; WASD drives, C coasts, B brakes, Q quits. That launcher uses the original archive, not this experimental candidate. Native route reproduction requires the explicit archive hash, independently reopened inventory, owned audit-document identity and a fresh output path described in [the native route audit](NATIVE_ROUTE_CHECK.md). Explicit contact roots are checked to be actual belt/tread bodies before classification.
