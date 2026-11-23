# Guided cassette POC enters Detailed Design

**Decision, 22 November 2026.** Proceed with the greenfield guided cassette from preliminary review 002. Use Profile A, the powered belts, the existing pivot and slide datums, four-module drive, and the two-line guided climb. Competition repeatability remains a validation target during detailing. The historical side-toe assembly is a different architecture.

## What the CAD team can use

The first detailed article is the [editable Profile A blade](profile-a/profile-a.f3d). Its contour sketch, symmetric extrusion, tool-start plane and four countersinks survived a fresh Fusion reopen. The width is 434 mm. The four M3 clearance bores lie at Y = −165, −55, +55 and +165 mm, with recessed heads on a common inclined seat. The purchased stock specification for this study is UHMW polyethylene. The archive is a component in deployed pickup coordinates. Use the recorded transform to place it in the parked assembly. [Reopened native features](profile-a/readback.json) · [Verified placement and STEP comparison](profile-a/geometry-check.json)

The section matches the selected blade in the preliminary assembly in both directions. The blade seats on the pickup crossbar with zero material overlap and leaves 2.335 mm to the nose roller. Moving the blade 2 mm along its width produced 219.20 mm³ of difference in each direction. The first native cut left small crescents at four facet transitions. Starting the countersink cutter 0.5 mm above the face removed them. [First-cut comparison](profile-a/first-cut-geometry.json)

Next detail the crossbar and roller bearing supports around this common seat. Keep the blade removable and the runner height adjustable. Check the roller axis, installed screw envelopes, and supported blade withdrawal in that assembly before fixing the rest of the cassette around it.

## Operating assumptions at handoff

| Item | Basis for starting CAD | Remains open |
| --- | --- | --- |
| Pickup and scoring | With Profile A and the 6.88 m/s command, six distinct NOTES counted by 116.50 and 115.08 s in two nominal numerical resolutions. | Adverse R06 counted four. The 19-of-20 requirement stays on the validation plan. An R02 return reached the control-board envelope. Keep a guard or electronics relocation available if this mode persists. |
| Runner and intake | The nominal transition and pickup are represented. The replaceable nose permits cheap profile changes. | The unloaded NOTE overlaps the modeled runner by 1.36 mm and the central belt by up to 7.67 mm. Resolve hard clearance separately from elastic compression. Verify loaded pitch and carpet clearance before machining permanent mounts. |
| Weight and power | Upper inspection estimate 56.270 kg against 56.699 kg, including the existing reserve. At the modeled 25 A concurrent auxiliary allowance, the governing case has 8.705 V bus and +0.448 V motor reserve. | The 39 A case misses the motor requirement by 0.089 V. Reopen the load schedule if selected actuators exceed the allowance. |
| Climb and service | Use the modeled guided catch and supported, power-isolated service pose. The separate 24 s endgame case has about 1.99 s remaining at power cut. | Six scores plus climb on one 135 s clock leaves about 0.18 s. Carry this as a stretch case. Load holding, disabled retention and proof testing stay on the hardware plan. |

The POC handoff starts component and assembly CAD. Fabrication, robot operation and a repeatability claim need their own evidence. [Review inputs and file identities](handoff-evidence.json)
