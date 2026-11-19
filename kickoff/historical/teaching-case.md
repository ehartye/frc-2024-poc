# 2024 CRESCENDO: Fixed-location SPEAKER-cycle teaching mechanism

[Official manual](https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf) — 153-page retrieved PDF; section 6 V10, section 7 V13. Reviewed 2026-11-15. The [case manifest](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/case-studies/2024/case.json) holds selected rule facts and trace IDs. This is an original educational design case, not a reproduced team robot or complete competition design.

## Strategy alternatives

| Candidate | Tradeoff |
|---|---|
| AMP-oriented scorer | Lower-speed handling and simpler aiming; partner dependence and limited task range. |
| Fixed-location SPEAKER cycle | Repeatable alignment and one-piece indexing; requires launch characterization. |
| Variable-range shooter plus climb | Greater task reach; added calibration, packaging and development burden. |

Selected: **Fixed-location SPEAKER-cycle teaching mechanism**. Expose acquisition, inventory, flywheel readiness and release interlocks in a bounded subsystem. This is a teaching selection, not a claim of optimal match strategy.

Before choosing for a real team, estimate task success probability, cycle time, downtime, mass, available fabrication time and alliance contribution. Use measured evidence rather than invented match scores. Do not optimize one point value while ignoring reliability or penalties.

## Subsystems and interfaces

| Subsystem | Contract and remaining work |
|---|---|
| Drive/pose | Reach a reviewed stationary shooting pose; drive and collision integration remain future work. |
| Intake/indexer | One-pocket numeric inventory with acquire/release sensor contracts. |
| Flywheel/aim | Closed-loop synthetic speed target; external target-valid gate. |
| Safety/mode | Stop requests, jam latch and bounded release; no climb included. |

The implemented slice accepts enabled/score-request/availability/alignment inputs, estimates actuator state in a desktop plant and gates release. Inventory and jam behavior are synthetic. Drive, complete autonomous routes, real sensors and actuator IO remain integration work.

## Code, CAD and simulation

Run the shared [Java case laboratory](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/examples/java-2026-sim/CASES.md). All cases use the explicitly pinned 2026 software baseline; that does not verify historical deployment compatibility. Synthetic setpoints and CAD dimensions are design assumptions, never official dimensions.

[CAD package](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/examples/fusion-case-concepts/README.md) contains the shared geometry generator, sideplate DXF/SVG outputs and native Fusion script. Fusion access is restored, but this season's native case remains unexecuted. The separate 2026 sideplate experiment does not validate this case.

CAD inputs feed the simplified plant through generated properties: flywheel inertia uses an assumed solid cylinder; the lift uses assumed mass, gearing, drum radius and travel. This coupling validates parameter consistency, not CAD-derived mass properties or contact behavior.

## Acceptance and remaining gates

- Require reviewed trace IDs before implementing a strategy change. Read complete rules and updates before actual design decisions.
- Test normal operation, unknown/invalid gates, disabled requests, jam timeout and voltage reduction. Report numeric simulator results separately from proposed physical checks.
- Confirm geometry, material, bearings, fasteners, guarding, frame envelope and service access in Fusion before treating the concept as manufacturable.
- No ballistics, note deformation or launch accuracy model.
- Intake compression, guarding, bearing fit and chassis integration unverified.

[End-to-end workflow](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/case-studies/README.md).

## Hardware reading map

Use the [selected rule locators and evidence tasks](rule-reading-map.json) with the [cross-season review workflow](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/docs/rulebook-hardware-review.md). Team Update 21 changed weight from 125 to 125.5 lb and OpenMesh to VH-109. Resolve the event and update date before selecting hardware.

[Sensor/model boundary](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/examples/java-2026-sim/SIMULATION-BOUNDARIES.md): controller feedback comes directly from the mathematical plant. CAN encoder integration and physical parameter identification are unverified.

Review the [Game-to-Robot Strategy Brief](strategy-brief.md) before freezing subsystem choices. Its recommendation is a proposal; no user selection has been recorded.
