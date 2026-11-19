# 2024 CRESCENDO: Game-to-Robot Strategy Brief

Review state: **proposal-for-review**. User-selected strategy: **none**. Reviewed 2026-11-16.

Provisional recommendation: **Fixed-location SPEAKER cycle**. Transparent teaching-scope heuristic: bounded scope favors the narrower task; expanded scope evaluates broader capability. This is not measured optimization.

## Source and event boundary

[Official manual](https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf); 153-page retrieved PDF; section 6 V10, section 7 V13. [HTML sections rechecked](https://firstfrc.blob.core.windows.net/frc2024/Manual/HTML/2024GameManual.htm) on 2026-11-16. HTML selected sections rechecked; PDF fingerprint belongs to earlier retrieval. Source bodies are not redistributed.
PDF SHA-256: 1b7e1dc3fe1b1bd64d97a482d7e78eb419ec77d611a15226a16b0c7011069b94. Event snapshot: Archived final/Championship update snapshot, not the original regional-season configuration.
Applicable event: UNRESOLVED: school, event name/date/stage and applicable update set. Do not transfer archive rules to an earlier event without checking updates.

## Rules and opportunities

AMP, SPEAKER and STAGE offer different task families; amplification changes the value of SPEAKER scoring. Reference: 6.5.1-6.5.6.
- **24-TIME — rule synthesis:** AUTO lasts 15 seconds. (6.4, p45).
- **24-COUNT — rule synthesis:** TELEOP NOTE control has one-piece limits with SOURCE ZONE and momentary qualifications. (G409, p65).

## Distinct strategy alternatives

| Alternative | Tradeoff | Required capabilities |
| --- | --- | --- |
| AMP-oriented scorer | Lower-speed handling and simpler aiming; partner dependence and limited task range. | AMP alignment, gentle transfer and alliance amplification coordination |
| Fixed-location SPEAKER cycle | Repeatable alignment and one-piece indexing; requires launch characterization. | Repeatable stationary pose, one-NOTE indexing and characterized launch |
| Variable-range shooter plus climb | Greater task reach; added calibration, packaging and development burden. | Range/pose uncertainty, variable launch settings and separately engineered climb |

All alternatives remain proposals. The school has not selected one.

## Decision criteria

- Hard constraints and safety first: fail any applicable rule or unresolved hazardous load path and do not advance hardware.
- Compare expected successful contribution, cycle time, consistency, alliance fit and development effort using measurements; weights await school priorities.
- Prefer bounded scope only provisionally when team resources, alliance role and prototype evidence are unknown. No fabricated weighted score.

## Explicit assumptions

- **A-GATE:** Target pose valid. proposed conservative release policy; not an official pose/alignment rule or inventory rule.
- **A-SCOPE:** bounded. unverified teaching assumption.
- **A-WINDOW:** 60 s. unverified usable service interval; not the full official phase duration.
- **A-PIECES:** 5 pieces. unverified task target; not a possession-rule limit.
- **A-ROLE:** A repeatable stationary shooting location will remain available.. unverified strategic assumption.

## Capability map and verification

### 2024-CAP-TIME: Mode coordinator

**rule-derived policy; basis 24-TIME.** Cancel autonomous demand at 15 s; physical stopping distance remains unmeasured.
Acceptance: Desktop test crosses deadline; all modeled voltage/release requests cancel.
Evidence: SeasonCaseTest.autoDeadlineRespectsSeason; FMS/robot adapter absent

### 2024-CAP-GATE: Release coordinator

**strategic inference; basis A-GATE.** Target pose valid; loss of validity prevents release.
Acceptance: Inject invalid/unknown state while ready and assert no feed.
Evidence: SeasonCaseTest.misalignmentBlocksRelease; gate is injected, not a validated field sensor

### 2024-CAP-ROLE: Scoring/targeting

**assumption-derived requirement; basis A-SCOPE, A-ROLE.** Reach one characterized stationary launch pose; hold release when pose validity is unknown.
Acceptance: Measure task success over repeated representative trials and confirm field geometry, loads and guard conditions.
Evidence: OPEN: teaching plant does not validate this capability

### 2024-CAP-RATE: Acquisition/drive/scoring cycle

**assumption-derived requirement; basis A-WINDOW, A-PIECES.** Average service rate at least 0.083 pieces/s over the assumed usable interval; at most 12.000 s per piece on average.
Acceptance: Demonstrate 5 successful deliveries in 60 s with acquisition, transit, alignment and recovery included where applicable.
Evidence: OPEN: arithmetic requirement, no measured throughput or match-score forecast

### 2024-CAP-SAFE: Power/mechanical safety

**engineering requirement; basis A-SCOPE.** Resolve electrical protection, physical retention, guarding, load paths and inspection requirements before hardware.
Acceptance: Document component-specific ratings, rule review, load/energy analysis and supervised physical checks.
Evidence: OPEN: no hardware or manufacturing approval

## What changes when an assumption changes

If that location is regularly denied, add tested alternate poses, range-dependent launch characterization and uncertainty gating; reconsider the AMP role before adding complexity.
For the entered interval, reducing usable time to 30 s requires 0.167 pieces/s. This changes feed/recovery or whole-cycle requirements; it does not demonstrate feasibility.

## Next artifact and review gate

**Capability and Verification Contract v1**: capability ID and source/assumption IDs; owner and interfaces with units; acceptance criterion and test method; design envelope and power/mass allocations; failure response and safety review; evidence status and open decision.
Bounded feed/recovery fixture with measured cycle timing; do not infer launch success from flywheel speed.
Reviewer records a strategy decision and resolves event/resource assumptions before subsystem requirements are frozen. Draft code/CAD may explore alternatives without implying approval.

## Existing evidence and open gates

[Case manifest](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/case-studies/2024/case.json); [Java laboratory](https://github.com/ehartye/agent-frc/blob/f6ab64ace84f8aca7168375364a9fd36b4eed224/examples/java-2026-sim/CASES.md). CAD state: Script and DXF only for this case; native case execution unperformed. Fusion access restored separately..
- No ballistics, note deformation or launch accuracy model
- Intake compression, guarding, bearing fit and chassis integration unverified
- No complete legality review or physical system verification.
- Java 17/WPILib 2026.2.1 is the teaching baseline for every season, not historical deployment.
- Direct plant feedback; no vendor sensor adapter or calibrated contact/flight physics.

Generated from original authored profiles and case metadata. No copied manuals, field assets or private vault content.
