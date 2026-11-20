# When should Blue 3 yield the midfield lane?

**Use the ground-supported concept for the next rehearsal.** Assign Blue 3 to retrieve when it has a separate lane. When opponents or partners occupy that lane, leave Blue 3 at the support point. The team can rehearse both roles before committing to a robot-weight mast. The current mass, load-path and service evidence leaves a suspended terminal role open.

## What could students infer at kickoff?

The January 6 release-day manual, sections 5–6 V0, gives one point for PARK, three for an unlit ONSTAGE robot and two for each additional qualifying robot on the same chain. Compare two decisions when parking is certain and missing a hang leaves the robot outside the zone. Hold the rest of the alliance score fixed. [V0 source fingerprint and comparison scope](../../kickoff/stage-rules.json) · [V0 manual, Table 6-2](https://bchrobotics.com/wp-content/uploads/2024/01/2024GameManual-unlocked.pdf)

| Partner already on the chain | Park | Successful hang | Miss | Hang probability needed to match parking |
| --- | ---: | ---: | ---: | ---: |
| None | 1 | 3 | 0 | 1/3 |
| One qualifying partner | 4 | 8 | 3 | 1/5 |

At a hypothetical 25% success rate, parking leads 1 to 0.75 points without a partner. With a partner already on the same chain, the hang attempt leads 4.25 to 4. That changes what is worth investigating at kickoff. The team has no measured 25% reliability. This calculation assumes the partner stays qualified and uses only the release-day scoring entries. It excludes ranking points, later team updates, congestion and fall consequences. [Reproducible kickoff projection](season-measurements.json)

## What happened in the later six-robot rehearsal?

The following comparison uses the **archived final 2024 manual through Team Update 21**. At each matchup the same five other robots, field setup, 15-second AUTO and 135-second TELEOP ran twice. Only Blue 3's TELEOP role changed. Both runs assign Blue 3 the same opening and no terminal action. The final six-robot scores are retrospective evidence. [Season profile](../../kickoff/crescendo-field.json) · [Role inputs](roles.json) · [Full matched runs](role-results.json)

| Context | Blue 3 role | Focal TELEOP traffic wait | Distinct center NOTE acquired | Final alliance margin |
| --- | --- | ---: | ---: | ---: |
| Separate partner lane | Retrieve | 0 s | 1 | −5 |
| Separate partner lane | Hold support lane | 0 s | 0 | −5 |
| Contested midfield | Retrieve | 133.05 s | 0 | +7 |
| Contested midfield | Hold support lane | 0 s | 0 | +7 |
| Same contested roster, only Blue 2 switched to AMP | Retrieve | 133.05 s | 0 | +8 |
| Same contested roster, only Blue 2 switched to AMP | Hold support lane | 0 s | 0 | +8 |

Switching only Blue 2 to the AMP-side assignment raised the modeled alliance margin by one, but Blue 3 remained blocked by Red 3. A partner lane change alone did not resolve the traffic. Alliance score alone leaves both focal roles tied. One field-level acceptance event requires an identified NOTE aboard Blue 3 at its prescribed home-lane endpoint. The clear case acquired the NOTE at 3.35 s and reached home at 5.60 s. The rate over the full 135 seconds is 0.00741 pieces/s. The single-piece mission offers no evidence for repeated cycles or a partner handoff. [Engine observation](transport.mjs) · [Event records](transport-runs.json)

| Disturbance over the same 135 s window | Accepted / demanded | Outcome |
| --- | ---: | --- |
| Separate lane, one center NOTE | 1 / 1 | Pass |
| Shared midfield lane | 0 / 1 | Blocked in traffic |
| Empty supply station | 0 / 1 | No compatible stock |
| Displaced mast guide | 0 / 1 | Team holds the robot in the pit for the full window |
| Disconnected pack fails receiving gauge | 0 / 1 | Same no-dispatch policy |

The latter two cases are service-availability scenarios. The R10 native 2 mm guide-offset control produced 17,485.77 mm³ interference. The R11 pack gauge, displaced 1 mm above its pocket limit, produced 5,920 mm³. Either measured geometric failure triggers the assumed pit hold. No time-to-repair data are available. The worst observed rate is zero. The five deliberately selected runs do not estimate an event-season failure frequency. [Throughput review](throughput-results.json) · [R10 measurements](../../cad/service-readiness/reopen.json) · [R11 measurements](../../cad/service-readiness/r11-reopen.json)

## Whole-robot architecture decision

The fixed-retrieval concept depends on a clear center approach, sliding mast, lock and frame connections. The ground-supported concept delegates midfield by partner role, omits the robot-weight mast and uses detached packing tooling. Both use the same established field footprint for comparison. The latter is a concept, with no new integrated ground-only CAD assembly. The trade compares the combined concepts. It does not attribute lane delay to the mast.

The study sets a planning target of at most 15 s of focal traffic wait, no service interference above 0.01 mm³ and a demonstrated pit turnaround within 180 s. The thresholds are project choices. In the adverse case the fixed concept misses the wait target by 118.05 s and fails the guide fit. The ground-supported concept clears both modeled checks. Both lack a timed physical service record. Select ground support provisionally and retain the hang as a separate engineering question. [Architecture inputs](architectures.json) · [Objective-by-objective results](architecture-results.json)

## Can the mast support the actual robot?

**The evidence does not establish a working load.** The STAGE mast is the sole mechanism intended to elevate the entire robot. The separate R11 service spacer assumes external support. The R9 installed mass subtotal is 43.712–47.449 kg with 136 unresolved articles. Neither endpoint bounds the complete robot. The older R8 60.735 kg case is a scenario. Its 596 N static weight and 1,191 N doubled demand are screening values. [Mass register](../../cad/assembly-readiness/mass/result.json) · [R8 load screen](../stage-climb/static-screen.json)

| Load-path portion | Evidence and remaining limit |
| --- | --- |
| Chain, bearing roof and spine | One saddle bears on one chain. The shared-chain case gives a 159.90 N difference between the chain-leg tensions at the seat. Longitudinal slip capacity and attachment strength remain unknown. |
| Slide and guides during transfer | Full travel clears nominally. The 2 mm displaced guide jams. The moving assembly has no verified working load at its limiting extension. The doubled demand represents an assumed transient. Actual acceleration, shock and buckling margins remain unknown. |
| Candidate nut, thrust bearing and housing | Thomson rates the BN3708S bronze nut at 300 lbf maximum dynamic load, about 143 N above the earlier doubled demand. EZO lists 2,160 N basic dynamic and 2,440 N basic static for one F6-14M. Opposed bearings act in opposite directions. Neither a contact rating nor a nut rating sets allowable load for the assembled drive or its housing. [Thomson](https://www.thomsonlinear.com/en/product/BN3708S) · [EZO](https://www.ezo-brg.co.jp/english/product/spec.php?eid=02419) |
| Lock and guide walls after power loss | The inherited ideal seated-pin case retains 152.8 mm floor clearance at +5 s. A missed pin returns the robot to the floor. At the doubled load, average bearing demand rises from 23.82 MPa with equal wall sharing to 47.65 MPa if one wall carries all of it. Local bending, alignment and an allowable capacity for that condition remain open. |
| Crossmember, fasteners, sleeves, rail attachments | The earlier ideal beam gives 33.89 MPa bending and the roof gives 29.85 MPa. Rail holes and seating are measured geometry. The records supply no verified joint allowables for welds, clamp slip, fastener tension or local wall tear-out. Actual structural and attachment margins remain unknown. |

The shared elevation study uses one free saddle. In the settled pose the vertical reaction balances gravity, but the lock and frame allowable loads remain open. Before rotation, the nominal fore-aft CG offset creates 5.23 N·m that the free saddle cannot resist in static equilibrium. This fails the fixed-pose assumption. At the limiting transfer pose, the upper R8 mass with a hypothetical upward acceleration of 1 g gives 1,191.21 N demand, with no second dynamic multiplier applied. The nut alone clears that screen by 143.26 N. The bearing basic dynamic rating leaves its working-capacity check open. Removing the only chain contact fails. The actual-mass case stays open. The independent disabled-hold and service receipts cover different configurations. [Elevation inventory](load-inventory.json) · [Common review](elevation-results.json) · [Season calculations](season-measurements.json) · [Disabled trajectory](../stage-climb/solo-trajectory.json)

## Reproduce

From the season repository, run `python experiments/alliance-service/replay.py PATH_TO_AGENT_FRC_SCRIPTS` with [Agent FRC 0.38.0](https://github.com/ehartye/agent-frc/tree/344fac71aa4aecbec66cf4015c7646b1149a542b), which includes `strategy-lab.mjs`, `design-review.mjs` and `hardening.mjs`. The replay regenerates the field events, recomputes the season measurements, binds the generated receipts by SHA-256 and runs all four reports.
