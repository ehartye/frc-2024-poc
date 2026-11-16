# A2 original cycle architecture

A2 is a provisional engineering experiment, not an owner-selected build. A Speaker cycle specialist remains the shortest complete-cycle investigation; Amp and feeder alternatives are retained if acquisition/release evidence defeats it. The shared artifact seed remains the editable narrative/blockout. `cad/cycle-contract-A2.json` is a separate, explicit engineering interface proposal, not an undocumented extension to its strict v1 schema.

## Decision

Compare a multifunction reversible intake/shooter against a separate reversible acquisition/indexing cassette and adjustable shooter. Explore the latter first because capture recovery and release-state characterization can be tested independently, with visible sensing between them. This costs mass and parts and is not proof it will outperform the shared mechanism. Intentional dual-acquisition redundancy is deferred until failure evidence warrants its packaging cost. Sensor cross-checks and bounded reverse recovery are included without pretending duplicated sensors are independent physical redundancy.

Use continuous legal-height bumper envelopes and an original pivoting over-bumper pickup for this candidate. A frame bay is not a NOTE-sized bumper opening. A2 does not erase the opening alternative, but the existing plugin rule review prevents treating a simple punched bumper as compliant. No team assembly geometry has been imported or replicated.

## Release correction, not a hidden success

One guide angle now determines both initial velocity elevation and NOTE body pitch. Spin remains an output needed from the contact plant or measurement, not a stabilization assumption. This removes the earlier inconsistent 50-degree velocity / 14.756-degree body combination. It does NOT solve the scoring problem: a coarse matching-angle screen found no positive conservative whole-torus bound. A failed conservative bound is not necessarily an actual collision. The next gate is a time-resolved whole-body surface sweep with physically derived release/spin, followed by complete target roof/wall registration. Until then the controller's release envelope is unverified and release remains inhibited in an executable mission, not silently bypassed for a complete-looking demo.

## CAD/code handoff

Use the numeric geometry contract to generate original endpoint/reserved volumes, then inspect the full intake sweep and piece transfer. Its proposed adjustable gap travel is not a measured tolerance. CAD must resolve pivot brackets, bearings, shaft support, locking/repeatable adjustments, wiring, retention and service access. Do not fabricate from these reserved volumes.

Code owns independent acquisition, drive and shooter activities with explicit resource guards. Initially permit drive plus spinup only as an unverified concurrency candidate with power limits; keep release stopped until a validated pose/release envelope exists. Moving shots are an experiment requiring motion compensation and sensor latency evidence, not a timing discount applied to the serial rehearsal.

Maintain a condition-based pit record linked to component IDs. Compare commanded and measured behavior with freshness/implausibility checks. A vibration change prompts inspection, not automatic diagnosis. Product/joint specifications govern locking compounds and repair; do not disturb intact joints every match. After repair, log configuration and retest the affected behavior.

## Positive sampled geometric candidate

A subsequent pointwise torus-plane crossing search found a useful original route: exit height 0.700 m, guide/body/velocity elevation 60 degrees, required speed 7.5 m/s and target-local exit X=-0.7334481 m. Across 64 x 16 torus surface samples, all rising crossing points lie inside the nominal aperture with a minimum 108.313 mm margin. This does not contradict the earlier failed conservative bound, which over-approximates the full body throughout the transit. It is a sampled geometric hypothesis, not a continuous proof or plant-generated exit. `scripts/a2-release-search.py` reproduces the search. The selected geometry uses a 0.240 m guide from [0.130,0,0.492154] to [0.250,0,0.700] in robot coordinates. Spin, release stability, roof/walls, field registration and motor achievement remain required gates.

## Intake revision from actual spatial screening

The initial 0.340 m arm endpoint intersects the front bumper. Its longer fixed 0.680 m replacement clears endpoint sections but exceeds the extension envelope during the intermediate zero-degree pose. Both are rejected. The current original candidate coordinates a telescoping arm with rotation: L=min(0.680,0.520/cos(theta)) when cos(theta)>0, otherwise0.680, from -40 to110 degrees about [0.100,0,0.480] m. A1501-pose 2D screen of the20mm arm section finds no front-bumper section intersection and4.8mm nominal roller extension margin. This is narrow, excludes tolerances/flex and does not qualify the retained NOTE, cradle, four-bar or complete assembly sweep. A second actuator/coordination and moving inertia are now required; the fixed-arm plant must not be presented as this mechanism. The lift arc is followed by a separate powered descending transfer, not a fictitious continuous conveyor.

Original stowed/deployed STEP and actual tessellated renders are produced by `python cad/build_a2_layout.py`. Green parts reserve service, linkage and transfer space, not engineered hardware. Editable source is the generator plus cycle JSON. Every STEP is reopened and its solid count checked.

## Review correction: intermediate height and parameter authority

Independent review found that the retention roller reaches1.230m at90degrees under the extension-only law, exceeding the1.2192m screen. That law is superseded by `cad/a2_kinematics.py`: it limits telescope length by nominal travel, configured forward-center cap and a height cap covering every moving arm/sleeve/link/roller/shaft. At90degrees length is0.6642m and retention-roller top is1.2142m, leaving5mm nominal allowance. All inputs come from the contract, and changes invalidate its SHA-bound screen. Tests include pivot/radius/lateral mutations, malformed values and native CAD bounds at-40,0,90,110degrees. The1501-pose screen is not a continuous collision certificate; narrow4.8mm forward clearance and assumed retained-piece motion remain design risks. The initial hardcoded screen is superseded and must not be used as acceptance evidence.
