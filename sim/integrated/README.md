# Integrated current-geometry robot plant

<!-- cad-still -->
[![Preliminary robot with the scoring cassette raised above the blue bumper](../../cad/preliminary-design/media/002-preliminary-robot.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Guided cassette at the start of a cycle, with a floor NOTE ahead. Preliminary design release 004, effective.** The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. This still uses the same robot geometry as the interactive preliminary review. [Still and geometry record](../../cad/preliminary-design/media/002-preliminary-robot.json)
<!-- /cad-still -->

This directory is the integration layer under construction, not a completed robot simulator. The saved browser rehearsal is not the requested modeled robot.

## One electrical bus

`shared_supply.solve_supply` receives named pure subsystem electrical demand functions. A demand function evaluates actual terminal power at a candidate bus voltage without advancing any rotor, chassis, piece or controller state. The coordinator solves one voltage and then advances all subsystems together. Separate stiff batteries must not be hidden inside individual mechanisms.

The present supply is a quasistatic open-circuit-voltage plus resistance hypothesis. It selects one positive-slope equilibrium of `V + R P(V)/V = Voc`; a second stable equilibrium or missing equilibrium is an explicit domain stop. The finite bracket search is not a proof for arbitrary pathological port functions. Expected motor/current-limit ports are continuous piecewise smooth. It does not model battery chemistry, wiring inductance, controller switching, thermal state or hardware protection.

Signed power allows a braking motor to supply another subsystem on the common ideal bus. Net battery regeneration is rejected by default. Explicitly enabling ideal absorption is a test hypothesis, not approval of real battery/controller regenerative behavior. Source terminal work, internal resistance heat and load power are separately reported and balance checked.

## State continuity requirements

- Chassis pose, linear/angular velocity and rotor states evolve from tire forces and reciprocal motor dynamics, not commanded chassis velocity.
- Acquisition receives actual robot transform/velocity and returns equal/opposite reaction force/torque to the chassis, replacing prescribed-base work.
- One NOTE world pose, velocity and spin persists between acquisition, pitch/lift and launcher. No recentering, ideal attachment, aligned-pose reset or invented feed speed.
- Controller observations and commands are separate from plant truth; disable/coast preserves momentum. Brownout and recovery require explicit stateful behavior.
- Component/contact parameters, incomplete mass/inertia, source-native bearing seating and unresolved hard-contact domains remain visible gates.

Current tests cover parallel loads, shared voltage sag, constant-power collapse, explicit regeneration, internal power exchange and invalid input. The drivetrain plant is being authored separately from the preserved mechanism experiments. These tests do not establish an integrated full cycle.
## First coupled drive-to-loose-NOTE milestone

Run `python sim/integrated/run_coupled_acquisition.py --output artifacts/fresh-coupled.json`. Optional `--step .0005` gives the half-step control; `--scenario command-off` opens the drivetrain and pinch commands at 0.6 s. A fresh output is required.

The current six-motor experiment combines four CIM drivetrain motors and two pinch motors. Chassis and loose NOTE begin at rest. One shared bus is solved against their simultaneous midpoint electrical demand. Finite contact forces act equally and oppositely on the NOTE and chassis; chassis translation/yaw is integrated from tire and mechanism forces, not prescribed speed. End-state pose and midpoint force records are distinguished by the numerical method. Mechanical reactions are internal transfers, not invented external-base work.

The open-loop nominal case first contacts at about 0.562 s. It continues through the modeled floor corridor and stops at its rear boundary around 0.752 s, with robot speed about 0.859 m/s. This is an unresolved-domain stop, NOT capture: the robot has not yet braked or transitioned into a verified retained state. Peak mechanism reaction is about 1.535 N under the stated compliance hypotheses. Halving timestep reduces maximum global energy residual from 1.066 mJ to 0.256 mJ; finite boundary-crossing times differ by 0.5 ms. Command-off preserves wheel/NOTE momentum and also reaches the domain boundary.

The three bound traces preserve the source geometry, parameters and failed continuation. Five kernel tests cover reaction force/world torque balance, chassis response, atomic domain stops, common bus loading and coast; three evidence tests cover lineage, energy/refinement and off-command momentum. No actual acquisition controller, lift/launcher phase, brownout supervisor, field contact, calibrated mass/traction or full-cycle acceptance is implied. Rotor inertias are reduced generalized drive inertias; 3D gyroscopic coupling and load transfer are not modeled. The conservative bearing/obstacle representation is a gate, not proof of collision.

Next: closed-loop approach/braking using explicit observations, then continuous retained-piece and lift dynamics without resetting this state. This baseline remains available as the open-loop overrun control.
## Threshold braking is a retained negative control

`run_observed_approach.py` adds exact virtual pose observations at 100 Hz and a latched stale-observation fault. Both a 250 mm and an earlier 340 mm braking threshold still leave the bounded rear corridor. The later case reaches the boundary with chassis speed about 0.545 m/s; the earlier case still has about 0.170 m/s. A stale-observation control also brakes without inventing a stopped state and overruns. These are unresolved continued motion, not a safety verdict or automatic requirement for a retaining stop.

The three source-bound traces in `evidence/approach-*.json` preserve those results. Pinch command-off is open-circuit coast; wheel/NOTE inertia remains. No numerical dwell or capture is declared. Next controller work must regulate approach speed early and coordinate feed, rather than assuming a position threshold alone stops the robot. The existing `relativeVx` diagnostic omits rotating-frame derivative terms; this straight symmetric experiment has negligible yaw and the threshold controller does not use that diagnostic for control.

## Early speed regulation, not a stopped-piece latch

`python sim/integrated/run_speed_approach.py --output artifacts/fresh-speed.json` uses a separate straight-only virtual controller. The observation derivative is `qdot = R(-heading)*(vNOTE-vrobot) - yawRate*J*q`; common rigid co-rotation and finite-difference tests expose the transport term that the older unused diagnostic omitted. Unexpected lateral/yaw observations are rejected rather than pretending this is a steering controller.

The nominal 1 ms experiment tapers a provisional 0.25 m/s approach toward a 215 mm relative goal, coordinates low pinch demand, and commands dissipative drive braking rather than assigning a stopped velocity. At 3.009 s it reaches a 100 ms numerical ground-dwell window: relative NOTE X is 228.676 mm, chassis speed is still 17.430 mm/s, NOTE world speed is -0.092 mm/s, and pinch rotors are still 1.446 rad/s. The run ends at that diagnostic, not at a physical capture latch. Continued motion after this endpoint remains to be integrated. Maximum global energy residual is 0.935 mJ; this alone does not establish trajectory convergence or calibration.

This is an improvement over the preserved threshold-overrun controls, not a demonstrated pickup-to-shot cycle. Exact virtual observations, gains, traction, compliance, mass and controller modes remain hypotheses. No ideal holding or reset is introduced. A later pitch/lift model must consume this actual world pose, velocity, spin and rotor state, extract contact at its actual offset, and preserve chassis motion; the older 9.888 mm-offset retention example cannot simply be attached. Half-step, stale and zero-pinch controls are separate required checks before broader use.

### Actual entry preload is not the older lift assumption

`retention_entry_bound.py` recomputes contact forces at the saved END state (the trace contact rows themselves are midpoint values). At this endpoint the two side forces are each 1.755213 N under the present 2 kN/m stiffness and 0.877607 mm indentation hypotheses. Optimistically assigning all friction vertically gives 2.106256 N, below the 2.304563 N NOTE weight by 0.198307 N. Thus unchanged side-friction preload fails even a necessary static lift condition. This is not a measured hardware failure or a mandate to stop a moving piece.

The earlier 10 N-per-side finite-patch lift experiment is a separate hypothesis, not a continuation of this acquisition result. The bound preserves current world pose, velocity, yaw/spin and rotor state; it does not advance lift or attach the piece. Higher-stiffness scenarios show a possible force budget only, not sufficient torque allocation, maintained contact, actuator holding, tolerance or hardware acceptance. Measured preload/compliance or an explicit supporting mechanism is needed to justify that transition.

## Same-model continuation boundary

`continuation_state.checkpoint/restore` preserves the complete chassis, NOTE, rotor, clock and cumulative-energy state. Restore requires the same externally supplied source identity and physical configurations, and rejects missing coordinates, changed parameters, nonfinite state or inconsistent clocks. Three regression tests compare resumed versus uninterrupted braking steps exactly and retain nonzero motion. This is a same-kernel checkpoint, not a phase adapter: it does not authorize changing from floor-constrained XY/yaw to pitch/lift, invent contact forces, or discard a velocity component. That transition remains separate work.

## Refined controls and deeper indexing

The 0.5 ms approach rerun also reaches the diagnostic; its endpoint relative position differs by 8.59 micrometres and maximum energy residual falls from 0.935 to 0.221 mJ. That is refinement evidence, not a whole-trajectory convergence or hardware claim. With stale observations the fault latches at 0.550 s while the chassis is still moving 0.244 m/s, then braking continues another 52.24 mm before near rest. The pinch rotors retain momentum. The zero-pinch control also reaches the low-speed diagnostic because the chassis moves over the ground-supported piece and passive contact can spin the rollers. Therefore this diagnostic does not demonstrate powered capture or retention.

`python sim/integrated/run_regulated_acquisition.py --index --output artifacts/fresh-index.json` uses the reviewed explicit optimized electrical query, unchanged full supply-root scan and full final force/state update. Its default non-index run reproduces every original approach trace row exactly. The index option instead targets 185 mm and requires both speeds below 3 mm/s for 100 ms. It reaches qX 188.567440 mm at 4.437 s, with actual chassis speed 1.999 mm/s, NOTE world speed -0.570 mm/s and pinch rotor speeds 0.2314 rad/s. A complete same-model checkpoint preserves all motion and cumulative energy. It is not reset into the old retention example.

Neither the first forward dwell nor this deeper index is asserted lift-ready. Separate whole-robot geometry experiments show the forward 50.676 mm offset intersects bumper backing during early pitch; even a prescribed +5 mm offset produces represented support contact. These are contact witnesses, not a foam-safety verdict or proof compliant guiding cannot work. The next step is a bounded guide/contact or corridor revision at the actual indexed state, plus a defensible preload/support mechanism. The present side-friction force deficit remains. No force or successful compression is inferred from rigid overlap.

The deeper-index half-step control preserves the same diagnostic and reduces maximum energy residual from 0.945 to 0.223 mJ. Its exact endpoint and all input hashes are retained separately; neither timestep supplies a pitch-clearance or retention claim.

## Finite contact interfaces, not an attached payload

The indexed state now has 30 explicitly registered side-belt material-face witnesses. Their world positions and normals are derived from the unchanged actual chassis/NOTE transforms. All 30 classify inside the nominal native torus at this initial state. Common rigid motion preserves the geometric test, while relative slip or pitch can invalidate it. Runtime must reevaluate these witnesses; this fixed-footprint model stops at lost coverage rather than asserting attachment. It supplies neither pressure calibration nor missing elastic history.

The additive holding interface uses finite motor inertia and signed shared power. Its memoryless option can carry the current traction law without inventing shear-spring energy; the elastic option is separately initialized. The proposed guide remains unadopted and materially unqualified. `guide_contact.wrench` accepts only supported analytic finite-guide queries and explicit stiffness/damping/friction hypotheses, returns equal/opposite piece/chassis wrenches, and accounts for actual moving-guide work. Normal elastic energy, damping and friction power balance are tested. Exact archived Boolean-disagreement and other unknown geometry queries are rejected, not reinterpreted as clearance. These are force interfaces; no pitch/lift trajectory or successful guiding is yet claimed.
