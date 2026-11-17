# Integrated current-geometry robot plant

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
