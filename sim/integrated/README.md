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
