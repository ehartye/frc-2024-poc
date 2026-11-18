# Isolated moving cassette and cam dynamics

**Experimental, unadopted, fixed-cradle mechanical feasibility prototype.** This is not a robot trajectory, servo calibration, completed locking mechanism, or electrical model. Canonical geometry is unchanged. Both coordinates evolve independently: cassette inward translation `u` and cam angle `theta`. No commanded position is assigned to either coordinate.

## Declared hypotheses and source facts

The fixture uses **1.5 kg moving cassette mass**, **0.0001 kg mÂ² output-referred inertia**, **1,000,000 N/m normal stiffness**, **10 N s/m compression-only damping**, and **Â±0.03 N m externally requested mechanical torque**. These are transparent example hypotheses, not weighed parts, measured stiffness, manufacturer controller limits, or a calibrated servo model. Tests vary mass 0.75/1.5/3 kg and inertia 0.00005/0.0001/0.0002 kg mÂ². They verify mathematical parameter response, not physical validation. All parameters must be explicitly supplied.

Geometry comes from the reviewed independent cam port. The reviewed spring law supplies nominal pair force and potential; it does not prescribe NOTE preload. The source-pinned `powered-cam-interface.json` identifies SRS V1 REV-41-1097: nominal 6 V, 13.5 kg-cm stall torque, 2 A maximum stall current at 6 V, and 0.13 s/60Â° no-load speed. Those endpoints do **not** identify output inertia, torque/current/speed law, continuous holding capability, internal control dynamics, gear friction or power-loss response. The prototype's low torque cap is not a manufacturer continuous rating. Historical REV-11-1144 provides a 6 V supply path; no 12 V servo supply or invented active per-port current limit is introduced. Electrical current remains explicitly `None`; mechanical work cannot be relabeled battery energy.

## Equations, reactions and integration

`m*u_ddot = spring_force + cam_Q_u + optional_NOTE_Q_u + static_stop_reaction`.

`J*theta_ddot = bounded_external_torque + cam_Q_theta`.

Cam generalized forces are reciprocal derivatives of the same gap and potential. Spring potential, cam potential and both kinetic energies form total mechanical energy. Separate ledgers integrate signed external actuator work, signed external NOTE-load work and nonnegative contact heat. The instantaneous identity is `dE/dt + heat_rate = actuator_power + NOTE_load_power`. The external NOTE port is force in positive u with work `Q_u*udot`; an eventual coupled caller must apply the opposite spatial reaction to its NOTE/body exactly once. These tests use a supplied load only, not an integrated NOTE.

RK4 uses constant external torque and NOTE load over each step and validates every stage and endpoint. It is **not** energy-exact or an impact solver. Receipts preserve numerical energy residuals and half-step comparisons; no ledger correction hides integration error. An immutable input state survives every rejected step. Unrepresentable clock advances are rejected.

Cassette end-stop reaction is implemented only for a resting coordinate exactly at u=0 or 1.5 mm with outward net force. Its power is zero. A moving impact or a stage leaving the studied domain raises `DomainStop`; momentum is not clipped, reflected, reset, or called stopped. A future impact law needs its own restitution/compliance and energy accounting. Theta [0,pi] is the CAD study domain, **not** a fabricated angular hard stop.

## Fresh controls and interpretation

`check_cassette_cam_dynamics.py` records seven 10-ms fixtures: held-open equilibrium, moving powered state and half step, removed-torque hypothesis, exact dead center, perturbed dead center, and an external NOTE-load port. Two additional 250-ms zero-torque cases start at the resting held-open state and compare full/half steps. Static-stop and rejected-impact controls are separate.

At theta=0.2 rad, nominal held-open u is 0.0294035 mm. Feedforward holding torque balances the cam reaction; this is an external mechanical fixture, not proof an SRS controller can hold indefinitely. With motion initially present, removing applied torque retains state and accumulated work and permits spring/contact-driven backdrive. Exact theta=0 symmetry is stationary in this ideal model, but a +0.001-rad perturbation increases; zero cam torque at dead center is not a positive latch. In the 250-ms zero-torque example, theta moves from 0.2 to about 1.266 rad and u from 0.0294 to 0.5362 mm: the frictionless idealization clearly does not retain the original open position. This is not a predicted real-servo speed or loss response. Power removal is only one **zero-mechanical-torque hypothesis**, not asserted real lost-PWM or unpowered-servo behavior.

## Actual mass partition and minimum characterization

- The translating assembly is the positive-side pinch belt, track/transfer pulleys, stub/remote shafts, bearing representations, pinch NEO body and plate, transfer belt, modified side weldment, sliding guide blocks and follower. Exact inclusion follows the cassette geometry selector and follower attachment. Its BOM mass, moving fasteners, material density, guide friction and cable forces remain unknown. Do not treat a nominal solid envelope as actual material volume.
- The fixed-cradle side contains guide rods/stops/backbone, servo body and wide support. The published complete-servo weight is **not** an output rotor inertia or cassette mass. Its fixed-base mass still belongs in any eventual whole-robot/cradle mass budget.
- Output inertia must include the real horn, center/attachment hardware, original flange/neck/cam and output-referred gear/rotor inertia. CAD can estimate authored-part inertia only after material selection. Reflected internal inertia/backlash/friction require characterization or explicitly bounded hypotheses.
- Spring mass and any distributed deformation are omitted here. A moving-base model must restore gravity, full spatial inertia and base/cassette/servo reciprocal forces and moments without double counting the previously declared mechanism mass.
- Minimum physical inputs: weigh the assembled moving cassette; identify materials/fasteners and their centers/inertias; measure force versus u and friction/hysteresis; characterize cam compliance and loss; obtain output inertia/backdrive bounds; measure commanded torque/position response, current and temperature under the proposed 6 V supply and fault modes. Verify mode/clocking and travel stops.
- Retention hardware is a source-supported nominal pairing, not verified installation torque, locking compound choice, tooth/thread strength, fatigue, impact strength or thermal holding. Frame anchorage/welds and shaft clamp remain unqualified. Neither the cam nor the powered hold adds a positive mechanical latch.

The feasible bounded model is therefore an isolated fixed-frame component with explicit positive inertia, passive spring/contact, bounded external mechanical work and atomic unresolved-event rejection. Adoption requires independent review and the missing coupling/characterization gates, not a successful small fixture alone.

## Reproduction

`python -B -m unittest discover -s tests -p test_cassette_cam_dynamics.py`

`python -B cad/assembly/check_cassette_cam_dynamics.py`

Evidence: `evidence/cassette-cam-dynamics.json`. No native CAD process is needed. Current receipts bind producer/model, spring and geometry/source contracts. Supplier CAD is neither copied nor exported by this study.
