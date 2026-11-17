# First actual-acquisition bridge: constrained limit

`FixedCassettes` wraps the actual pressure acquisition plant. Three fresh 1 ms steps must be byte-equivalent to the original outputs, with the same pressure, friction, rotors, bus solver, state and cumulative ledger. A separate query evaluates the recorded 0.58 s contact state without importing or advancing it. This is a small deterministic parity gate, not another full approach replay.

Both u values are fixed at zero, and both cam angles fixed at declared values. Spring/cam potentials are declared at fresh initialization and remain constant. The wrapper reports their energy separately; it does not alter the underlying zero reference or double-count the NOTE. Constraint forces balance each side's pressure/friction reaction plus spring and cam forces. Relative constraint work is zero; the inherited moving-chassis reaction and work are not erased. Caller input is copied, and all candidate step/diagnostic evaluations complete before committing wrapper state.

The wrapper rejects a moving/energized acquisition checkpoint as initialization. It also rejects dynamic release: adding masses and resetting coordinates would not be a valid extension. Unknown negative-side mounting, servo electrical behavior, loads and calibration are unchanged.

## Next coupled coordinates and mass hypothesis

A concrete hypothesis for the next extension is two 1.5 kg translating cassette groups already included in the existing 45 kg chassis, leaving 42 kg fixed chassis, plus two explicitly hypothetical output-referred cam inertias of 1e-4 kg m². These are NOT measured component masses. Fixed-u must recover the existing aggregate translation mass exactly; yaw inertia cannot be preserved by a scalar subtraction without fixed-frame COMs and inertia tensors. The two moving cassette COMs must be specified relative to the chassis, and velocities must include chassis translation, yaw cross-arm transport and inward u velocity. Their kinetic energy then produces the reciprocal coupled mass matrix, rather than treating the chassis as fixed.

First target after that derivation: the same fresh-rest experiment with stored spring/cam energy included from time zero, then 2 ms of dynamic-u behavior at the first supported contact state while preserving state/ledger and recording actual constraint-release work (zero only if position/velocity and potential remain unchanged). Any deliberate restraint release must be explicit, not an arbitrary old-checkpoint switch. Existing longitudinal belt forces, rotor reactions and shared supply remain active. This target is NOT implemented by the current wrapper.

Run `python -m unittest discover -s tests -p test_constrained_cassettes.py -v` and `python sim/pressure/run_constrained_cassettes.py <fresh.json>`. No full acquisition/lift/retention acceptance follows.
