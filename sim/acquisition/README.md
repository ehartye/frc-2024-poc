# Centered floor-entry experiment

<!-- cad-still -->
[![Preliminary design of the robot](../../cad/preliminary-design/media/guided-cassette-preliminary.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Project CAD · Preliminary design of the robot · Guided cassette 1.0 · effective.** The robot is at the start of a cycle, with a floor NOTE ahead. The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. The still shows the layout solids of the preliminary design review in its opening pose. [Open the design review](https://ehartye.github.io/frc-2024-poc/preliminary-design/)
<!-- /cad-still -->

Run `python sim/acquisition/ground_entry.py --output artifacts/a-new-ground-entry-run`.

This model starts a flat NOTE at rest in the world, not already held in the robot. Robot approach/deceleration is prescribed and its contact work is accounted separately. Two pinch rotors, sampled centered belt occupancy, uncalibrated friction/preload and regularized ground friction evolve continuously. The ideal dissipative brake is not a selected controller specification. Lateral/yaw, deformation and lift retention remain absent.

With the declared assumptions, the braked case settles at robotX187.888mm in1.262s. Coasting leaves the measured entry domain; braking earlier stops at227.859mm rather than the178–198mm diagnostic window. The zero-duty control ALSO reaches ground dwell through prescribed base work and passive belt drag. Therefore ground dwell is not proof of active capture, gripping or retention. The +/-5mm fixed-offset geometric counterexamples remain unresolved and are not simulated as successful recentering.

Halving timestep100→50us changes nominal finalrelativeX by0.0308mm and reduces maximum energy residual1.503→0.749mJ. This is limited numerical refinement, not contact calibration. Normalized overlap is a contact schedule only; loads and friction remain independent hypotheses. No scoring/physical safety inference follows from coast motion.

The next required model is piece retention during rotation/lift with finite contact patch and friction wrench/torque limits, not a rigid attached NOTE. The ground-position observation cannot authorize handoff to the launcher state.
