# Rejected fixed-arm lift experiment

<!-- cad-still -->
[![Preliminary design of the robot](../../cad/preliminary-design/media/guided-cassette-preliminary.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Project CAD · Preliminary design of the robot · Guided cassette 1.0 · effective.** The robot is at the start of a cycle, with a floor NOTE ahead. The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. The still shows the layout solids of the preliminary design review in its opening pose. [Open the design review](https://ehartye.github.io/frc-2024-poc/preliminary-design/)
<!-- /cad-still -->

This is not part of the runnable prototype. The fixed 0.68 m arm crosses an extension envelope during its sweep even though endpoint poses appeared plausible; the design worker is revising the geometry. Its finite-torque model must not be transferred to a telescoping mechanism without radial dynamics and actuator work.

The first native trial rejected unsupported regeneration in the unchanged SharedSupply model. An explicit hypothetical shorted-motor brake avoided that rejection, but the energy-residual assertion still failed at 0.5 ms integration steps. No convergence or energy validity was established. Power-off motion was observed; zero demand did not establish retention. Preserve these controls rather than treating them as a passing lift model.

Files are archived here, outside active Java source/test directories, pending a coherent revised geometry and dynamics model. No installed actuator, braking behavior or load acceptance is implied.
