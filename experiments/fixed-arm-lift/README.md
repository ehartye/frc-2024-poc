# Rejected fixed-arm lift experiment

<!-- cad-still -->
[![Connected floor-pose assembly](../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2024-poc/robot/cad/)

**Project CAD · Connected floor-pose assembly.** Six-wheel baseline with project-authored solids and supplier-sized envelopes. The swerve redesign is in development. [Source STEP · b08b08a8e7](../../cad/assembly/connected-floor.step) · [Rotate the model](https://ehartye.github.io/frc-2024-poc/robot/cad/)
<!-- /cad-still -->

This is not part of the runnable prototype. The fixed 0.68 m arm crosses an extension envelope during its sweep even though endpoint poses appeared plausible; the design worker is revising the geometry. Its finite-torque model must not be transferred to a telescoping mechanism without radial dynamics and actuator work.

The first native trial rejected unsupported regeneration in the unchanged SharedSupply model. An explicit hypothetical shorted-motor brake avoided that rejection, but the energy-residual assertion still failed at 0.5 ms integration steps. No convergence or energy validity was established. Power-off motion was observed; zero demand did not establish retention. Preserve these controls rather than treating them as a passing lift model.

Files are archived here, outside active Java source/test directories, pending a coherent revised geometry and dynamics model. No installed actuator, braking behavior or load acceptance is implied.
