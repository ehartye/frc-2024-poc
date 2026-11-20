# A2 powered-guide exploration

Fresh offline WPILib/DCMotor execution of `A2PoweredReleaseTest` explores the original A2 0.24 m, 60 degree guide. NOTE mass 0.235 kg, roller radius 0.0508 m, per-roller inertia 0.0003 kg m2, friction coefficient 0.8, 15/25/35 N assumed normal force per roller. Force/friction are hypotheses, not hardware specifications. Both rollers start at rest; no injected feed energy or prescribed release speed.

Twelve cases at 9/12 V and duty 0.6/0.8 passed integration and energy accounting. Best tested 12 V, 0.8 duty, 25 N case exits at 7.2577341265 m/s after 0.0625932463 s. Same configuration at 9 V exits at 6.1318669884 m/s. Baseline 12 V, 0.6 duty, 15 N exits at 6.4387451268 m/s. Increasing normal force does not automatically increase launch speed: shared supply, motor torque and roller/body dynamics remain coupled.

These are fresh plant results, not the earlier prescribed 7.5 m/s design target. Target aperture checks must use these actual outputs. Plant has no body attitude/rotation, compliance or physical normal-force actuator; fixed guide-aligned attitude is a separate explicit hypothesis requiring investigation. No full-field clearance, scoring, ground pickup, impact or hardware acceptance follows.

The new test does not alter the old public preloaded runtime or its claims. This isolated integration worktree is not yet a releasable repository snapshot: provenance and the final one-command mission runner must be regenerated after the full integration.
