# Extending an experimental boundary without inventing success

The original 178 mm lower NOTE-center bound came from the floor-entry study range, not a FIRST rule or a demonstrated robot obstruction. It was used both as a query guard and to prefilter obstacles. Merely removing the guard could therefore omit rearward obstacles.

The additive experiment extends the bound to 138 mm and rebuilds candidates from the complete saved obstacle inventory. The force law, finite belt/end-wrap geometry, quadrature and material/domain rejection rules are unchanged. Tests compare the prefilter with the full inventory and include an obstacle that the old prefilter would miss. The original rejected continuation remains archived.

The domain transition preserves the physical state, momentum, potential and cumulative energy. It has a distinct checkpoint identity. Controller continuity also preserves the held command and absolute tick phase: at 0.910 s, the next update is 0.912 s, not an extra update at the transition.

## Observed first segment

Both 1 ms and 0.5 ms runs continue from 0.910 to 0.960 s. NOTE-relative position reaches 170.648225 mm, beyond the old cutoff, while chassis speed decreases from about 0.1615 to 0.1276 m/s. The robot and piece remain moving. The endpoint has both straight and wrap contact samples, with about 1.589 N normal force per side under the explicit pressure hypothesis.

The endpoint position differs by 18.4 nm between these two runs. New-segment energy residual decreases from 1.687 to 0.424 microjoules. This is local timestep evidence, not physical calibration, spatial convergence, capture, retention or lift success. Full-CAD spot checks and the conservative obstacle inventory are sampled/source-sized evidence, not continuous collision qualification.

The rebuilt gate retains the same 40 real obstacle candidates in this configuration. Its query microbenchmark cost is effectively unchanged; the 1024-square contact integration dominates runtime. Extending the model domain therefore primarily costs additional validation, not a weaker force law. Contact decreases toward the rear of the finite belt and may disappear farther inward; that behavior must remain in subsequent runs.

## Evidence and limits

See `inward-domain-static-evidence.json`, `inward-continuation-1ms-evidence.json`, `inward-continuation-half-evidence.json`, and `inward-continuation-comparison.json`. Independent review checked source inventories, all saved command/energy rows, comparison reconstruction, targeted physical replay and scheduler split cases; it did not repeat both full 50 ms runs.

These studies remain flat-ground, bounded model experiments with assumed material and drivetrain parameters. A removed artificial cutoff is not itself a successful robot task. Subsequent continuation must retain actual finite contact, state and controller history rather than attach the piece ideally or reset its motion.
