# Standalone finite-area pressure law

<!-- cad-still -->
[![Preliminary robot with the scoring cassette raised above the blue bumper](../../cad/preliminary-design/media/002-preliminary-robot.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Guided cassette, parked. Preliminary design release 003.** The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. This still uses the same robot geometry as the interactive preliminary review. [Still and geometry record](../../cad/preliminary-design/media/002-preliminary-robot.json)
<!-- /cad-still -->

This is an explicitly hypothetical elastic penalty density, not measured NOTE foam, a fitted physical material, a retainer, or a new accepted phase handoff. The original weak-grip simulations and their28/30 witness-loss stops remain unchanged.

## Constitutive definition and work

Uniform material-coordinate torus samples carry their physical surface-area quadrature weights. Each sample queries the source-authored finite capsule-annulus belt SDF. For shallow supported penetration delta, the cell potential is `0.5 * k_area * delta^2 * area`; `k_area` has SI units N/m^3. Force and moment are derivatives of that discrete potential. No normal force is redistributed or renormalized when cells stop contacting. Equal/opposite cassette wrench is evaluated at the same point. Body, rigid cradle and positive-cassette `u` work are separate. The positive cassette translates along negative cradle Y by `u`, consistent with the frozen authored travel interface. All comparison fixtures use `u=0`.

Straight outer faces and finite-height caps have analytic SDF gradients checked against the original query. Entry wraps, cavity/deep penetration and gradient ties remain unsupported. This is sampled geometry, not whole-solid native contact proof. There is no friction/motor reaction, damping, moving-cassette inertia, spring equilibrium or opening actuator in this module. The source spring interface does not authorize equating its generalized load with NOTE normal force.

## Observed numerical limits

At indexed diagnostic pose and `k_area=1e7`,32x32 samples miss all contact. Per-side normal magnitude at64/128/256/512/1024 resolution is approximately1.77771/1.65521/1.62957/1.62712/1.62692 N. The legacy single-contact value was1.75521 N per side. These are comparisons, not a fit or selected successful preload. Stiffness sensitivity at1e6/1e7/1e8 is retained.

The1024-grid potential is0.952183 mJ versus the inherited normal reservoir1.540387 mJ: a model-change delta of-0.588203 mJ. That delta is NOT removed by choosing a new energy reference. The old checkpoint is only a diagnostic fixture. Any physical continuity claim needs acquisition rerun from the original no-contact start under the same new law.

Yaw of this axisymmetric torus changes no physical shape. Changing the surface-grid phase nevertheless produces numerical variations: at64 resolution the side force ranges1.3620 to1.7777 N; at1024 it ranges1.62660 to1.62692 N. The quarter-cell1024 sample has a numerical yaw moment about3.33 microNm. This is quadrature anisotropy, not real material torque. Finite-height-cap sampling also varies with phase. Coarse grids are not suitable for subsequent dynamics merely because an energy derivative test passes.

## Reproduce

Fresh output paths required:

```text
python -m unittest discover -s tests -p test_finite_area*.py -v
python sim/pressure/run_comparison.py artifacts/fresh-pressure.json
python sim/pressure/phase_sensitivity.py artifacts/fresh-phase.json
```

The comparison binds its implementation, source geometry/config, frozen cassette translation source, original checkpoint and later diagnostic event lineage before/after running. The phase study binds the checked-in comparison receipt and its own source. Frozen cassette metadata is used only for coordinate/translation provenance; the pressure runner does not replay all native mounting evidence.
