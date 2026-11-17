# Same-grid conservative culling

`fast_geometry.prepare` is an additive acceleration of `finite_transport.prepare`, not a new contact law. It keeps the exact1024x1024 material sample coordinates/areas and evaluation order. The scalar reference remains unchanged.

The material array is cached read-only with a two-entry cache. A conservative whole-torus affine AABB can reject fully separated geometry. Otherwise vectorized coordinates generate only a padded union-of-belt-AABBs candidate mask. Every candidate is recomputed using the original scalar transforms, gradient, tangent and force preparation. This avoids the tiny rounding changes observed when vectorized transforms were originally used as actual contact inputs. No points are accepted solely because of the broadphase.

Bounds use the actual affine basis product, including accepted near-unit transforms, rather than normalizing away a scale and risking an underbound. Padding includes a fixed1e-12 m floor and a coordinate-scaled floating-point allowance. The exact scalar query retains deep/cavity/tie rejection. Point order and material-grid phase are not altered. No coarse grid is silently substituted.

Tests compare full contact dictionaries and serialized bytes, then the resulting force/torque/work results, across straight/wrap/cap/separated/near-boundary fixtures, randomized rigid frames, near-unit transforms, deep/tie failures and immutable-cache controls. The benchmark separately pins all source inputs before/after one1024-grid front-wrap measurement. Its cold/warm wall times describe that run only, not a whole simulation or real-time guarantee.

```text
python -m unittest discover -s tests -p test_pressure_fast_geometry.py -v
python -m unittest discover -s tests -p test_pressure_benchmark_receipt.py -v
python sim/pressure/benchmark_geometry.py artifacts/fresh-grid-benchmark.json
```

The unverified trajectory adapter is excluded from this optimization evidence. Fresh acquisition must start with zero new pressure potential, preserve the shared bus and obstacle gates, and establish timestep/grid sensitivity independently.
