# Electrical-only midpoint port experiment

The reference coupled model and `shared_supply.solve_supply` are unchanged. The earlier scan-only candidate is not used here. This experiment keeps all 513 grid samples, multiple-equilibrium rejection, derivative classification and signed-regeneration policy.

## Why this is equivalent within the current explicit midpoint model

At the start of an interval, tire and belt forces depend on the persistent velocities, rotor speeds, geometry and normal loads, not the candidate bus voltage. Their forces/torques can be computed once. Midpoint position and heading use the start velocities, so finite contact geometry is also bus-independent. Voltage changes the initial electromagnetic torques, hence midpoint rotor speeds/back EMF, hence midpoint bus power. `make_port` computes those changing electrical quantities with the same operation order and clipping laws as the reference. It does not replace the contact mechanics, chassis response or energy ledger.

The factory snapshots initial NOTE and drive state. It validates start and midpoint geometry, including unsupported-domain results. A candidate query returns only `busPowerW`, just like the current coupled runner's port. The original full preview still runs once at the solved voltage; its start/midpoint controller-voltage checks, final contact gate, state updates and energy accounting remain authoritative. Current-limit flags must not be inferred solely from the reduced scalar port.

## Local integration interface

```python
port = make_port(model, dt, drive_demands, pinch_demands,
                 normal_loads, modes, geometry=cached_geometry)
supply = solve_supply({'robot': port}, source_v, source_r)
# Keep the existing full preview, domain checks, and commit below unchanged.
```

Rebuild the port for every step. Do not mutate state/configuration between preparation, supply solution and commit. This derivation is specific to the current explicit midpoint force laws; a voltage-dependent contact force or different integration scheme requires revalidation.

`ElectricalOnlyCoupled` is an experimental comparison harness: it derives the reference step by replacing exactly its local voltage-query block. It refuses to load if that expected block changes. This avoids a silently divergent duplicate state/energy integrator. The coordinator can adopt the factory explicitly after review rather than deploy the dynamic comparison harness.

## Verification

```sh
python -m unittest discover -s tests -p test_electrical_midpoint.py -v
python sim/integrated/verify_electrical_midpoint.py artifacts/fresh-midpoint-evidence.json
```

The tests cover 120 seeded state/load/duty/mode combinations, 720 candidate voltages, all 513 scan voltages with contact, coast/brake, signed pinch states, reaction loads, query purity and atomic domain/current-limit failures. Exact power equality is required, not a relaxed fitted-power tolerance. The whole-trace runner compares every returned state/energy row and exception against the reference, binds sources before/after, and refuses existing output. Its evidence records the actual speed ratio and model-domain stop. No calibration, successful capture, scoring or real-time performance follows from computational equivalence.

## Recorded full-run result

Both implementations completed exactly 752 steps before the same `OUTSIDE_BOUNDED_PLANAR_EXPERIMENT` stop; first contact occurred at 0.562 s. Every full row and the terminal exception match exactly, with identical accumulated trace digests. Peak absolute energy residual is 1.066 mJ. Reference runtime was 56.947 s versus 11.806 s electrical-only, about 4.82 times faster on this bounded run. This remains slower than real time and does not enlarge the accepted geometry domain.
