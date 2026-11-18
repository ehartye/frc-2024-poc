# Explicit fast coupled acquisition implementation

`fast_coupled_acquisition.FastCoupledAcquisition` is an additive subclass of the immutable reference `CoupledAcquisition`. Its explicit `step` method replaces only the voltage-query block with `electrical_port.make_port`. All final mechanical preview, controller-voltage checks, end-geometry gate, state updates and energy accounting remain the reference operations in the same order. It uses the unchanged full-grid `shared_supply.solve_supply`.

The production import path contains no dynamic source replacement, `exec`, `eval`, or code generation. `electrical_port.py` contains the reviewed pure factory copied verbatim into a standalone module. The earlier dynamic derivation remains an unchanged experiment, not a runtime dependency of this subclass.

```python
from fast_coupled_acquisition import FastCoupledAcquisition
plant = FastCoupledAcquisition(drive_config, note_config, geometry, initial_note)
row = plant.step(dt, drive_demands, pinch_demands, normal_loads,
                 source_v=12., source_r=.02, modes=('drive', 'drive'))
```

No caller should replace historical source pins or relabel an old receipt. A new runner must bind the new module, pure factory, unchanged reference and all model/geometry inputs. The standalone comparison runner does this:

```sh
python -m unittest discover -s tests -p test_fast_coupled.py -v
python sim/integrated/verify_fast_coupled.py artifacts/fresh-fast-equivalence.json
```

Tests require exact factory source equality with the reviewed experiment, exact reference-step equality except the intended port block, no dynamic code in production modules, exact stepped states/energy and atomic domain failures. The fresh report compares the whole nominal trajectory against the unchanged reference, including its terminal unknown-domain boundary. Hypothetical forces, incomplete robot integration, finite geometry scope and lack of physical calibration remain unchanged.
