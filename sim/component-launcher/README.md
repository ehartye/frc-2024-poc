# Current component-launcher integration slice

<!-- cad-still -->
[![Preliminary robot with the scoring cassette raised above the blue bumper](../../cad/preliminary-design/media/002-preliminary-robot.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Guided cassette at the start of a cycle, with a floor NOTE ahead. Preliminary design release 004, effective.** The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. This still uses the same robot geometry as the interactive preliminary review. [Still and geometry record](../../cad/preliminary-design/media/002-preliminary-robot.json)
<!-- /cad-still -->

This experiment uses the frozen f8a35ed component CAD: nominal tread radius48.26mm, shifted shooter pivotX188mm, and cradle handoff220mm before the wheel station. It replaces neither a complete robot simulation nor the separate CAD/mount validation.

Fresh native intersection samples use all four actual authored tread annuli and the full NOTE torus. They measure rigid occupancy, not force. Valid solid inputs and non-cancelling per-solid intersection volumes are required. Positive10mm samples span station offsets-80..80mm; this is not a continuous clearance/contact certificate.

## New counterexample

The launcher-only surrogate with a prescribed1.5m/s handoff reverses near-86.708mm before useful capture. Halving timestep reproduces it. A hypothetical2m/s handoff passes, but increasing an assumed speed is **not a design fix**. The current cradle's pinch belts are meant to keep driving during transfer; their contact/force/power coupling is absent here and is the next integration step. Preserve the reversal as an omitted-upstream-drive control.

A last-positive-contact sample from a reversing piece is not registered as a release. Only forward-departure cases enter the provisional thin-aperture screen. Current source geometry is used for the transform; the old provisional docking pose and fixed guide-aligned body attitude remain assumptions. No roof/wall clearance or score is established.

## Reproduce

Requires existing Python/OCP8. The four-drive surrogate and optional stateful power-policy files preserve reviewed dd3aada logic with UTF-8/LF normalization; UPSTREAM.json pins source and local bytes. Base-case9V+load does not apply brownout state logic and is not operability evidence.

```text
python sim/component-launcher/extract_geometry.py
python sim/component-launcher/run_current.py --output artifacts/a-new-launcher-run
python tests/test_current_launcher.py
```

The run rejects an existing output directory and stale CAD/contact pins, executes fresh trajectories, and binds trace/runtime/config/source bytes. All friction, normal force, total rotating inertia and entry velocity remain explicit hypotheses. The geometry does not include installed rivet heads or tread compression. Old launch success is not transferred.

## Powered-transfer kernel in preparation

`transfer_plant.py` provides a separate six-drive reciprocal-DC kernel: two upstream pinch drives and four launcher drives share a supply. It supports explicit virtual hold/release, independent duty commands, motor-off momentum and energy bookkeeping. Eight unit tests use synthetic occupancy tables to check shared current limits, invalid inputs, hold/release, free-body energy and timestep refinement. These are algorithm tests, not real robot results. The current source-bound transfer report is rejected because authored structure intersects the advancing NOTE. No successful powered-transfer trace or upstream force calibration is claimed. Pinch normal force, friction, inertia and virtual holding are still hypotheses requiring deliberate selection and sensitivity checks after the geometry is revised.
