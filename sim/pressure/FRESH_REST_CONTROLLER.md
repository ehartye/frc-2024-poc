# Fresh-rest controller comparison

This is a separate trajectory under the same pressure law and 138â€“658 mm experiment domain, not a replacement of any archived state. The existing controller starts at true rest with goal 185 mm, speed cap 0.25 m/s, its original gains, and its original 10 ms clock. The robot, NOTE and pinch rotors start at zero speed; potential, kinetic and cumulative energy start at zero. No parameter is tuned to the previous final position.

The declared maximum horizon is 3 seconds or the first unsupported model state. The first contact and first-contact-plus-100-ms milestones are observations, not capture criteria. Durably saved `PARTIAL` files contain the full physical checkpoint, held command, next controller tick, events, trace and source pins. They are not final reports. The run keeps going after these saves.

For a fresh clone or a new output directory, use the filesystem-only entry point:

```powershell
python sim/pressure/run_fresh_rest_safe.py artifacts/fresh-rest-controller-3s.json nominal .001 3
```

The safe entry point only creates the output directory and delegates the frozen producer. Its actual 2 ms test matches the direct producer's entire non-timing report. Existing output or partial files are not overwritten.

The full 1024-by-1024 contact grid is retained. Fine contact currently costs roughly 1â€“2 seconds per simulated millisecond on the tested computer; a long contact-heavy run is not real-time. No grid reduction is silently applied.

`braking_headroom.py` is a separate diagnostic using the actual drivetrain's motor, current-limit, wheel/rotor inertia, tire-slip and drag equations. It clones a contact-state drivetrain and applies brake commands until a 5 mm/s observation threshold or 3 seconds. Its energy reference is local to that counterfactual, never injected into acquisition. It omits NOTE reactions and latency and is therefore **not a conservative full-robot stopping bound**. The same diagnostic can compare the old high-speed contact state with the new actual contact state without fitting a decay constant or moving the controller's goal.

No result here establishes capture, lift, spring preload calibration, complete hardware fit or physical operation.
