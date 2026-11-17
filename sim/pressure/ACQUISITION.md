# Fresh same-law pressure acquisition entry

This adapter starts at the original resting robot/NOTE state: NOTE world X0.658 m, zero body/rotor velocities, zero new pressure potential and zero cumulative source work. It rejects legacy moving/contact checkpoint imports. It does not import or offset the old normal reservoir. The new constitutive law is used throughout the approach.

Persistent differential-drive, free planar NOTE and two pinch rotors share one supply. The electrical-only midpoint port follows the reviewed bus-independent start-force pattern; the unchanged513-grid supply solve is retained. Full selected-bus midpoint preview and end geometry checks precede an atomic state/energy commit. Original conservative hard-obstacle and finite-geometry gates remain active. Cumulative normal potential is derived from the new material grid, not stored as the old normal energy.

The NOTE remains under the explicit flat-ground constraint. Nonplanar pressure force/moment above1e-8 is rejected; floor support and tire load transfer are not a solved3D model. Ground friction and45 kg chassis hypothesis remain inherited. The old point-contact compression damping is NOT carried into this conservative new normal pressure law. All material/friction assumptions remain uncalibrated; cassette u stays0 and no spring force is adopted.

## First bounded result

A genuine0.58 s run at1 ms and1024x1024 material resolution reaches contact entry. First recorded midpoint contact is0.5615 s (accepted state time0.562 s). At0.58 s, NOTE velocity is approximately-0.00932 m/s, chassis speed0.82346 m/s, pinch rotor71.455 rad/s and bus11.9630 V. End geometry has540 wrap and1828 straight contacts; midpoint normal magnitude is0.80648 N per side. Potential grows from zero to0.46823 mJ.

The1.065 mJ global integration residual is predominantly inherited drive integration error accumulated before contact, not a pressure handoff jump. Residual change between0.561 and0.580 s is about0.278 microjoule. This is one-step-size evidence, not convergence. Runtime135.7 s for0.58 simulated seconds is not real time. No capture, lift, retention, scoring or whole-robot clearance is claimed.

Contact forces/cell summaries are midpoint samples; state and pressureFeatures are accepted end samples. Both timestamps are recorded. New material-grid phase is unchanged. A half-step fresh run and separate grid sensitivity are required before interpreting longer travel.

```text
python -m unittest discover -s tests -p test_pressure_acquisition.py -v
python -m unittest discover -s tests -p test_pressure_entry_receipt.py -v
python sim/pressure/run_acquisition.py --output artifacts/fresh-entry.json --horizon .58
```

The runner records all580 accepted states and actual commands with before/after source binding. Initial rest and zero new potential are asserted. Unknown obstacle/contact/controller domains preserve the last accepted state, never become clearance or success.
