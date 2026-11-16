# A2 continuous shooter adapter

`A2ShooterAdapter` owns one unchanged `PoweredGuidePlant` instance. It advances actual unloaded rotor dynamics, waits for actual RPM and dwell, calls `feed(0)` once, then continues that same plant. It never assigns RPM, constructs a replacement plant at release, resets the clock, or supplies fictitious feed energy. Before/after feed samples are exposed and tested for exact equality.

## Explicit hypothesis configuration

The tested A2 inputs are mass 0.235 kg, roller radius 0.0508 m, inertia 0.0003 kg m2 per roller, travel 0.24 m, incline 60 degrees, normal force 25 N each, friction 0.8, fixed duty 0.8, and 9/12 V supplies. Readiness 1800 RPM, 50 ms dwell and loaded release floor 1000 RPM are separate provisional control hypotheses, not measured hardware limits.

The plant only supports a fixed motor duty. Adapter requests must match that duty while enabled. Zero, changed or disabled demand after startup latches COMMAND_UNSUPPORTED and suspends further integration. The last RPM remains nonzero when appropriate. This does NOT simulate stopping, coasting, braking, electrical shutdown, gravity fall or any post-fault dynamics. Exit is also the plant's integration boundary, not a stopped rotor. A real adapter must support those dynamics before control claims extend beyond this scope.

Startup timeout, premature release, loaded RPM floor and release timeout remain faults. Zero-power and unreachable-readiness controls fail rather than supplying a fabricated RPM. No automatic lower-range or alternative aiming solution is generated.

## Fresh numerical results

At 50 microsecond steps with readiness 1800 RPM / dwell 50 ms:

| Supply | Actual prefeed time | Actual prefeed RPM | Exit speed | Exit RPM |
|---|---:|---:|---:|---:|
| 9 V | 0.112600 s | 2537.841 | 8.809413 m/s | 1672.689 |
| 12 V | 0.092200 s | 3050.278 | 8.810538 m/s | 2487.932 |

The overshoot above readiness occurs during the declared dwell; it is measured from the plant, not imposed. Across 25/50/100 microsecond runs, total energy residual magnitude stayed below 1.2e-10 J. At 12 V / 50 microseconds, accumulated motor work was 52.121330 J and modeled slip loss 22.157682 J. `feedEnergy` remains zero; stored rotor energy is retained. This is internal model accounting, not physical calibration or an uncertainty bound.

The unchanged from-rest control reproduces 7.257734 m/s and 1371.063 RPM. A stricter readiness 2500 / release floor 2500 test fails during the 9 V release at 2499.670 RPM, before exit. It passes at 12 V after longer spinup, exiting at 2693.688 RPM. Thus equal thresholds are supply-dependent, not universally invalid.

## Mission compatibility

`CrescendoMissionSupervisor.Config` now includes `releaseMinRpm` independently of `readyRpm`. Its compatibility constructor and original hypothesis retain equal thresholds, preserving existing behavior. The new explicit A2 integration test uses 1800/1000 RPM, samples actual plant RPM and actual exit, and maps a positive shooter request to the adapter's declared fixed duty 0.8. This mapping is an open-loop command policy, NOT an RPM regulator.

At 10 ms mission updates and 50 microsecond plant steps, the first observed release completes at global test times 0.20 s (9 V) and 0.18 s (12 V). Actual exit speeds are 8.810537791 and 8.810537998 m/s; loaded RPMs 1713.136 and 2560.327. The test injects drive/alignment/possession facts before release; it is not a physical drive or acquisition loop. Plant time begins at motor enable; the global mission clock includes the preceding disabled interval. The single-piece test stops after the first actual exit and does not reset the rotor to fake a second cycle.

New speeds require fresh target/pose analysis. Earlier from-rest launch margins and scoring assessments do not carry over. No official score, physical safety or full robot acceptance is claimed.

## Explicit reusable opt-in

A later additive mode, constructed with reusable=true, supports accounted body departure and open-circuit coasting across shots. The default single-use behavior described above is retained. See A2-MISSION-SIMULATION.md for the reusable energy ledger, power-off persistence controls and full-cycle scope. No physical brake or automatic zero-speed assumption is added.
