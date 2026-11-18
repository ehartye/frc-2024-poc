# Bounded inward-domain braking experiment

The original 178 mm lower bound came from the sampled floor-entry study, not a physical stop or FIRST rule. The old 1 ms and 0.5 ms runs remain unchanged counterexamples: both reach 0.910 s before rejecting a query below that experiment bound, while the chassis is still moving about 0.1615 m/s.

## What changed

The additive gate permits center X in [138,658] mm. Its obstacle prefilter is rebuilt from all 720 inventory boxes using the new lower bound; its torus/cylinder material-candidate tests are unchanged. Actual finite belt geometry, pressure quadrature (1024 squared), constitutive parameters, controller, motor/supply, friction and integration are unchanged. Positive obstacle candidates still return UNKNOWN.

A synthetic newly relevant obstacle verifies the old prefilter cannot be reused. A grid check compares the rebuilt gate to all inventory obstacles. Centered 1 mm samples from 178 to 138 mm pass the finite adapter, but this is not whole-trajectory or physical qualification.

The .910 bridge changes only explicit domain identity. All physical state, rotor speed, controller history, held command, original next tick (.912), energy ledger and reconstructed potential are preserved. The new checkpoint schema is rejected by the old restore. No historical receipt is repinned.

## First 50 ms outcome

Both step sizes complete to .960 s. Relative X reaches 170.648225 mm; chassis speed falls to .127551 m/s and NOTE velocity is about -.002920 m/s. This is useful continuation beyond an arbitrary former guard, not stopped/captured/lift-ready behavior.

The last midpoint normal force is about 1.589 N per side. The accepted endpoint has 3864 straight-face and 200 wrap material cells. A finite footprint can begin reaching the wrap before its center passes the rear pulley station. Pressure potential drops from .952183 to .941853 mJ rather than being held constant by attachment.

The same-start half-step difference is 18.37 nm in relative X and 8.49e-8 m/s in chassis velocity. New-phase energy residual reduces from 1.687 to .424 microjoule. This is local temporal refinement only, not spatial convergence or calibration. Runtime was 90.0 and 182.9 seconds for 50 ms simulated; this is not real-time simulation. The retained hard-object prefilter has 40 candidates before and after extension; pressure evaluation remains the computational cost.

## Next predeclared experiment

`run_inward_extended.py` restores the exact .960 new-domain checkpoint and absolute controller clock, then requests 100 ms or the first UNKNOWN. It does not retune commands, snap to a pose or declare a low-velocity threshold to mean capture. The purpose is to observe braking and naturally changing finite contact near the rear wrap. Actual bearing seating, free-piece compliance, material calibration and future pitch/lift remain separate gates.
