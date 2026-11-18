# Bounded controller engagement

The unchanged virtual-observation `SpeedApproach` controller is engaged at the verified 0.680 s pressure checkpoint, goal 0.185 m. Only controller state initializes. Plant momentum, pressure potential, clock and cumulative energy remain continuous. Three predeclared 100 ms cases use nominal control, stale observation from engagement, and disabled pinch demand.

All three accept 84 steps through 0.764 s, then reject the next midpoint at relative X 0.1777088313 m, outside the inherited [0.178,0.658] m experiment domain. The accepted center is 0.1780304026 m; chassis speed remains 0.580735819 m/s, down from 0.847529444 m/s. This is late braking and a model-domain stop, not a collision, capture, release or safety verdict. The three physical traces are exactly equal: modeled drivetrain braking is identical, while pinch commands remain below spinning-rotor back-EMF and produce no motor current. Fault state and commands differ; inertia does not disappear.

Global residual at stop is 1.581901958 mJ, including inherited 1.066179276 mJ; new-phase error is 0.515722682 mJ. This braking interval needs timestep refinement before precision/margin claims. No physical calibration or successful capture is established.

`lastGeometryQuery` in original policy receipts is the last accepted-state recheck during atomicity verification. Use the separately reconstructed `policy-evidence/boundaries.json` for the actual rejected next-step query. Reconstruction re-executes only that step, verifies identical rejection and unchanged state, and pins each source report hash.

Historical optimization source identity remains unchanged in these runs. Reviewed overflow correction is being adopted by a separate genuinely fresh run, not by repinning these receipts or importing them under changed source hashes.
