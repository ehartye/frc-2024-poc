# Floor acquisition remains part of the design

The connected CAD still includes a low side-pinch belt cradle, rotation mechanism and chain-lift carriage. The floor pose places the nominal NOTE center25.4mm above ground. Rotation to60 degrees and lift to center474.833mm lead to the shooter handoff. The handoff-only browser replay is not the entire robot cycle.

This new screen starts with a loose-piece location outside the robot, not an already-held NOTE. The full711-part floor-pose assembly is fixed while a rigid, flat, ground-tangent NOTE moves inward from centerX658 to178mm in49 steps. The centered path has no unresolved material overlaps. Side-belt geometric contact starts at the sampled centerX338mm and continues into the held position.

Two additional paths keep the NOTE at a prescribed lateral offset of minus/plus5mm. Each has11 sampled positions with overlaps against the near-side pulley-profile or lower-bearing representations. These counterexamples are preserved, not waived. They do not prove a free NOTE cannot be collected: real lateral recentering, foam deformation and contact impulses are absent from this rigid translation. Conversely, the centered path does not prove robust pickup.

Next gates are free-piece lateral/contact dynamics, floor friction, measured belt preload/compliance, motor-driven acquisition, and retention through rotation/lift. The apparent geometric indentation is not a force law. Power-off motion is a mission-dependent retention case, not an automatic safety claim. Exact native component witnesses may distinguish conservative representation overlap from actual material where needed.

Replay: `python cad/assembly/audit_floor_entry.py`. Source-bound147-pose receipt: `evidence/floor-entry.json`. This additive study does not alter the adopted f2be99b CAD or its current simulation registration.
