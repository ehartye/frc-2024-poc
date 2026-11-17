# Floor acquisition remains part of the design

The connected CAD still includes a low side-pinch belt cradle, rotation mechanism and chain-lift carriage. The floor pose places the nominal NOTE center25.4mm above ground. Rotation to60 degrees and lift to center474.833mm lead to the shooter handoff. The handoff-only browser replay is not the entire robot cycle.

This new screen starts with a loose-piece location outside the robot, not an already-held NOTE. The full711-part floor-pose assembly is fixed while a rigid, flat, ground-tangent NOTE moves inward from centerX658 to178mm in49 steps. The centered path has no unresolved material overlaps. Side-belt geometric contact starts at the sampled centerX338mm and continues into the held position.

Two additional paths keep the NOTE at a prescribed lateral offset of minus/plus5mm. Each has11 sampled positions with overlaps against the near-side pulley-profile or lower-bearing representations. These counterexamples are preserved, not waived. They do not prove a free NOTE cannot be collected: real lateral recentering, foam deformation and contact impulses are absent from this rigid translation. Conversely, the centered path does not prove robust pickup.

Next gates are free-piece lateral/contact dynamics, floor friction, measured belt preload/compliance, motor-driven acquisition, and retention through rotation/lift. The apparent geometric indentation is not a force law. Power-off motion is a mission-dependent retention case, not an automatic safety claim. Exact native component witnesses may distinguish conservative representation overlap from actual material where needed.

Replay: `python cad/assembly/audit_floor_entry.py`. Source-bound147-pose receipt: `evidence/floor-entry.json`. This additive study does not alter the adopted f2be99b CAD or its current simulation registration.

## Exact material witnesses and retention footprint

Exact private source CAD confirms the centered nominal path is clear of pulley and bearing material across all49 positions. The three valid pulley solids and one valid bearing solid were checked individually. Both possible bearing flange orientations were screened. The offset cases are not merely conservative envelope artifacts: each plus/minus5mm prescribed path has11 pulley-contact poses (maximum about420.50 cubic millimeters per solid) and10 bearing-contact poses (maximum about61.93 cubic millimeters). Free recentering and deformation remain absent, so these are counterexamples to fixed-offset rigid clearance, not a verdict on physical acquisition.

The actual flanged bearing exceeds the current simplified intake bearing envelope. Its orientation and carrier seating must be explicitly designed before real-bearing fit is claimed. Centered floor translation and aligned guide advance traverse the same relative NOTE/cradle-component path in reverse under a rigid pitch transform; this equivalence applies only to those components, not the whole chassis or field.

The offset side belt first contacts at sampled centerX368mm, before the first bare-material hit atX358mm. That nominal10mm window motivates lateral contact dynamics instead of assuming either instant centering or inevitable failure.

A separate finite footprint report supplies15 verified belt-face points per side at floor, rotated and handoff poses for longitudinal offsets0,9.88835038313412,10 and20mm. Points span x=minus/plus12mm and z=minus/plus3mm relative to the NOTE center and include inward normals. They represent a possible compressed contact footprint, not measured pressure. One total preload per side must be distributed across its points. The exact9.88835mm offset preserves the recorded acquisition dwell position; it is not snapped to the ideal cradle center. Torque-capacity feasibility is distinct from finite-slip retention dynamics.

Evidence: `evidence/native-floor-entry.json` and `evidence/retention-patch.json`. The slow initial native attempt produced no receipt; the completed run uses a conservative tested torus/box rejection filter to avoid false-positive AABB work through the NOTE's hollow center. No collision exemptions were added.
