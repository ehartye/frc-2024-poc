# Free-acquisition stopped-state native diagnostics

The exact final and rejected-next states for the 2, 4 and 8 kN/m right-offset experiments were checked against private native track pulleys and bearings. All six states have zero common material volume at the source pulley clocking, for both unseated bearing flange orientations. This resolves these particular conservative cylinder stop witnesses as non-intersecting nominal material samples, not as a clear entire pickup trajectory.

The experiment receipt is pinned by SHA-256 and all its dependency files are verified before and after the native run. The current robot geometry must match the experiment geometry. Per-solid valid native material is checked, with conservative torus broadphase rejection recorded separately.

Critical limitations:
- Rotor angular phase is not integrated or registered. These fixed-clock witnesses cannot establish clearance at every rotating-tooth phase.
- Bearings retain the original unseated centers. The separate flange-seat candidate is not adopted here.
- Only the six flat nominal rigid NOTE states and track pulley/bearing material were checked, not every part or the intervening trajectory.
- Foam deformation, contact forces, retention and physical capture are not validated.
- Zero intersections do not authorize bypassing the conservative runtime gate without a further geometric/contact model.

Run `python cad/assembly/check_free_acquisition_witness.py --vendor-cache <private-cache> --experiment <immutable-experiment-worktree>`; archived scalar evidence is `evidence/free-acquisition-native.json`. Supplier geometry is not redistributed.
