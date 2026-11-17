# Raised, seated track-bearing assembly candidate

This additive module preserves the canonical robot and its prior counterexamples. It replaces eight undersized cylindrical track-bearing representations with nominal stepped profiles and reconstructs two original cradle weldments with their lower annular carriers raised 2 mm. Belt centers, shafts, pulleys, NOTE poses and robot architecture do not move.

## Fresh geometry gates
- Ten changed parts checked against the complete 711-part robot at 15 rotation/lift poses: no raw material intersections or invalid changed shapes.
- All eight exact native seated bearing shapes minus their original stepped representations leave zero remaining solids. Thus these specific nominal representations contain the imported native material under the tested transforms, not just its bounding dimensions.
- Whole rigid NOTE at 147 floor-entry poses: centered path has no changed-part hits; each fixed lateral offset of 5 mm retains eight bearing-contact poses. This does not solve arbitrary-offset pickup.
- Centered reverse local path covers aligned feed relative to these cradle-attached parts only. It does not establish whole-robot transfer clearance.
- Actual recorded free final NOTE position and pitch are preserved, not snapped to the cradle. No changed-part contact occurs at that one state.

## Remaining adoption gates
Independent review, axial shaft retention and installation access remain open. The unchanged lower shaft has only about 0.25 mm end allowance beyond the bearing. Nominal ground clearance is about 2.65 mm before carpet, tolerance or deflection. Finite poses are not a continuous swept-volume guarantee. All unchanged-to-unchanged interfaces and all forces/loads remain outside this delta audit. No source-native geometry is redistributed.

The candidate is not canonical or fabrication-qualified. `seated_bearing_assembly.build_pose` returns an editable original candidate from the existing source contract without changing the canonical generator. Native source material is used only privately by `check_seated_bearing_note.py`; archived scalar reports are in `evidence/seated-bearing-*.json`.
