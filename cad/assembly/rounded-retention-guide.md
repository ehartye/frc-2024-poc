# Rounded wear-guide/contact candidate

Exact input: indexed endpoint X188.5674400200612 mm, longitudinal cradle offset10.5674400200612 mm. The source-bound runtime receipt and full state are retained; no velocities or rotor state are reset.

The original candidate is a1 mm low-friction polymer wear sheet,250 mm wide, on the inboard face of the existing rigid front backing support. Its face is X355.362 mm; the existing support face is356.362 mm. It spans Z58.5..150 mm with0.5 mm round long edges. Two4.5 mm provisional attachment bores lie at Y±110,Z110. These are hole-location hypotheses, NOT a completed mounting stack. The existing structural support and backing are not removed or relaxed.

The added sheet deliberately increases inward occupancy. Sixteen conditional pitch probes retain both original support/backing contacts and added guide overlap. Surface witnesses provide points and outward normals on the proposed face/lower round edge for a later bounded contact model. They are not normal forces, measured foam compression or successful guidance.

A native-kernel discrepancy is explicitly retained at20 degrees: Boolean common volume is zero while interior material classification witnesses exist. This case is UNKNOWN, not clear. Other early angles also have positive guide occupancy, peaking around3159 mm3 at10 degrees. No overlap is automatically unsafe, and none is automatically waived as harmless foam deformation.

Next gate: select/implement guide fastening and material; then use actual contact coordinates, a bounded foam/guide force/friction model, finite pitch-actuator work and side-belt preload. Preserve force and energy budgets and actual incoming state. The existing end-state friction budget still fails static lift and is not repaired by this geometry proposal.

Runner: `python cad/assembly/rounded_retention_guide.py --runtime <source-bound-runtime-worktree>`. Original editable STEP at `artifacts/rounded-retention-guide/original-guide.step` reopens one root; supplier CAD is not present. Scalar evidence is archived in `evidence/rounded-retention-guide.json`. Canonical robot unchanged.
