# Airborne obstruction characterization

Additive to the preserved `36bede8` arena baseline; upstream remains exact
`1b1e8ea7278de01d40a5ea14dffbaaf13381512a`, unmodified.

Run `python sim/maple/verify_projectile.py --maple-source <clean-pinned-checkout>`.
This selects only ProjectileCharacterizationTest, records all compiled source
inputs before/after, and writes a separately scoped receipt. No GUI or install.

Launch center (2,2,1) m, velocity (5,0,4) m/s. Target (5,2,1.42) m,
componentwise tolerance .05 m. Upstream uses gravity11 m/s2 and ignores drag.
The open, blocked and displaced runs use exactly the same launch. The blocker
has center (3.5,2,1.705), size (.4,.6,.6) m; displaced center Y=4.
Its actual dyn4j XY fixture is registered in the selected arena. The height is
an independent 3D test envelope, not something Maple applies to airborne pieces.

At t=.3 s the independently calculated projectile center equals the blocker
center. The bounding sphere of the upstream 14-inch-diameter, 2-inch-high NOTE
cylinder has radius hypot(.1778,.0254), less than every blocker half-extent.
Thus the entire finite proxy is inside the box: this is stronger than a
centerline or overlapping-AABB test. The expanded-X interval only bounds
possible contact, not earliest actual impact. The displaced box is separated
in Y for the entire flight. This is a .2 kg Maple circle/cylinder proxy, not
our earlier .235 kg torus or a measured foam contact model.

Ground control initializes the same upstream NOTE body at (3,2) m with vx=5.
No pose/velocity overrides follow initialization. Original attempted ground
start X=2 stopped short under inherited damping, so its failed assertion is
retained privately; moving the starting body closer makes obstacle interaction
observable without changing physics. The failed setup was not a collision bug.

Expected interpretation: identical airborne trajectories and target callbacks
with and without the intersecting obstacle establish the documented missing
collision space. Callback is NOT physical shot acceptance or scoring proof.
Ground control distinguishes absent airborne collision from an unregistered
obstacle. No upstream equations, contact, gravity or callback semantics changed.
