# Editable 2024 Fusion robot

The current display assembly is **FRC 2024 - Team 1780 - MAXSwerve Integration - Active Draft**, with the R7 boxed front crossmember. Authorized team members can open the named design in Fusion. Use the SHA-256 in the display receipt to identify its archived F3D. The nominal R7 replay uses that same F3D. The original D1 replay uses the earlier articulated archive identified by the layout record. [Display archive identity](../docs/media/swerve-r7.json) · [Scoring archive identity and component register](drivetrain/maxswerve-layout.json)

Four REV-21-3005 assemblies occupy the corners of a 520 mm square. Each contains 273 supplier solids in the private Fusion model. Each module has a drive motor, a steering motor, a fixed mount, and separate azimuth and wheel-roll joints. The public scene retains 612 team-authored bodies. Its purchased-part representations use dimensioned original display solids. The steering fork turns relative to the fixed motor housing and the hub rolls relative to the fork. [Frame build sheet](drivetrain/MAXSWERVE_FRAME.md) · [Native joint check](native/verify_swerve_joints.py) · [Installed model test](../tests/test_showcase_modules.py)

### Open the archived R7

In an authorized project checkout, use **File → Open → Open from my computer** and select `artifacts/native/r7/MAXSwerve-R7.f3d`. The immutable registered-front-crossmember export is 167,339,032 bytes. Its SHA-256 is `0db7b7df1ad6ecfae4f7185c6a5c36279711d8db643bfb5ac6a9031034c50517`. Check it before opening or exporting with `Get-FileHash -Algorithm SHA256 artifacts/native/r7/MAXSwerve-R7.f3d`. The editable archive includes restricted supplier solids. An authorized maintainer supplies this ignored file separately for a fresh clone. Save any changes as a new Fusion version. The public replay uses original display geometry.

The same private directory holds `physical-mass-r7.json` and `display-manifest.json`. The mass budget binds those files to this archive and partitions the raised mechanism from the chassis. [Budget, sensitivity cases and reproduction](drivetrain/R7_MASS.md)

### Inspect a revision

1. Check the active document name and archive identity before changing an editable design. Save a named revision for each installation change.
2. Check the four wheel centers in robot coordinates (+X forward, +Y left, +Z up) and the signs of all eight joints. Move each independently and return to the recorded pose.
3. Review the moving lift and cradle against the installed modules. Check the NOTE along the approach and the front crossmember against the bumper. Preserve the results with that revision. [Travel check](native/verify_swerve_travel.py) · [Bumper fit](drivetrain/maxswerve-bumper-fit.json) · [Crossmember](drivetrain/maxswerve-front-crossmember.json)
4. Use the published underside preset to inspect all four stations, then scrub the D1 replay. Its 188 moving bodies were registered against the earlier scored geometry. Choose **R7 · nominal mass D1 replay** to inspect the same-archive run. Its BOM estimate still needs as-built weighing. Wiring, starting configuration and formal repeatability remain open. [Inspect the robot](https://ehartye.github.io/frc-2024-poc/?view=swerve#model)
