# Portable 2024 blockout

This season-owned copy is independent of any hosted preview. From this directory, run an existing static server, for example `python -m http.server 8765 --bind 127.0.0.1 --directory dist`, then open the displayed local address. No remote browser service is required.

Source baseline: reviewed blockout integration 64bac59. Shared design-artifact module corresponds to agent-frc 0.7.0. Download/load the versioned JSON to preserve an editable artifact; browser state alone is not a durable project file. See ARTIFACT.md for scope and checks. Third-party Three.js files retain their MIT license and provenance.

This generic parameterized pre-vis is distinct from the A2 telescoping mechanism and full-cycle simulation. It does not silently import the A2 contract or validate that mechanism. It is a planning tool, not evidence of score, hardware safety or official inspection.
