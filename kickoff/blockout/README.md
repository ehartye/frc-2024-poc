# Portable 2024 blockout

<!-- cad-still -->
[![Preliminary robot with the scoring cassette raised above the blue bumper](../../cad/preliminary-design/media/002-preliminary-robot.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Guided cassette, parked. Preliminary design release 003-1.** The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. This still uses the same robot geometry as the interactive preliminary review. [Still and geometry record](../../cad/preliminary-design/media/002-preliminary-robot.json)
<!-- /cad-still -->

This season-owned copy is independent of any hosted preview. From this directory, run an existing static server, for example `python -m http.server 8765 --bind 127.0.0.1 --directory dist`, then open the displayed local address. No remote browser service is required.

Source baseline: reviewed blockout integration 64bac59. Shared design-artifact module corresponds to agent-frc 0.7.0. Download/load the versioned JSON to preserve an editable artifact; browser state alone is not a durable project file. See ARTIFACT.md for scope and checks. Third-party Three.js files retain their MIT license and provenance.

This generic parameterized pre-vis is distinct from the A2 telescoping mechanism and full-cycle simulation. It does not silently import the A2 contract or validate that mechanism. It is a planning tool, not evidence of score, hardware safety or official inspection.

## Historical standalone snapshot

The files at this directory's root preserve an earlier spatial blockout and rehearsal application from the former main branch. Run `python -m http.server 8000 --bind 127.0.0.1 --directory kickoff/blockout` from the repository root to inspect that historical snapshot. Its `SOURCE.json` and `THIRD_PARTY.json` pin source bytes and licenses. It is not the current editable `dist/` pre-vis above or the MuJoCo robot viewer. No historical bumper or intake configuration establishes legality, capture or structural acceptance.

Static checks: `python -m unittest discover -s tests -p test_blockout_assets.py`. Optional browser checks: `node tests/blockout_browser.cjs` with existing Playwright and Chromium. The harness serves loopback-only, installs nothing and closes its server/browser.
