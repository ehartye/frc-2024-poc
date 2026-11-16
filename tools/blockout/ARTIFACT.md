# Editable design artifact

The browser uses the shared agent-frc/design-artifact@1 module from plugin revision fb85947. Current profile is a parameterized 2024 Speaker blockout, not arbitrary CAD or all-season support.

Download design saves geometry parameters, frames, source references, narrative/requirements/decisions, reversible chassis bay operation, rehearsal assumptions, pseudocode and fabrication questions. Load validates before replacing state. Changes create a new revision and invalidate prior checks. Browser-local save is explicit and device-local; downloaded JSON is the portable source of truth.

Export CAD creates an editable OpenSCAD derivative in millimetres containing robot blocks and NOTE only. Retain JSON for traceability; field and target are not in this SCAD derivative. Native OpenSCAD execution has not been verified. Neither export is fabrication-ready.

The intake operation retains the continuous bumper and a front support-volume strip. This does not establish actual backing, attachments or pickup clearance. Rehearsal timing remains serial. Concurrent actions, degraded operation, game-piece wear and pit maintenance are planning sketches, not executable physical models.

## Local verification

Serve `dist` with a static HTTP server on loopback port 8765. Run `node verify-browser.cjs` with an existing Playwright installation; optional PLAYWRIGHT_MODULE and CHROMIUM_EXECUTABLE point to that installation. BLOCKOUT_URL can override the local preview. No dependency installation is performed by this test. Artifacts are ignored.

Verified locally: phone-width JSON download/load exact roundtrip, changed footprint geometry, narrative/pseudocode/fabrication retention, invalid units rejected without replacement, no JavaScript errors or horizontal overflow. The test also checks explicit local save/restore and millimetre CAD export. Browser state is not assumed to persist.
