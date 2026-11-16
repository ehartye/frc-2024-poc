# 2024 spatial blockout and rehearsal

This is the actual standalone static application source, including locally vendored Three.js 0.160.1. It explores occupied volumes, reach and assumed route timing. It does not select a robot architecture or prove pickup, shooting, impact survival, official legality or calibrated control behavior. This browser study is separate from the Java simulation slice.

## Run locally

From the repository root with an existing Python 3 installation:

```sh
python -m http.server 8000 --bind 127.0.0.1 --directory tools/blockout
```

Open `http://127.0.0.1:8000` in a current WebGL-capable browser. Stop the server with Ctrl+C. Serving only on loopback keeps this preview local. No build, cloud account, deployment configuration or package installation is required. Do not open index.html directly using a file URL; ES modules and JSON loading require HTTP.

Use the pose/height/reach controls, intake-opening toggle and starting/deployed checks. Rehearsal uses constant assumed drive speed, turn-before-drive segments, action waits and a scripted crossing robot. Footprint overlap marks potential contact exposure, not force, damage, penalty or a successful action. Target registration remains provisional. Field structures not drawn are not certified free space.

## Files and checks

`app.js` is the 3D view; `rehearsal.js` is route playback; `blockout.json` contains source-linked geometry and assumptions. `SOURCE.json` pins the backed-up static bytes. `THIRD_PARTY.json` pins upstream vendor files and URLs; the MIT vendor license is in `vendor/LICENSE.txt`.

Run `python -m unittest discover -s tests` for static-source and public-content checks. Optional live-browser verification uses an already installed Playwright package and Chromium:

```sh
node tests/blockout_browser.cjs
```

If needed, set PLAYWRIGHT_MODULE to an existing module path and CHROMIUM_EXECUTABLE to an existing browser executable. The harness starts an ephemeral loopback server, blocks non-loopback network requests, exercises desktop/mobile layouts and rehearsal controls, then closes the browser/server. It installs nothing and creates no screenshots or deployment artifacts.
