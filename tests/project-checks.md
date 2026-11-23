# Run the 2024 project checks

The project's CI is `.github/workflows/ci.yml`. It runs on every push to every branch, checks out this repository and nothing else, and runs only files in this repository. Use Python 3.13 and Node 22 to run the same commands from the season root:

```sh
node scripts/ci/scan.mjs .
node scripts/ci/site-check.mjs docs
python scripts/cad-readmes.py .
python -m pip install --only-binary=:all: -r tests/requirements-ci.txt
python tests/test_frame_stiffness.py
python tests/test_passive_service.py
python -m unittest discover -s tests -p "test_showcase*.py"
python tests/test_passive_entry.py
```

| Command | Fails when |
| --- | --- |
| `scripts/ci/scan.mjs` | A tracked file holds a personal machine path or a credential. `.branch-scan.json` lists the three evidence folders it skips. |
| `scripts/ci/site-check.mjs` | A design index and its folders disagree, a file an index or record names is missing, or a relative link in a published page leads nowhere. |
| `scripts/cad-readmes.py` | A project README lacks the CAD still below its title, or its caption does not name the current design and status. `--update` rewrites every block. |
| `tests/test_frame_stiffness.py` | The recomputed R9 and R13 frame stiffness differs from the recorded study. Set `OPENBLAS_NUM_THREADS=1`. |
| `tests/test_passive_service.py` | A service fixture record no longer matches the Fusion archive it names. |
| `tests/test_showcase*.py`, `tests/test_passive_entry.py` | The archived draft-001 showcase under `docs/detailed-design/archive/` no longer matches its records. |

The whole run takes under a minute. The other files in `tests/` belong to archived studies and are run by hand when their study is revisited.

CI runs no layout check and no design check. A design is checked before it is pushed, and an effective design does not change. This season serves `main:/docs`, so a merged pull request is the published site.
