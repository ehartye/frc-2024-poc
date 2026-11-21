# Run the 2024 project checks

Use Python 3.13, Node 22 and a full Git checkout of `ehartye/agent-frc`. Set `AGENT_FRC_PLUGIN_ROOT` to that checkout at the owner-selected `AGENT_FRC_PLUGIN_SHA`. Its history supplies the exact generators that issued the season evidence. Install the numerical dependency from `tests/requirements-ci.txt` in your project environment.

From the season root in PowerShell:

```powershell
python -m pip install --only-binary=:all: -r tests/requirements-ci.txt
$env:OPENBLAS_NUM_THREADS = '1'
$runner = Join-Path $env:AGENT_FRC_PLUGIN_ROOT 'scripts/project-checks.mjs'
node $runner (Get-Location).Path changed (git merge-base HEAD origin/main)
node $runner (Get-Location).Path full
```

The changed run checks layout and the CAD source, mesh, still, viewer and student READMEs every time. It selects additional suites by their inputs in `tests/ci.json`. Edits to that manifest or a workflow select every suite. The full run also regenerates historical evidence with the pinned revisions in `tests/ci_history.py`. Generators which write into the project run in a temporary checkout containing your current working files.

GitHub runs the same entry point on pull requests targeting main and pushes to main. A manual **Project evidence** run performs the full suite. Feature branches can run the changed command locally before opening a pull request.

## Check the branch-served showcase

This season serves `main:/docs`. With the independently maintained release configuration in `$trustedConfig`, run the following before promotion:

```powershell
$release = Join-Path $env:AGENT_FRC_PLUGIN_ROOT 'scripts/local-showcase-release.mjs'
node $release check $trustedConfig (Get-Location).Path
```

The release owner keeps the approved site snapshot, its digest, the approved commit and the public review key outside the candidate checkout. For an independent field-only change, supply the separately regenerated field output. Changed model bytes require paired source and published receipts. See the [mechanical release policy](../.github/mechanical/release-policy.md). The required hosted **Mechanical model eligibility** gate uses its separately approved verifier pin.
