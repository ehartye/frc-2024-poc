# Run the 2024 project checks

Use Python 3.13, Node 22 and a full Git checkout of `ehartye/agent-frc`. Set `AGENT_FRC_PLUGIN_ROOT` to that checkout at the `AGENT_FRC_REF` in `.github/workflows/design-publication.yml`. Its history supplies the exact generators that issued the season evidence. Install the numerical dependency from `tests/requirements-ci.txt` in your project environment.

From the season root in PowerShell:

```powershell
python -m pip install --only-binary=:all: -r tests/requirements-ci.txt
$env:OPENBLAS_NUM_THREADS = '1'
$runner = Join-Path $env:AGENT_FRC_PLUGIN_ROOT 'scripts/project-checks.mjs'
node $runner (Get-Location).Path changed (git merge-base HEAD origin/main)
node $runner (Get-Location).Path full
```

The changed run checks layout and the CAD source, mesh, still, viewer and student READMEs every time. It selects additional suites by their inputs in `tests/ci.json`. Edits to that manifest or a workflow select every suite. The full run also regenerates historical evidence with the pinned revisions in `tests/ci_history.py`. Generators which write into the project run in a temporary checkout containing your current working files.

The **Design publication** workflow runs the same entry point on every push to every branch, after it checks the published designs. A manual run of that workflow performs the full suite.

## Check the published designs

```powershell
$check = Join-Path $env:AGENT_FRC_PLUGIN_ROOT 'scripts/design-publication.mjs'
node $check check (Get-Location).Path --skip-rerun
node $check check (Get-Location).Path
```

The first command compares the records, review scenes and status indexes in seconds. The second also reruns each layout check from the committed inputs, as the workflow does. This season serves `main:/docs`, so a merged pull request is the published site. [Publish and check designs](https://github.com/ehartye/agent-frc/blob/main/docs/design-publication.md)
