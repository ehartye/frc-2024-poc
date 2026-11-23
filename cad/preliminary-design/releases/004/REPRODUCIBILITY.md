# Reproducing the release 004 trials

`EV/` means `2024/preliminary-review-lan/greenfield-retained/`. Run every command from the repository root.

## Environments

Two Python environments are needed. Neither is in the repository.

| Use | Packages | Pinned by |
| --- | --- | --- |
| Physics trials (shots, pickup, retention, climb) | MuJoCo 4.0.1, NumPy 2.5.5 | `sim/mujoco/requirements-lock.txt` |
| Layout build and STEP revision | `cadquery-ocp`, NumPy | The `frc-cad` environment |

Create the trial environment from the lock file:

```
python -m venv .venv-trials
.venv-trials/Scripts/python -m pip install -r sim/mujoco/requirements-lock.txt
```

Create it outside the checkout, or add its folder to `.gitignore` first. The shot solvers also read two folders that are in the repository: `2024/six-speaker-20261118/field-source/` (field geometry) and `2024/six-speaker-20261118/field-query-libs/` (bundled libraries).

## Running

| Trial set | Command | Output |
| --- | --- | --- |
| Single-shot scoring campaign | `python EV/scoring-window-04/campaign.py STAGE --workers N` then `python EV/scoring-window-04/analyse.py` | `results/STAGE.jsonl`, `summary.json` |
| Pickup brackets | `python EV/pickup-brackets-04/run.py`, with `2024/six-speaker-20261118/field-query-libs` on `PYTHONPATH` | `results.json` |
| Retention brackets | `python EV/retention-brackets-04/run.py`, with the same `PYTHONPATH` | `results.json` |
| Climb brackets | `python EV/climb-brackets-04/run.py` | `results.json` |

The campaign defaults to 12 workers. Set `--workers`. It writes scratch folders named `_w-*` beside the trial folders; `.gitignore` excludes them. A finished case is skipped on a rerun. The solver is deterministic, so a rerun of one case must reproduce its `shotSha256`.

## The `R:` drive

Commit `8b4d86b` replaced one workstation's paths with a drive letter, `R:`, mapped to the repository root with `subst`. The owner rejected that. Paths must be relative to the repository. The release 004 scoring campaign no longer needs the mapping. Archived scripts still do.

### Inventory

Counted on checkpoint `466c38e` with the pattern `R:/` or `R:\` not preceded by a letter. Two of the `EV/` scripts have been fixed since (see below), which leaves 710 files.

| Where | Files | Kind |
| --- | --- | --- |
| `2024/six-speaker-20261118/`, top level | 611 | Archived scripts (`.py`) |
| `2024/six-speaker-20261118/`, top level | 7 | Logs. They hold 1,394 of the occurrences |
| `EV/` | 46 | Scripts (`.py`) |
| `EV/` | 39 | Records: 34 logs, 3 text files, 1 JSON, 1 `.mjs` script |
| `2024/preliminary-review-lan/`, elsewhere | 5 | 4 scripts, 1 log |
| `plugin-qa/` | 3 | 2 logs, 1 JSON |
| `iterations/001/retrospective.json` | 1 | Record, 143 occurrences |
| **Total** | **712** | 4,678 occurrences: 661 Python scripts, 1 `.mjs` script, 50 records |

A further 38 files hold `%USERPROFILE%` in place of a home folder. In `EV/layout.py` line 23 it is a literal entry on the import path, which Python does not expand. That script cannot find the plugin as written.

`cad/preliminary-design/source-handoff/scrub-map.json` holds the SHA-256 of each rewritten file before and after the rewrite.

### What the release 004 trials load

| Trial set | Scripts that name `R:` | Status |
| --- | --- | --- |
| Scoring campaign | `EV/counter-drive-01/counter.py` line 6 | Fixed on 2026-11-22. One archived shot reruns to the same `shotSha256` |
| Scoring campaign | `EV/mission-physics/shot.py` lines 8 and 10 | Fixed on 2026-11-22. Two archived shots rerun to the same `shotSha256`, and the shot opens no file through `R:` |
| Pickup brackets | None | — |
| Climb brackets | None | — |
| Retention brackets | `EV/six-piece-02/loaded.py` | Possible. Found by a static trace only |
| Cycle trials, `EV/counted-reset-power-02/` | `shot.py`, `mission.py`, `collection_window.py`, `counter_clear.py`, `terminal_contacts.py`, `EV/counter-drive-01/counter.py`, `EV/loaded-actuation-01/purchased_power.py`, `EV/mission-physics/shot.py` | 13 lines in 8 files. Needed only to rerun the full six-NOTE cycle |

`EV/scoring-window-04/shot.py` already takes the repository from its own location. So do the three bracket runners.

The second dependency was found by logging every file one campaign shot opens. After the `counter.py` fix the shot still opened five field files under `2024/six-speaker-20261118/field-source/blue-speaker/` and `sim/pressure/speaker_score_observer.py` through `R:`. Both came from `EV/mission-physics/shot.py`. After its fix the log holds no `R:` path.

### Options

Ranks compare the three options. 1 is best.

**A. Fix the scripts the release runs.** One line in `EV/counter-drive-01/counter.py` and two lines in `EV/mission-physics/shot.py`: both done. The other six cycle-trial scripts when the cycle is next rerun. Leave the archive as it is and say in `source-handoff/README.md` that `R:` there is a placeholder for the repository root.

- Time: minutes now. No lag later. Rank 1.
- Risk: low. One deterministic case reruns to prove the same `shotSha256`. Rank 1.
- Complexity: one line reads the root from `__file__`. Rank 1.
- Best practice: the live path is right; about 650 archived scripts still name a drive. Rank 3.
- Maintainability: an archived script fails clearly if someone runs it unmapped. Rank 2.

**B. A, plus strip the prefix from the 50 records.** Logs, text and JSON records carry the path as text only. Removing `R:/` leaves a repository-relative path and cannot change behaviour.

- Time: about an hour with a script and a hash update. Rank 2.
- Risk: low. Records are not executed. Hashes in `scrub-map.json` change again. Rank 2.
- Complexity: one more column in `scrub-map.json`. Rank 2.
- Best practice: all records clean; archived scripts still name a drive. Rank 2.
- Maintainability: same as A for scripts. Rank 2.

**C. B, plus rewrite all 661 scripts.** Each literal becomes a path built from the script's own location.

- Time: a day or more, mostly verification. Rank 3.
- Risk: high. Trial scripts patch each other's source by exact text match and assert the match count. A rewritten literal can break a patch silently or change a pinned hash. Most archived scripts cannot be rerun to prove equivalence. Rank 3.
- Complexity: literal forms vary (raw strings, strings inside patched source). Rank 3.
- Best practice: nothing names a drive. Rank 1.
- Maintainability: best once verified. Rank 1.

Recommendation: A now, B with the coordinator's go-ahead. Do C only for a script when it is next run.

### The changes for A

Done. In `EV/counter-drive-01/counter.py`, `p` is already the script's folder. Line 6 is now:

```
fieldfolder=p.parents[3]/'2024/six-speaker-20261118/field-source'
```

Done. `EV/mission-physics/shot.py` is run as text from other folders, so `__file__` is the caller's script, not this file. Lines 8 and 10 now search upward for the repository:

```
repo=next(d for d in here.parents if (d/'sim/pressure').is_dir());root=repo/'2024/six-speaker-20261118'
sys.path[:0]=[str(root/'field-query-libs'),str(repo/'sim/pressure')]
```

Still to do: update both entries in `cad/preliminary-design/source-handoff/scrub-map.json`, and repeat the check once with the drive unmapped.
