"""Record the numerical stack used by a mission stage."""
import platform
from pathlib import Path


def runtime_versions():
    import mujoco
    import numpy as np

    return dict(python=platform.python_version(), mujoco=mujoco.__version__, numpy=np.__version__)


def verify_runtime_chain(stages):
    lock = Path(__file__).resolve().parents[1] / "mujoco/requirements-lock.txt"
    pins = dict(line.split("==") for line in lock.read_text().splitlines() if "==" in line)
    first = stages[0].get("runtime", {})
    if (not first.get("python") or first.get("mujoco") != pins["mujoco"]
            or first.get("numpy") != pins["numpy"]):
        raise ValueError("Official attempt requires recorded runtime versions matching the dynamics lock")
    if any(stage.get("runtime") != first for stage in stages[1:]):
        raise ValueError("Mission stages used different or unrecorded dynamics runtimes")
    return first
