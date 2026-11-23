"""Check or refresh the design still at the top of every project README.

This runs the Agent FRC README step (PLUGIN/scripts/cad-readmes.py), which reads cad-showcase.json at the repository
root. Without options it checks that each README shows the still block directly below its title, that the image
exists and that the caption names the current design and its status. --update rewrites every block.

Set AGENT_FRC_PLUGIN_ROOT to the Agent FRC checkout at the AGENT_FRC_REF in .github/workflows/design-publication.yml.
"""
import argparse
import importlib.util
import os
import subprocess
import sys
import types
from pathlib import Path

# These READMEs are not the project's own pages and are left as they are: the withdrawn showcase archive, and the
# recorded trial inputs with their vendored libraries.
KEPT = ("docs/detailed-design/archive/", "2024/")


def listed(*args, **kwargs):
    names = subprocess.check_output(*args, **kwargs).split(b"\0")
    return b"\0".join(n for n in names if not n.decode().startswith(KEPT))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--update", action="store_true", help="Rewrite every README still block")
    args = parser.parse_args()
    plugin = os.environ.get("AGENT_FRC_PLUGIN_ROOT")
    if not plugin:
        raise SystemExit("Set AGENT_FRC_PLUGIN_ROOT to the Agent FRC checkout.")
    scripts = Path(plugin) / "scripts"
    sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location("agent_frc_cad_readmes", scripts / "cad-readmes.py")
    step = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(step)
    # The step lists every README in the repository. Give it the list without the kept ones.
    step.subprocess = types.SimpleNamespace(check_output=listed)
    step.run(args.root.resolve(), args.update)
