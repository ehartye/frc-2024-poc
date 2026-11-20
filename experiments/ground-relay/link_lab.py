"""Add the season-owned ground relay entry point to the generated kickoff lab."""
import sys
from pathlib import Path
path=Path(sys.argv[1])
marker=b'<p id="season"></p>'
link=b'<p><a href="ground-relay.html">Explore the Blue 3 ground relay</a></p>'
page=path.read_bytes()
if page.count(marker)!=1 or link in page:
    raise ValueError("Expected an unextended generated kickoff field lab")
path.write_bytes(page.replace(marker,marker+link))
