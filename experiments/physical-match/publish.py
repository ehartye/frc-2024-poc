"""Add 2024-specific supply context and navigation to the pinned shared lab."""
import sys
from pathlib import Path

def replace_once(content, before, after):
    if content.count(before) != 1:
        raise ValueError("Expected one generator marker: "+before)
    return content.replace(before, after, 1)

def publish(path):
    html=path.read_text(encoding="utf-8")
    html=replace_once(html,'<p id="season"></p>','<p id="season"></p><p><a href="ground-relay.html">Explore the Blue 3 ground relay</a> · <a href="https://github.com/ehartye/frc-2024-poc/blob/main/experiments/physical-match/study.md">2024 stock and handling assumptions</a></p>')
    html=replace_once(html,'<div id="replayRoster"></div>','<h3>Teammate defaults</h3><div id="replayRoster"></div>')
    html=replace_once(html,'<summary>Stations and human service</summary>','<summary>Stations and human service</summary><p>Each SOURCE has one human player, a four-second handling cycle and space for one waiting NOTE. Human supply begins in TELEOP. Each used preload comes from its alliance’s 48-piece allocation.</p>')
    # Keep the two scores visible when a student scrolls down to roster or playback controls.
    start=html.index('<section class="match-console"')
    strip=html.index('<div class="score-strip">',start)
    end=html.index('<figure class="rehearsal-board">',strip)
    html=html[:start]+html[strip:end]+html[start:strip]+html[end:]
    html=replace_once(html,'</head>','<style>#rehearsal>.score-strip{position:sticky;top:0;z-index:4;background:#142f50;color:white;border-radius:6px 6px 0 0}#rehearsal>.match-console{border-radius:0 0 6px 6px}</style></head>')
    path.write_bytes(html.encode())

if __name__=="__main__":
    publish(Path(sys.argv[1]))
