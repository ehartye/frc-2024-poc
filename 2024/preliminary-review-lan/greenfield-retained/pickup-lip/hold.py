from pathlib import Path
import json
p=Path(__file__).resolve().parent
s=(p.parent/'mission-physics/flap_ablation.py').read_text()
s=s[:s.index("if __name__")].replace("here=Path(__file__).resolve().parent","here=Path(__file__).resolve().parent.parent/'mission-physics'").replace("(.3995,0,.058","(.4575,0,.070").replace("s>.3995","s>.4575")
exec(s)
rows=[run('pickup',gate,'coast',.024/n,n) for n in (48,96) for gate in (False,True)]
(p/'power-off-hold.json').write_text(json.dumps(rows,indent=2));print(rows)

