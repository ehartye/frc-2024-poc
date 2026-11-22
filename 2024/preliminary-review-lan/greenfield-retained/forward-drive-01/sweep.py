from pathlib import Path
p=Path(__file__).resolve().parent;prior=p.parent/'frame-pocket-01/sweep.py'
script=prior.read_text().replace("src=p/'frame-and-feeder.step'","src=p/'forward-drive.step'").replace("root/'split-cycle-01/package.json'","p/'package.json'")
exec(compile(script,'forward motor clearance','exec'))
