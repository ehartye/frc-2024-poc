from pathlib import Path
base=Path(__file__).resolve().parent.parent/'split-cycle-01'
src=(base/'split_fire.py').read_text()
src=src.replace("exec(compile(script,'fire","script=script.replace(\"'125','1.53'\",\"'125','1.54'\")\nexec(compile(script,'fire")
exec(compile(src,'revised station field shot','exec'))
