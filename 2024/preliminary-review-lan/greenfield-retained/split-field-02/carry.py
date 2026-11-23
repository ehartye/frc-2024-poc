from pathlib import Path
base=Path(__file__).resolve().parent.parent/'split-cycle-01'
src=(base/'split_intake.py').read_text().replace('"('+"sources/"+ "'plan.json')"+'"','"(folder/\'plan.json\')"')
assert 'sources/'+"'plan.json'" not in src
exec(compile(src,'revised route full intake and carry','exec'))
