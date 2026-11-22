from pathlib import Path
import sys
p=Path(__file__).resolve().parent
if sys.argv[1]=='field':
    src=(p/'field-check.py').read_text(encoding='utf8').replace("p/'stop-slot.py'","p/'inboard.py'").replace("src.split('partsAll=')[0]","src.split('partsAll={')[0]").replace("'winch-'","'return-'").replace("p/'field-check.json'","p/'return-field.json'")
else:
    src=(p/'inboard-budget.py').read_text(encoding='utf8').replace("'winch-'","'return-'").replace("p/'inboard-budget.json'","p/'return-budget.json'").replace("allowances={","allowances={'Paired take-up springs, arbor mounts and retainers (provisional)':[.05,.20],")
exec(compile(src,'take-up '+sys.argv[1],'exec'))
