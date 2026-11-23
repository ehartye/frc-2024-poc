"""Test a parallel firing lane against the original failed field approach on the same carried NOTE."""
from pathlib import Path
p=Path(__file__).resolve().parent
raw=(p/'shot.py').read_text()
exec(compile(raw[:raw.index("src=(root/'split-cycle-01")],'same deployed field and captured NOTE','exec'))
sys.argv[3]='outerlane'
build=(root/'split-cycle-01/split_fire.py').read_text().split("exec(compile(script,'fire from carried state','exec'))")[0]
exec(compile(build,'construct powered launcher','exec'))
assert script.count("'125','1.53'")==1
script=script.replace("'125','1.53'","'122','1.39'")
build=script.split("exec(compile(base,'physical split drive trial','exec'))")[0]
exec(compile(build,'construct launcher and power arbitration','exec'))
mark="exec(compile(old,'two drive zones','exec'))"
assert base.count(mark)==1
base=base.replace(mark,"assert old.count("+repr("'5.55'")+")==1;old=old.replace("+repr("'5.55'")+","+repr("'5.871'")+");"+mark)
exec(compile(base,'constant lateral offset trial','exec'))

