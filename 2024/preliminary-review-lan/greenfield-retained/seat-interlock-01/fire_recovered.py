"""Release only the continuous carried state with the actual cam within one degree of open."""
from pathlib import Path
import json,math,sys
trial=Path(__file__).resolve().parent
source=trial/(sys.argv[1]+'-'+sys.argv[2]+'.json')
carry=json.loads(source.read_bytes())
assert carry['held'] and carry['seated'] and carry['beamProofS'] is not None and not carry['failedGate']
assert abs(math.degrees(carry['endFlapRad'])-90)<=1
exec(compile((trial.parent/'split-field-02/fire.py').read_text(),'fire confirmed physical capture','exec'))
