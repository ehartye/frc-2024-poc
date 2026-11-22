"""Continue the matched incoming free-body and three rotor states into the registered field shot."""
from pathlib import Path
import sys
here=Path(__file__).resolve().parent
reference=here.parent/'scorer-contact-01'
sys.path.insert(0,str(reference))
script=(reference/'split.py').read_text()
start=script.index('src=root/')
end=script.index(';incoming=',start)
script=script[:start]+"src=p/f'{key}-{n}.json'"+script[end:]
needle="env.update(params=coefficients()"
assert script.count(needle)==1
script=script.replace(needle,'''sub('belts=np.zeros(2)',"belts=np.array(incoming['endZoneSpeeds'][1:])")
sub('feedSpeed=0.;feeding=',"feedSpeed=incoming['endZoneSpeeds'][0];feeding=")
'''+needle)
exec(compile(script,'fire from carried state','exec'))
