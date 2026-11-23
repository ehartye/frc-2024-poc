from pathlib import Path
import json,hashlib
import numpy as np
from component_intent import explain
def add_review(data,seat,held):
    here=Path(__file__).resolve().parent
    body=data['bodies']
    for name,b in body.items(): b.update(explain(name))
    source=Path('R:/kickoff/strategic-narrative.md')
    text=source.read_text(encoding='utf8')
    assert 'End-of-match decision' in text
    data['narrativeReview']=dict(source=dict(file='kickoff/strategic-narrative.md',sha256=hashlib.sha256(source.read_bytes()).hexdigest()),objectives=[
        dict(objective='Acquire loose NOTES and repeat close Speaker deliveries',disposition='open',basis='P2.2 tests floor approach and cassette poses. Powered capture, release and repeated scored cycles still need evidence.'),
        dict(objective='Finish the next attainable delivery or PARK',disposition='open',basis='Compare travel and reserve with 24 seconds left and verify the terminal bumper position.'),
        dict(objective='Solo ONSTAGE or the partner-dependent shared-chain branch',disposition='open',basis='No chain interface, load-bearing hold, climb allocation or matched endgame route is represented.'),
        dict(objective='Optional overhead entry',disposition='open',basis='The narrative calls for an entry pose and retention evidence before crediting this route.')])
    result=json.loads((here/'gravity-hold.json').read_bytes());assert data['step']==result['sourceStep']
    matrices=[]
    for i in range(25):
        m=np.eye(4);m[:3,3]=seat[:3,2]*result['strokeMm']*i/24
        matrices.append(m[:3].ravel().tolist())
    data['cases'].append(dict(id='Power-off escape at 125°',stage='retention control',basis=result['claim'],gap=0,status='fail',
        frames={held:matrices},positions=[[965.2,5547.36]]*25,removed=[],
        results=[dict(pair=[held,'Rear piece stop'],status='fail',witness=dict(t=1,reason='80 mm open path · 45.9 mm downhill'))]))
    data['status']='Held · capture and retention'
    data['focus']='At 125° the NOTE can escape 80 mm through the open face. Select Power-off escape at 125°. Capture and endgame also remain open.'
    data['notes'].append('Retention control: a 0.25 kg NOTE at 125° has 1.41 N of outward gravitational load with no friction or belt preload credited. The exact 80 mm swept ring clears all 91 neighbors, ending 45.89 mm lower. A trial keeper coupon with 4.60 mm initial clearance obstructs the sweep by 2400 mm³. The coupon is a test control. A functional keeper needs an attachment, release path and load qualification. At the initial −50° pose gravity also pulls 1.88 N along the channel toward the mouth. That requires a gate or demonstrated grip. A top keeper alone would address only the inverted part of the lift.')

