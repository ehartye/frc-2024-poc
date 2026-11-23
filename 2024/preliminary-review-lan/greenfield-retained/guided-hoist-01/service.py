"""Measure the installed change against the continuous disconnected battery extraction at both indexed poses."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
sourceText=(p/'inboard.py').read_text(encoding='utf8')
exec(compile(sourceText.split('partsAll={')[0],'inboard hoist solids','exec'))
sys.path.insert(0,str(p.parent/'follower-01'))
from stroke import scheduled
pack=bounds(original['Battery and terminal case'])
sweep=box(*pack[:3],pack[3]-pack[0],pack[4]-pack[1],pack[5]-pack[2]+370)
out={}
for angle in [80,125]:
    m=mat(angle,scheduled(angle),0,501);gaps={};hits={}
    for n,sh in parts.items():
        item=place(sh,m);gaps[n]=distance(item,sweep)
        if gaps[n]<.02:
            volume=common(item,sweep)
            if volume>.02:hits[n]=volume
    out[angle]=dict(minGapMm=min(gaps.values()),closest=min(gaps,key=gaps.get),hits=hits)
save(p/'inboard-service.json',out);print(out)
