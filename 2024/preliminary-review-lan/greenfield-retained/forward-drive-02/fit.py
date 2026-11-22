"""Machine a single entry bearing bracket with the upper motor bridge on each side."""
from pathlib import Path
p=Path(__file__).resolve().parent
src=(p.parent/'forward-drive-01/fit.py').read_text(encoding='utf8')
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepCheck import BRepCheck_Analyzer
old="key='Forward launcher motor bridge · '+side;setstock(key,mount)"
new="""key='Entry bearing bracket · '+side
    oldEntry=place(G[key],ip)
    mount=fuse(oldEntry,mount)
    solids=TopExp_Explorer(mount,TopAbs_SOLID);assert solids.More();solids.Next()
    assert not solids.More() and BRepCheck_Analyzer(mount).IsValid()
    setstock(key,mount)"""
assert src.count(old)==1
exec(compile(src.replace(old,new),'integral nose bearing and motor bracket','exec'))
