from pathlib import Path
p=Path(__file__).parent.resolve();src=(p/'sweep.py').read_text()
operation=src[src.index('traj=json'):src.index('cl=json')].replace("if gap<1e-5:shotHits.append([r['t'],n,common(piece,G[n])])","""if gap<1e-5:
            from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
            hit=BRepAlgoAPI_Common(piece,G[n]).Shape()
            print(r['t'],n,bounds(place(hit,np.linalg.inv(rest))),bounds(place(hit,pickpose@np.linalg.inv(rest))))""")
exec(src.split('def xbounds')[0])
exec(operation)


