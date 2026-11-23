import json,pathlib,sys,subprocess
p=pathlib.Path(__file__).parent
template=(p/'fusion-native-outer-trim-plus15-24-0.py').read_text()
template=template[:template.rindex('    check(')]
extra="""    mgr=adsk.fusion.TemporaryBRepManager.get()
    C=adsk.core
    P=C.Point3D.create
    report=[]
    for o in doc.products.itemByProductType('DesignProductType').rootComponent.allOccurrences:
        for body in o.bRepBodies:
            if body.name not in ('intake candidate side pinch belt -1','intake candidate side pinch belt 1'):
                continue
            rec=next(x for x in scan['inventory'] if x['name']==body.name)
            b=rec['boundsMm']
            cx,cy,cz=[(b[i]+b[i+3])/20 for i in range(3)]
            radius=(12*6.35/(2*math.pi)+1)/10
            half_length=((64-12)*6.35/2)/20
            width=1.27
            box=C.OrientedBoundingBox3D.create(P(cx,cy,cz),C.Vector3D.create(1,0,0),C.Vector3D.create(0,1,0),2*half_length,2*radius,width)
            sleeve=mgr.createBox(box)
            for x in (cx-half_length,cx+half_length):
                cap=mgr.createCylinderOrCone(P(x,cy,cz-width/2),radius,P(x,cy,cz+width/2),radius)
                assert mgr.booleanOperation(sleeve,cap,adsk.fusion.BooleanTypes.UnionBooleanType)
            outside=mgr.copy(body)
            assert mgr.booleanOperation(outside,sleeve,adsk.fusion.BooleanTypes.DifferenceBooleanType)
            residue=outside.volume*1000 if outside.isValid and outside.faces.count else 0.
            report.append(dict(key=rec['key'],outsideMm3=residue))
    assert len(report)==2
    Path(OUTPUT).write_text(json.dumps(report))
"""
output=p/'native-belt-envelope-verification.json'
script=p/'fusion-check-belt-envelope.py'
script.write_text(template+extra.replace("OUTPUT",repr(str(output))))
probe=p.parent/'native-loading-gate-20261118/fusion_probe.py'
r=subprocess.run([sys.executable,'-B',str(probe),str(script)],capture_output=True)
if not output.exists():raise RuntimeError(r.stderr.decode()[-650:]+r.stdout.decode()[-1000:])
print(output.read_text())
