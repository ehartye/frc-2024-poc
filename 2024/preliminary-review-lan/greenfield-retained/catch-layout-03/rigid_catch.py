"""Cross-check the angular spring with a unilateral constraint on the CAD tangent face."""
from pathlib import Path
p=Path(__file__).resolve().parent
def stiffen(src):
    replacements={
        'axis="0 -1 0" damping=".5"':'axis="0 -1 0" damping=".5" limited="true" range="-360 360" solreflimit=".001 1" solimplimit=".9999 .9999 .00001"',
        'request=None;depth=0.':'overloaded=False;maxCompression=0.;request=None;depth=0.',
        'reaction=float(np.clip(12000.*sink-18*d.qvel[vi],0,cap)) if armed and sink>0 else 0.':'''m.jnt_range[jid.id,0]=np.deg2rad(158 if armed and not overloaded else -360)
    ids=np.flatnonzero((d.efc_type==int(mj.mjtConstraint.mjCNSTR_LIMIT_JOINT))&(d.efc_id==jid.id))
    reaction=float(max([0.,*d.efc_force[ids]])) if armed and not overloaded else 0.
    if armed and not overloaded:maxCompression=max(maxCompression,sink*55.)
    if reaction>cap:overloaded=True;m.jnt_range[jid.id,0]=-6.28''',
        'd.qfrc_applied[vi]=torque+reaction':'d.qfrc_applied[vi]=torque',
        'r.update(commandTime=request':'r.update(overloadVeto=overloaded,maxLimitCompressionMm=maxCompression,commandTime=request',
        'preTorque>=60 and maxFloor':'preTorque>=60 and not overloaded and maxFloor'}
    for a,b in replacements.items():
        assert src.count(a)==1,a
        src=src.replace(a,b)
    return src
a=(p/'sequence.py').read_text()
anchor="exec(compile(src,"+chr(39)*3+")";assert a.count(anchor)==1
a=a.replace(anchor,"src=stiffen(src)\n"+anchor)
exec(compile(a,'rigid dog cross-check','exec'))

