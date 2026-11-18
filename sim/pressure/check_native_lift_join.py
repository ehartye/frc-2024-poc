"""Fail closed before substituting the historical lift model for native pressure."""
import argparse,hashlib,json,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for n in ['project','binding','trajectory','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();binding=json.loads(a.binding.read_bytes());trajectory=json.loads(a.trajectory.read_bytes());source=a.project/'sim/integrated/coupled_lift_phase.py';text=source.read_text()
assert 'self.inheritedSpring=config[\'inheritedNormalSpringJ\']' in text
assert "self.patch=copy.deepcopy(patch['cradlePoints'])" in text
assert binding['corridor']['fixedFloorCenterZM']==.0254
state=trajectory['final'];report=dict(schema='native-pressure-lift-join/1',accepted=False,hardwareValidated=False,sourceTrajectorySha256=sha(a.trajectory),nativeBindingSha256=sha(a.binding),historicalLiftSourceSha256=sha(source),attainedGroundIndex=trajectory['index'] is not None,sourceTimeS=state['timeS'],sourceStateSha256=hashlib.sha256(json.dumps(state,sort_keys=True,separators=(',',':')).encode()).hexdigest(),physicalStateMutated=False,blockingReasons=[
 'Current native binding covers a sampled flat-floor corridor; lifted and pitched hard-part clearance is unbound.',
 'Current pressure state constrains NOTE Z/roll/pitch. Releasing these constraints requires explicit new degrees of freedom, floor support, inertia and energy accounting.',
 'Historical lift class imports fixed patch witnesses and inheritedNormalSpringJ; this is not the native finite-area pressure state or constitutive law.',
 'Lift and pitch moving masses, native transmission transforms and collision geometry must be registered before a continuous physical handoff.',
 'Staged launcher remains a separate configuration; no acquired NOTE state may be replaced by its initial pose.'
 ],nextSupportedWork='Bind native moving-body mass/COM/inertia and lift/pitch transforms, then couple the same pressure field to free NOTE and finite actuators on one supply with continuous state and energy. Recheck dynamic support and clearance before feed.',limits=['Executable source/contract composition rejection, not a simulated lift or hardware failure','No Fusion call, audit replay, computer control or native mutation'])
a.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(report,indent=2))
