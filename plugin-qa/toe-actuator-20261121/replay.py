from pathlib import Path
import sys
import mujoco
q=Path(__file__).parent; geom=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/over-bumper-pickup'
head=geom/'head_clear_trial.py'
n=int(sys.argv[1]);sys.argv=[str(head),str(n),'enabled',str(q/f'replay-{n}')]
text=head.read_text()
# Preserve the canonical head-clear interlock exactly; instrument immediately before the forward integration.
at="exec(compile(code,str(base),'exec'),"
before="""
code=code.replace('for i in range(round(15/dt)):',"toeTelem=[]\\nfor i in range(round(15/dt)):")
code=code.replace("    mujoco.mj_forward(m,d)","    if rel>=-.1 and rel<=4.75: toeTelem.append([t,rel,*[float(v) for j in fingers for v in [d.qpos[pose(j)],d.qvel[dof(j)],d.qfrc_applied[dof(j)]]]])\\n    mujoco.mj_forward(m,d)")
code=code.replace("save(out/'run.json',manifest)","np.save(out/'toe.npy', np.asarray(toeTelem))\\nsave(out/'run.json',manifest)")
"""
assert text.count(at)==1;text=text.replace(at,before+at)
print('runtime',mujoco.__version__,flush=True);exec(compile(text,str(head),'exec'),dict(__name__='__main__',__file__=str(head)))

