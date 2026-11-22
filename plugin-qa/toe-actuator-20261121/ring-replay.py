from pathlib import Path
import sys
q=Path(__file__).parent;c=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/over-bumper-pickup'
head=c/'head_clear_trial.py';n=int(sys.argv[1]);sys.argv=[str(head),str(n),'enabled',str(q/f'ring-{n}')]
text=head.read_text()
injection="""code=code.replace("for i in range(round(15/dt)):", "ringTelem=[]\\nfor i in range(round(15/dt)):")
code=code.replace("    if i%round(.025/dt)==0:", "    if i%round(.002/dt)==0: ringTelem.append([t, *xy, *[d.qpos[pose(j)] for j in ['reach','lift','pitch',*fingers]], *floorQ, *d.qpos[q0[names[0]]:q0[names[0]]+7]])\\n    if i%round(.025/dt)==0:")
code=code.replace("save(out/'run.json',manifest)", "np.save(out/'ring.npy',np.asarray(ringTelem))\\nsave(out/'run.json',manifest)")
"""
start="exec(compile(code,str(base),'exec'),";assert text.count(start)==1;text=text.replace(start,injection+start)
exec(compile(text,str(head),'exec'),dict(__file__=str(head),__name__='__main__'))

