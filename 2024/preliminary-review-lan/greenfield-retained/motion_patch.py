# Captured NOTE is the default pose. During admission the pin stays fully retracted.
oldposes=poses; oldgroup=group
release='Unlock and discharge'
draw='Power-off axial escape control'
normal='Power-off face escape control'
reach='Extend solo chain saddle'
closepin='Latch the seated NOTE'
def group(n):
    if n==pin:return 'plunger'
    if n=='Battery service pin':return 'index'
    if n in ['Climber slider · left','Climber slider · right','Climber crosshead','Chain saddle']:return 'elevator'
    return oldgroup(n)
def poses(case,t):
    if case in cases[:6]:
        ms=oldposes(case,t);angle=(park+(pick-park)*t if case==cases[1] else pick if case==cases[2] else pick+(park-pick)*t if case==cases[3] else service_angle if case==cases[5] else park)
    elif case==closepin: angle=pick;ms=oldposes(cases[3],0)
    else:angle=park;ms=oldposes(cases[0],0)
    stroke=66 if case in [cases[1],cases[2]] else 66*(1-t) if case==closepin else 66*min(1,t*4) if case==release else 0
    current=mat(angle,pivot_x(angle),0,pivot[2])
    ms['plunger']=current@mat(0,0,0,stroke)@inverse
    ms['index']=mat(0,pivot_x(angle)-185,-12 if case==cases[5] else 0,0)
    ms['elevator']=mat(0,0,0,142*t if case==reach else 0)
    if case==release:
        ms['piece']=current@mat(0,215+650*max(0,(t-.25)/.75),0,50.4)@invseat
    return ms

