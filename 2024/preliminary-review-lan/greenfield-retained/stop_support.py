# Each strap seats against the underside of the stop and laps the underside of
# the existing rear crossbar. Moving the crossbar ends over the beam tops
# gives finite bearing area at the chassis side of this load path as well.
for st in (125,370,599):
    name=f'Runner crossbar {st}'
    wide=box(590 if st==599 else st,-226,14,10,452,5)
    if st==125:wide=cut(wide,cyl((130,55,13),Z,3.6,12))
    G[name]=place(wide,mat(park,*pivot))
for y,side in [(-47,'right'),(35,'left')]:
    # 5 mm return at the stop forms an opposed vertical shoulder face.
    shape=prism([(24,y,9),(135,y,9),(135,y,14),(125,y,14),(125,y,19),(28,y,19),(28,y,46),(24,y,46)],(0,12,0))
    name='Rear stop bracket · '+side
    local(name,shape,'Piece support','structure','6061 aluminum bracket')
    specify(name,'Transfer NOTE impact from the rear stop into the runner crossbar.','The bent-up shoulder takes the rearward load. The lower return laps under the crossbar; the crossbar overlaps both cassette beams.')

