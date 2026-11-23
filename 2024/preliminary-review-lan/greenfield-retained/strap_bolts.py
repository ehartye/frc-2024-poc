cross='Runner crossbar 125'
for y,side in [(-41,'right'),(41,'left')]:
    bracket='Rear stop bracket · '+side
    through=place(cyl((130,y,6),Z,1.7,15),mat(park,*pivot))
    for name in (cross,bracket):G[name]=cut(G[name],through)
    screw=fuse(cyl((130,y,19),Z,2.7,2),cyl((130,y,5),Z,1.5,14))
    nm='Stop bracket through bolt · '+side
    local(nm,screw,'Piece support','mechanism','M3 steel through bolt')
    specify(nm,'Fasten the return of the stop bracket to the runner crossbar.','A through joint gives the removable stop a positive connection under load in either direction.')
    nm='Stop bracket locknut · '+side
    local(nm,ann((130,y,6),Z,3.1,1.5,3),'Piece support','mechanism','M3 steel locknut envelope')
    specify(nm,'Retain the crossbar through bolt under the stop bracket.','The nut bears on the underside of the bracket, leaving the runner lanes free.')

