# The two screws enter the rigid stop backing from behind. The pad has its own
# uninterrupted NOTE-facing surface. The bonded facing is a service item.
name='Rear piece stop'
backer=box(28,-48,19,6.2,96,45)
for y,side in [(-41,'right'),(41,'left')]:
    bracket='Rear stop bracket · '+side
    G[bracket]=cut(G[bracket],place(cyl((24,y,33),X,1.7,4),mat(park,*pivot)))
    backer=cut(backer,cyl((28,y,33),X,1.5,4))
    screw=fuse(cyl((21,y,33),X,2.9,3),cyl((24,y,33),X,1.5,8))
    label='Rear stop screw · '+side
    local(label,screw,'Piece support','mechanism','M3 steel screw envelope')
    specify(label,'Clamp the removable stop backing to its bracket.','The screw enters from behind. The NOTE sees an uninterrupted padded face; thread engagement is a preliminary allocation.')
G[name]=place(backer,mat(park,*pivot));meta[name]['material']='6061 aluminum stop backing'
specify(name,'Take the chambered NOTE load through a padded, replaceable stop.','Two rear-driven screws retain this backing against the bracket shoulders. The brackets land on the rear runner crossbar.')
local('Rear stop pad',box(34.2,-48,19,3,96,45),'Piece support','mechanism','Bonded urethane')
specify('Rear stop pad','Cushion the NOTE on arrival at the stop.','A replaceable 3 mm face covers the backing and separates the foam NOTE from metal hardware.')

