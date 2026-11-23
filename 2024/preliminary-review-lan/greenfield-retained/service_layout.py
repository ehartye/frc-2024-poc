# The same slide truck carries a retractable service index pin and a rotating disk.
# The indexed face comes into line with the truck at exactly 80 degrees.
service_angle=80
shole=mat(service_angle,*pivot)@np.array([-22,0,-4,1])
pinaxis=(shole[0],247,shole[2])
service_bore=cyl((shole[0],246,shole[2]),Y,3.35,20)
truck='Translating bearing block · left'
G[truck]=cut(G[truck],service_bore)
disk=cut(ann((0,226,0),Y,28,9,11),cyl((-22,226,-4),Y,3.35,11))
local('Service index disk',disk,'Pivot support','mechanism','Steel index plate')
specify('Service index disk','Lock the cassette angle for battery service.','The disk transfers gravity torque through a dedicated pin to the translating bearing block.')
add('Service pin guide',ann((shole[0],261,shole[2]),Y,6,3.35,8),'Pivot support','mechanism','Steel')
add('Battery service pin',cyl((shole[0],239,shole[2]),Y,3.175,43),'Pivot support','mechanism','Steel manual lock pin')
slide.extend(['Service pin guide','Battery service pin'])
specify('Service pin guide','Support the locking pin outside the truck.','Provides guidance through the aligned bore and clearance outside the pivot shaft.')
specify('Battery service pin','Manually pin the rotating disk before battery removal.','Retracts 12 mm for motion. The selected service position is distinct from the obstructed launch angle.')

