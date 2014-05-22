from morse.builder import *

# First Human
lenka = Human()
#lenka.disable_keyboard_control()
lenka.use_world_camera()
lenka.translate(x=-1.0,y=-3.0)
lenka.rotate(z=1.6)
lpose = Pose()
lenka.append(lpose)
lpose.add_stream('ros')

# Second Human
ferdi = Human()
ferdi.disable_keyboard_control()
ferdi.use_world_camera()
ferdi.translate(x=-2.5,y=-0.3)
fpose = Pose()
ferdi.append(fpose)
fpose.add_stream('ros')
ferdi.add_service('ros')

#Third Human
marco = Human()
marco.disable_keyboard_control()
marco.use_world_camera()
marco.translate(x=1,y=-0.05)
marco.rotate(z=3.2)
mpose = Pose()
mmotion = MotionVW()
marco.append(mmotion)
marco.append(mpose)
mmotion.add_stream('ros')
mpose.add_stream('ros')

# Pioneer 3DX
pioneer = Pioneer3DX()
pioneer.translate(-1,2,0)
piomotion = MotionVWDiff()
pioneer.append(piomotion)
piomotion.add_interface('ros')

# Laser sensor
laser = Sick()
laser.properties(laser_range=0.7, scan_window=360.0, Visible_arc=False, resolution=10)
laser.translate(0.0,0.0,0.5)
#laser.frequency(1)
laser.rotate(z=3.2)
marco.append(laser)
laser.add_interface('ros')

cornflakes = PassiveObject("props/kitchen_objects", "Cornflakes")
cornflakes.setgraspable()
cornflakes.properties(Label = "Cornflakes")
cornflakes.translate(x=-0.3,y=-2.6,z=0.91)

plate = PassiveObject('props/kitchen_objects', 'Plate')
plate.setgraspable()
plate.properties(Label = "Plate")
plate.translate(x=0.4, y=-2.8 ,z=0.92)

fork = PassiveObject('props/kitchen_objects', 'Fork')
fork.setgraspable()
fork.properties(Label = "Fork")
fork.translate(x=0.6, y=-2.8, z=0.91)

knife = PassiveObject('props/kitchen_objects', 'Knife')
knife.setgraspable()
knife.properties(Label = "Knife")
knife.translate(x=0.2, y=-2.8, z=0.91)

bottle = PassiveObject('props/misc_objects', 'Bottle')
bottle.setgraspable()
bottle.properties(Label = "Bottle")
bottle.translate(x=-0.1, y=-3.0, z=1)

cup = PassiveObject('props/kitchen_objects', 'Cup_Ocher')
cup.setgraspable()
cup.properties(Label = "Cup")
cup.translate(x=1, y=-2.5, z=0.91)

env = Environment('apartment')
