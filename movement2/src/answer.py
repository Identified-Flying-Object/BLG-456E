#!/usr/bin/env python
#It is assumed that the main package will be here.
from i_like_to_move_it import Move_It
import rospy
#Code that moves the robot in funny ways.
x = Move_It()

i = 0



altitude = 1

while i != 'x':
	print "welcome to the answer, please follow these instructions to move the bot:"
	print "forward: 'w', backward: 's', turn right: 'd', turn left: 'a'"
	print "strafe right: 'e', strafe left: 'q', ascend: 'r', descend: 'f'"
	print "quit: x"
	i = raw_input('enter your command: ')
	if i == 'w':
		x.setLinear(1, 0, 0, 0.1)
	elif i == 's':
		x.setLinear(-1, 0, 0, 0.1)
	elif i == 'd':
		x.setAngular(0, 0, -1, 0.1)
	elif i == 'a':
		x.setAngular(0, 0, 1, 0.1)
	elif i == 'q':
		x.setLinear(0,1,0,0.1)
	elif i == 'e':
		x.setLinear(0, -1, 0, 0.1)
	elif i == 'r':
		altitude += 1
		x.setAltitude(altitude)
		x.setLinear(0,0,1,0.1)
	elif i == 'f':
		altitude += -1
		x.setAltitude(altitude)
		x.setLinear(0,0,-1,0.1)
	elif i == 'x':
		break
	x.activate()

rospy.spin()
