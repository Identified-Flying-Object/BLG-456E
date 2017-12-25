#!/usr/bin/env python

## Emre Yeniay
## 150110013

import roslib
import rospy
import random

from geometry_msgs.msg import Twist

def firstPrint():
	straight = 0
	while (1):
		print """
Wind simulation package for Aider Drone. Select the direction of the wind first.

1   : Random wind (Shake robot)
2   : Straight wind (Push robot one way)
E/e : Exit
		"""
		key = raw_input('Enter your command: ')
		if (key == '1'):
			print "Random direction is selected."
			straight = 0
			break
		elif (key == '2'):
			print "Straight direction is selected."
			straight = 1
			break
		elif (key == 'E') or (key == 'e'):
			print "Terminating wind simulation package."
			return
		else:
			print "Unknown selection. Please try again."

	while (1):		
		print """
Secondly, select the magnitunde of the wind.

1   : Light wind (Breeze)
2   : Medium wind (Gust)
3   : Strong wind (Storm)
E/e : Exit
		"""
		key = raw_input('Enter your command: ')
		if (key == '1'):
			print "Light force is selected. Applying wind right now. Press CTRL+C to quit."
			publish(straight, 0.02)
			return
		elif (key == '2'):
			print "Medium force is selected. Applying wind right now. Press CTRL+C to quit."
			publish(straight, 0.07)	
			return
		elif (key == '3'):
			print "Strong force is selected. Applying wind right now. Press CTRL+C to quit."
			publish(straight, 0.2)
			return
		elif (key == 'E') or (key == 'e'):
			print "Terminating wind simulation package."
			return
		else:
			print "Unknown selection. Please try again."

def publish(straight, magnitude):
	global orgX, orgY, orgZ, sentX, sentY
	delay = rospy.Rate(10)

	try:
		while(1):
			if (straight == 0):
				randomX = random.uniform(-1 * magnitude, magnitude)
				randomY = random.uniform(-1 * magnitude, magnitude)
			else:
				randomX = random.uniform(orgX, 2 * magnitude)
				randomY = random.uniform(orgY, 2 * magnitude)

			sentX = orgX + randomX
			sentY = orgY + randomY
			twist = Twist()
			twist.linear.x = sentX;
			twist.linear.y = sentY;
			twist.linear.z = orgZ;
			pub.publish(twist)
			delay.sleep()
	except KeyboardInterrupt:
		return
			
def callback(msg):
	global orgX, orgY, orgZ

	if (sentX == msg.linear.x) and (sentY == msg.linear.y):
		return

	orgX = msg.linear.x
	orgY = msg.linear.y
	orgZ = msg.linear.z

if __name__=="__main__":
	orgX = 0.0
	orgY = 0.0
	orgZ = 0.0

	sentX = 0.0
	sentY = 0.0

	rospy.init_node('wind_sim')
	rospy.Subscriber('/cmd_vel', Twist, callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

	firstPrint()

