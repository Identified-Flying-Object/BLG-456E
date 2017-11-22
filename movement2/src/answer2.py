#!/usr/bin/env python
#Piece of garbage code. Just used for testing some stuff.
#Ignoreable
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from sensor_msgs.msg import LaserScan
import math as M
import tf
from tf import transformations

def laserscan_callback(data):
	print data
	
	print 'Number of points in laser scan is: ', len(data.ranges)
	print 'The distance to the rightmost scanned point is: ', data.ranges[0]
	print 'The distance to the leftmost scanned point is: ', data.ranges[-1]
	print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
	## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
	print 'The minimum angle scanned by the laser is: ', data.angle_min
	print 'The maximum angle scanned by the laser is: ', data.angle_max
	print 'The increment in the angles scanned by the laser is: ', data.angle_increment
	## angle_max = angle_min+angle_increment*len(data.ranges)
	print 'The minimum range (distance) the laser can perceive is: ', data.range_min
	print 'The maximum range (distance) the laser can perceive is: ', data.range_max
    
if __name__ == '__main__':
	rospy.init_node('heelo')
	
	global mcp
	rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size =100)
	mcp = rospy.Publisher('/cmd_vel', Twist, queue_size = 100)
	m = Twist()
	m.linear.z = 1
	while not rospy.is_shutdown():
		mcp.publish(m)
