#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from sensor_msgs.msg import LaserScan
import math as M
import tf
from tf import transformations

class Move_It():
	
	def __init__(self):
		print("Entering the init function yay\n")
		self.shouldMove = False
		rospy.init_node('movement')
		self.waypoint = None
		self.motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
		self.waypoint_subscriber = None
		self.laserscan_subscriber = None
		self.listener = tf.TransformListener()
		self.destin_list = []
		self.destin_list2= []

	#callback for waypoints, if ever needed.
	def waypoint_callback(msg):
		print("w_p call back\n")
		self.waypoint = msg
	
	def laserscan_callback(data):
		print("ls call back\n")
		self.data = data
	
	#For subscribing to the topics of interest.
	def activate(self):
		print("Entering the activate function shouldMove is now true\n")
		self.shouldMove = True
		#self.waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, self.waypoint_callback)
		#self.laserscan_subscriber = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)
		if (len(self.destin_list) > 0 ):
			self.moveItTo(self.destin_list.pop())
	#For unregistering from subscribed topics, Therefore cancelling out any and all movement.
	def deacvtivate(self):
		self.shouldMove = False
		self.waypoint_subscriber.unregister()
		self.laserscan_subscriber.unregister()
	
	def moveItTo(self, destin):
		print("Entering moveItTo, ", self.destin_list )
		if self.shouldMove == False:
			self.destin_list.insert(0, destin)
			return
		
		delay = rospy.Rate(1.0);
		
			
		if len(self.destin_list) > 0:
			self.destin_list.insert(0, destin)
			destin = self.destin_list.pop()
		
		while not rospy.is_shutdown():
			
			try:
				(translation,orientation) = self.listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
			except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				print("EXCEPTION:",e)
				delay.sleep()
			
			print("Robot is believed to be at (x,y,z): (",translation[0],",",translation[1],",", translation[2], ")")
			r_x, r_y, r_z = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
			print("Robot's angle with the Z-axis is: ", r_z)
			print("Robot's angle with the Y-axis is: ", r_y)
			print("Robot's angle with the X-axis is: ", r_x)
			
			print("Destination is believed to be at (x,y,z): (",destin[0],",",destin[1],",", destin[2], ")")
			
			motor_command=Twist()
			
			x = destin[0] - translation[0]
			y = destin[1] - translation[1]
			z = destin[2] - translation[2]
			theta1 = M.atan2(y,x)
			theta2 = M.atan2(y,x)
			newSpeed = sqrt(x**2 + y**2 + z**2)
			
			if abs(theta1) < 0.1 and abs(theta2) < 0.1:	#The robot is not facing the exact destination, but it's close enough. so let's move.
				motor_command.linear.x= newSpeed/2
			motor_command.angular.z = theta1
			motor_command.angular.y = theta2

			motor_command_publisher.publish(motor_command)
	
	def moveItTo2(self, destin):
		if self.shouldMove == False:
			self.destin_list2.insert(0, destin)
			return
			
		delay = rospy.Rate(1.0)
		
		if len(self.destin_list2) > 0:
			self.destin_list2.insert(0, destin)
			destin = self.destin_list2.pop()
		while not rospy.is_shutdown():
			try:
				(translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
			except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				print("EXCEPTION:",e)
				delay.sleep()
			print("Robot is believed to be at (x,y,z): (",translation[0],",",translation[1],",", translation[2], ")")
			r_x, r_y, r_z = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
			print("Robot's angle with the Z-axis is: ", r_z)
			print("Robot's angle with the Y-axis is: ", r_y)
			print("Robot's angle with the X-axis is: ", r_x)
			
			print("Destination is believed to be at (x,y,z): (",destin.translation.x,",",destin.translation.y,",", destin.translation.z, ")")
			destinrotq = [destin.rotation.x,destin.rotation.y,destin.rotation.z,destin.rotation.w]
			d_x, d_y, d_z = transformations.euler_from_matrix(transformations.quaternion_matrix(destinrotq))
			print("Destination's angle with the Z-axis is: ", d_z)
			print("Destination's angle with the Y-axis is: ", d_y)
			print("Destination's angle with the X-axis is: ", d_x)
			
			motor_command=Twist()
			
			x = destin.translation.x - translation[0]
			y = destin.translation.y - translation[1]
			z = destin.translation.z - translation[2]
			theta1 = M.atan2(y,x)
			theta2 = M.atan2(y,x)
			newSpeed = sqrt(x**2 + y**2 + z**2)
			
			if abs(theta1) < 0.1 and abs(theta2) < 0.1:	#The robot is not facing the exact destination, but it's close enough. so let's move.
				motor_command.linear.x= newSpeed/2
			motor_command.angular.z = theta1
			motor_command.angular.y = theta2

			motor_command_publisher.publish(motor_command)
	
