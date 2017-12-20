#!/usr/bin/env python
#Robot Movement. Object Oriented.
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import math as M
import tf
from tf import transformations

class Move_It():
	
	def __init__(self):
		self.shouldMove = False
		rospy.init_node('movement')
		self.waypoint = None
		self.motor_command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.waypoint_subscriber = None
		self.laserscan_subscriber = None
		self.sonarhieght_subscriber = None
		self.listener = tf.TransformListener()
		self.destin_list = []
		self.destin_list2= []
		self.motor_command = Twist()
		self.speed = 0
		self.angle = 0
		self.altitude = 1
		self.flightTime = 0
		self.rotationTime = 0
		self.clk = rospy.get_time();
		print self.clk

	#callback for waypoints, if ever needed.
	def waypoint_callback(self, msg):
		print("w_p call back\n")
		self.waypoint = msg
	
	#Laserscan callback used in order to avoid crashing. Detect the objects in its path and then halts the robot if found.
	def laserscan_callback(self, data):
		for i in range(len(data.ranges), 10):
			if data.ranges[i] < 1.5:
				angle = 135 - i*data.angle_increment*180/M.pi
				print "Detected close object at: ", angle, "Distance: ", data.ranges[i]
				if angle > self.angle - 15 and angle < self.angle + 15:
					print "we are at a crashing course, stopping machine"
					self.setLinear(0, 0, 0)
		#print "This is the x, y, z speed: ", self.motor_command.linear.x, self.motor_command.linear.y, self.motor_command.linear.z
	
	#Sonar_height callback. Subscribed to the '/sonar_height' topic, takes in Range. Used for maintaining altitudes.
	def sonar_callback(self, data):
		if data.range <= self.altitude:
			#print "too Low goin higher: ", data.range
			self.motor_command.linear.z = 1
		elif data.range > self.altitude:
			#print "Too high goin lower: ", data.range
			self.motor_command.linear.z = -data.range/3
		
		if not self.shouldMove:
			self.motor_command_publisher.publish(self.motor_command)
	
	#For subscribing to the topics of interest and running the code.
	def activate(self):
		print("Entering the activate function shouldMove is now true\n")
		print rospy.get_time() , "/", self.clk + self.flightTime
		self.shouldMove = True
		#self.waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, self.waypoint_callback)
		self.laserscan_subscriber = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)
		self.sonarheight_subscirber = rospy.Subscriber("/sonar_height", Range, self.sonar_callback)
		activationTime = rospy.get_time()
		if (len(self.destin_list) > 0 ):
			self.moveItTo(self.destin_list.pop())
		else:
			while self.shouldMove and not rospy.is_shutdown() and rospy.get_time() < self.flightTime + activationTime:
				print "in loop, time: ", rospy.get_time(), ". Finishing time: ", self.flightTime + activationTime				
				self.motor_command_publisher.publish(self.motor_command)
		self.shouldMove = False
		self.motor_command = Twist()
		print "Successfully exited activate"
	
	#For unregistering from subscribed topics, Therefore cancelling out any and all movement.
	def deactivate(self):
		self.motor_command = Twist()
		self.shouldMove = False
	
	#Moves the robot to a specified point. May be buggy. Will be reviewed once SLAM is completed.
	#destin must be an array of size 3 in the order: [x, y, z]
	def moveItTo(self, destin):
		print "Entering moveItTo, ", self.destin_list 
		if self.shouldMove == False:
			self.destin_list.insert(0, destin)
			print "Inserted :", destin
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
				return
				delay.sleep()
			
			print "Robot is believed to be at (x,y,z): (",translation[0],",",translation[1],",", translation[2], ")"
			r_x, r_y, r_z = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
			print "Robot's angle with the Z-axis is: ", r_z
			print "Robot's angle with the Y-axis is: ", r_y
			print "Robot's angle with the X-axis is: ", r_x
			
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
	
	#Same as moveitTo, except takes in transformations. May be buggy. Will be reviewed once Pathfinding is completed.
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
	
	#Sets the linear velocity and the flight time. Pretty self-explanatory
	def setLinear(self, x=0, y=0, z=1, flightTime = 3):
		self.motor_command.linear.x = x
		self.motor_command.linear.y = y
		self.motor_command.linear.z = z
		self.speed = M.sqrt(x**2 + y**2)
		self.angle = M.atan2(y, x)
		self.flightTime = flightTime
		
	#Do I even need to write what this ine does?
	def setAngular(self, Ox=0, Oy=0, Oz=0, rotationTime = 1):
		self.motor_command.angular.x = Ox
		self.motor_command.angular.y = Oy
		self.motor_command.angular.z = Oz
		self.rotationTime = rotationTime
	def setAltitude(self, x = 1):
		self.altitude = x
