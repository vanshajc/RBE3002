#!/usr/bin/env python
import rospy, tf
import math
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types used
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from astar import *

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	print 'Starting'
	rospy.sleep(0.1)	
	diffX = goal.pose.position.x - pose.pose.position.x
	diffY = goal.pose.position.y - pose.pose.position.y
	print goal.pose.position.x
	print goal.pose.position.y
	print pose.pose.position.x
	print pose.pose.position.y
	print "Distance", diffX, diffY
	print "Rotate by", math.degrees(math.atan2(diffX, diffY) + pose.pose.orientation.z)
	print "Current:", pose.pose.position.x, pose.pose.position.y


	rotate((math.degrees(math.atan2(diffY, diffX) - pose.pose.orientation.z))%360)


	rospy.sleep(0.1)
	driveTo(math.sqrt(diffX**2 + diffY**2))
	rospy.sleep(0.1)
	print "Rotating again", math.degrees(goal.pose.orientation.z - pose.pose.orientation.z)
	rotate((math.degrees(goal.pose.orientation.z - pose.pose.orientation.z)))
	print "Finished"

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():

	driveStraight(0.2, 0.6)
	rotate(-90)
	driveStraight(0.2, 0.45)
	rotate(135)




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	global pub
	
	lin_vel = (u1 + u2)*0.5
	ang_vel = (u1 + u2)/0.23

	now = rospy.Time.now().secs

	while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
		publishTwist(lin_vel, ang_vel)

	publishTwist(0, 0)

def driveTo(distance):
	global pose
	ix = pose.pose.position.x
	iy = pose.pose.position.y
	reached = False

	while (not reached and not rospy.is_shutdown()):
		cx = pose.pose.position.x - ix
		cy = pose.pose.position.y - iy
		cd = math.sqrt(cx **2 + cy **2)
		if (cd >= distance):
			reached = True
			publishTwist(0, 0)
		else:
			publishTwist((distance - cd)*5/distance, 0)
			rospy.sleep(0.10)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pose
	ix = pose.pose.position.x
	iy = pose.pose.position.y
	reached = False

	while (not reached and not rospy.is_shutdown()):
		cx = pose.pose.position.x - ix
		cy = pose.pose.position.y - iy
		cd = math.sqrt(cx **2 + cy **2)
		if (cd >= distance):
			reached = True
			publishTwist(0, 0)
		else:
			publishTwist(speed, 0)
			rospy.sleep(0.10)

def toGlobal(angle):
	if (angle <= 180 and angle >= 0):
		return angle
	return (360 + angle)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global pose
	
	done = False
	dest = ((angle) + toGlobal(math.degrees(pose.pose.orientation.z)))%360

	while (not done and not rospy.is_shutdown()):
		#print "Current Angle", toGlobal(math.degrees(pose.pose.orientation.z))
		error = (dest - toGlobal(math.degrees(pose.pose.orientation.z)))%360
		e2 = (360 + toGlobal(math.degrees(pose.pose.orientation.z)) - dest)%360
		if (abs(e2) < abs(error)):
			error = -1*e2
		if (abs(error) <= 2):
			done = True
		publishTwist(0, 2 * error/90)
		rospy.sleep(0.1)

	publishTwist(0, 0)



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):
	if (msg.state == 1):
		executeTrajectory()

def publishTwist(linV, angV):
	global pub
	msg = Twist()
	msg.linear.x = linV
	msg.angular.z = angV
	pub.publish(msg)
	

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	global pose
	#print "timercallback"
	#odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
	#(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	#pose.pose.position.x = position[0]
	#pose.pose.position.y = position[1]

	#odomW = orientation

	#q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	#roll, pitch, yaw = euler_from_quaternion(q)
	
	#pose.pose.orientation.z = yaw

def updateMap(oc_grid):
	global oc
	global updatedMap

	print 'Retreived'
	oc = oc_grid
	updatedMap = True

def c(n):
	r = []
	for i in range(len(n)):
		r.append(n[i]/100)
	return r

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('vchowdhary_jwilder_lab3')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global pub
	global pose
	global odom_tf
	global odom_list
	global pub_end
	global pub_path
	global pub_visited
	global pub_frontier
	global grid
	global oc
	global updatedMap

	updatedMap = False
   
	pose = PoseStamped()

	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) 
# Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

	map_sub = rospy.Subscriber('/map', OccupancyGrid, updateMap, queue_size=1)

	pub_end = rospy.Publisher('/EndPoints', GridCells, queue_size=1)
	pub_path = rospy.Publisher('/PathPoints', GridCells, queue_size=1)
	pub_visited = rospy.Publisher('/VisitedPoints', GridCells, queue_size=1)
	pub_frontier = rospy.Publisher('/FrontierPoints', GridCells, queue_size=1)

	nav_sub = rospy.Subscriber('/vanisamazing', PoseStamped, navToPose)
    # Use this object to get the robot's Odometry 
	#make the robot keep doing something...
	rospy.Timer(rospy.Duration(0.01), timerCallback)

	odom_list = tf.TransformListener()
    # Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(2, 0))
	
	while (not updatedMap):
		1 + 1

	p1 = Point()
	p1.x = 4
	p1.y = 15
	p2 = Point()
	p2.x = 35
	p2.y = 35
	
	print oc.info.width, oc.data[int(p1.x + p1.y*oc.info.width)]
	r = Astar(p1, p2, oc, pub_end, pub_path, pub_visited, pub_frontier)
	r.calculate()


	
