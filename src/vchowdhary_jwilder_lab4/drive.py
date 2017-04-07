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

def drivePath(gridcells):
	waypts = gridcells.cells
	for p in waypts:
		navToPose(p)

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	print 'Starting'
	rospy.sleep(0.1)	
	diffX = goal.x - pose.pose.position.x
	diffY = goal.y - pose.pose.position.y
	print goal.x
	print goal.y
	print pose.pose.position.x
	print pose.pose.position.y
	print "Distance", diffX, diffY

	print "Current:", pose.pose.position.x, pose.pose.position.y


	rotate((math.degrees(math.atan2(diffY, diffX) - pose.pose.orientation.z))%360)

	driveTo(math.sqrt(diffX**2 + diffY**2))

	print "Finished"

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



def publishTwist(linV, angV):
	global pub
	msg = Twist()
	msg.linear.x = linV
	msg.angular.z = angV
	pub.publish(msg)
	
def timerCallback(event):
	global pose
	#print "timercallback"
	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	pose.pose.position.x = position[0]
	pose.pose.position.y = position[1]

	odomW = orientation

	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	
	pose.pose.orientation.z = yaw


if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('vchowdhary_jwilder_lab4')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global pub
	global pose
	global odom_tf
	global odom_list
	global sub_waypoints

	updatedMap = False
   
	pose = PoseStamped()

	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) 
# Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
	sub_waypoints = rospy.Subscriber('/Waypoints', GridCells, drivePath, queue_size=10)

	nav_sub = rospy.Subscriber('/vanisamazing', PoseStamped, navToPose)
    # Use this object to get the robot's Odometry 
	#make the robot keep doing something...
	rospy.Timer(rospy.Duration(0.01), timerCallback)

	odom_list = tf.TransformListener()
    # Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(2, 0))
	
