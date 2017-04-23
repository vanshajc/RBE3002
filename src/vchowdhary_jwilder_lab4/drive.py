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
	global pub_way

	waypts = gridcells.cells
	print 'Driving!', waypts
	#rotate(180)	
	#driveTo(1)	
	#for p in waypts:
	if (len(waypts) == 0):
		return
	navToPose(waypts[0])
	pub_way.publish(waypts[0])
	print 'Waypoint reached'

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	global pose
	global oc
		
	print 'Currently at', pose.pose.position.x, pose.pose.position.y
	print 'Going to', goal.x, goal.y
	rospy.sleep(0.1)	
	diffX = goal.x - pose.pose.position.x
	diffY = goal.y - pose.pose.position.y
	print "Distance", diffX, diffY
	rotate((math.degrees(math.atan2(diffY, diffX) - pose.pose.orientation.z))%360)

	driveTo(math.sqrt(diffX**2 + diffY**2))

	print "Finished"

def toGlobal(angle):
	if (angle <= 180 and angle >= 0):
		return angle
	return (360 + angle)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global pose
	global pub_way
	
	done = False
	dest = ((angle) + toGlobal(math.degrees(pose.pose.orientation.z)))%360

	while (not done and not rospy.is_shutdown()):
		#print 'Rotating'
		#print "Current Angle", toGlobal(math.degrees(pose.pose.orientation.z))
		error = (dest - toGlobal(math.degrees(pose.pose.orientation.z)))%360
		e2 = (360 + toGlobal(math.degrees(pose.pose.orientation.z)) - dest)%360
		if (abs(e2) < abs(error)):
			error = -1*e2
		if (abs(error) < 1):
			done = True
			break
		publishTwist(0, (1.5 * error/90) + 0.3*(error/abs(error)))
		rospy.sleep(0.01)

	rospy.sleep(1)
	#pub_way.publish(Point())
	rospy.sleep(0.5)
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
		if (abs(cd - distance) < 0.1):
			reached = True
			publishTwist(0, 0)
		else:
			publishTwist((distance - cd)*0.1/distance + 0.1, 0)
			rospy.sleep(0.01)

def publishTwist(linV, angV):
	global pub
	msg = Twist()
	msg.linear.x = linV
	msg.angular.z = angV
	pub.publish(msg)
	
def timerCallback(event):
	global pose
	#print "timercallback"
	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	pose.pose.position.x = position[0]
	pose.pose.position.y = position[1]

	odomW = orientation

	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	
	pose.pose.orientation.z = yaw


def updateMap(oc_grid):
	global oc
	global updatedMap

	print 'Retreived'
	oc = oc_grid
	updatedMap = True

if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('vchowdhary_jwilder_Drive')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global pub
	global pose
	global odom_tf
	global odom_list
	global sub_waypoints
	global map_sub
	global oc
	global pub_way

	updatedMap = False
   
	pose = PoseStamped()

	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) 
# Publisher for commanding robot motion
	map_sub = rospy.Subscriber('/map', OccupancyGrid, updateMap, queue_size=1)
	sub_waypoints = rospy.Subscriber('/Waypoints', GridCells, drivePath, queue_size=10)

	pub_way = rospy.Publisher('/WayReached', Point, queue_size=1)
    # Use this object to get the robot's Odometry 
	#make the robot keep doing something...
	rospy.Timer(rospy.Duration(0.01), timerCallback)

	odom_list = tf.TransformListener()
    # Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(2, 0))
	print 'Started'
	#while(1):
	rospy.sleep(100000)
	
