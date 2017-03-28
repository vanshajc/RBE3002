#!/usr/bin/env python
import rospy, tf
import math
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types used
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	print "spin!"
	print "move!"
	print "spin!"
	print "done"
	pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	pass  # Delete this 'pass' once implemented




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	global pub
	
	lin_vel = (u1 + u2)*0.5
	ang_vel = (u1 + u2)/0.23

	now = rospy.Time.now().secs

	while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
		publishTwist(lin_vel, ang_vel)

	publishTwist(0, 0)


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
			rospy.sleep(0.15)

def toGlobal(angle):
	if (angle <= 180 and angle >= 0):
		return angle
	return (360 + angle)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global odom_list
	global pose
	
	done = False
	dest = (angle + toGlobal(math.degrees(pose.pose.orientation.z)))%360
  	print "Destination", dest
	while (not done and not rospy.is_shutdown()):
		print "Current Angle", toGlobal(math.degrees(pose.pose.orientation.z))
		error = dest - toGlobal(math.degrees(pose.pose.orientation.z))
		e2 = dest + 360 - toGlobal(math.degrees(pose.pose.orientation.z))
		if (abs(e2) < abs(error)):
			error = e2
		if (abs(error) <= 2):
			done = True
		publishTwist(0, 1.5 * error/90)
		rospy.sleep(0.1)

	publishTwist(0, 0)



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):
	if (msg.state == 1):
        # What should happen when the bumper is pressed?
		pass  # Delete this 'pass' once implemented

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

	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	pose.pose.position.x = position[0]
	pose.pose.position.y = position[1]

	odomW = orientation

	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	
	pose.pose.orientation.z = yaw
	theta = math.degrees(yaw)



# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global pub
	global pose
	global odom_tf
	global odom_list
   
	pose = PoseStamped()
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) 
# Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()
    
 	#make the robot keep doing something...
	rospy.Timer(rospy.Duration(0.01), timerCallback)

    # Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(2, 0))
	print "Starting Lab 2"

   

    # Make the robot do stuff...
	#spinWheels(1, 1, 10)
	#driveStraight(0.5, 1)
	rotate(90)	
	print "Lab 2 complete!"

