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


def padding():
	global oc
	global pad_pts

	pad_pts = []
	noc = [0] * len(oc.data)
	# NOTE: THIS NEEDS TO CHANGE
	for i in range(oc.info.width):
		for j in range(oc.info.height):
			#print 'checking'
			if (oc.data[i + j * oc.info.width] == -1):
				noc[i + j * oc.info.width] == -1
			if (oc.data[i + j * oc.info.width] == 100):
				for rx in range(1):
					for ry in range(1):
						noc = addAround(noc, i + rx, j + ry)
						noc = addAround(noc, i - rx, j + ry)
						noc = addAround(noc, i + rx, j - ry)
						noc = addAround(noc, i - rx, j - ry)

	return noc


def addAround(n, i, j):
	global oc
	global pad_pts
	if (i < 0 or j < 0 or i >= oc.info.width or j >= oc.info.height):
		return n
	p = Point()
	p.x = i
	p.y = j
	pad_pts.append(p)
	n[i + j*oc.info.width] = 100
	return n


def updateMap(oc_grid):
	global oc
	global updatedMap
	global running

	print 'Retreived map'
	oc = oc_grid
	updatedMap = True
	findFrontiers()
	print 'FInsihed'
	#if running:
		#replan()

def c(n):
	r = []
	for i in range(len(n)):
		r.append(n[i]/100)
	return r

def goToGoal(g):
	global oc
	global pose
	global pub_end
	global pub_path
	global pub_visited
	global pub_frontier
	global pub_waypoints
	global pad_pts
	global p1
	global p2
	global running
	global odom_list

	running = True
	print 'HUH?'

	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

	#print 'ORIGIN IS', oc.info.origin.position.x
	pose.pose.position.x = position[0] - oc.info.origin.position.x
	pose.pose.position.y = position[1] - oc.info.origin.position.y
	p1 = Point()
	p1.x = int((pose.pose.position.x / 0.3))
	p1.y = int((pose.pose.position.y/ 0.3))
	print 'Currently at', p1.x, p1.y, pose.pose.position.x, pose.pose.position.y
	p2 = Point()
	print 'goal at', g.pose.position.x - oc.info.origin.position.x, g.pose.position.y - oc.info.origin.position.y 
	p2.x = int((g.pose.position.x - oc.info.origin.position.x)/0.3) # 30
	p2.y = int((g.pose.position.y - oc.info.origin.position.y)/0.3) # 35
	
	noc = OccupancyGrid()
	noc.info = oc.info
	noc.data = oc.data
	noc.data = padding()
	visit(pad_pts, pub_frontier)
	r = Astar(p1, p2, noc, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)

	print 'occupied?', r.isOccupied(p2)
	print 'origin', oc.info.origin.position.x, oc.info.origin.position.y
	print 'robot at', p1.x, p1.y, pose.pose.position.x, pose.pose.position.y
	r.calculate()
	print 'Path length', len(r.path)
	
def replan(e):
	global p1
	global p2
	global oc
	global pad_pts
	global pub_end
	global pub_path
	global pub_visited
	global pub_frontier
	global pub_waypoints
	global pose
	global odom_list	
	print 'Replanning'

	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

	#print 'ORIGIN IS', oc.info.origin.position.x
	pose.pose.position.x = position[0] - oc.info.origin.position.x
	pose.pose.position.y = position[1] - oc.info.origin.position.y
	p1.x = int((pose.pose.position.x / 0.3))
	p1.y = int((pose.pose.position.y/ 0.3))

	if (p1.x == p2.x and p1.y == p2.y):
		return

	noc = OccupancyGrid()
	noc.info = oc.info
	noc.data = oc.data
	noc.data = padding()
	visit(pad_pts, pub_frontier)
	r = Astar(p1, p2, noc, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)

	print 'occupied?', r.isOccupied(p2)
	print 'origin', oc.info.origin.position.x, oc.info.origin.position.y
	print 'robot at', p1.x, p1.y, pose.pose.position.x, pose.pose.position.y
	r.calculate()
	
	
def visit(lofp, pub):
	
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = oc.info.resolution
	grid.cell_height = oc.info.resolution
	
	for p in lofp:
		point = Point()
		point.x = (p.x)*oc.info.resolution + 0.15 + oc.info.origin.position.x
		point.y = (p.y)*oc.info.resolution + 0.15 + oc.info.origin.position.y
		grid.cells.append(point)

	pub.publish(grid)


def findFrontiers():
	global oc
	global marked
	global pub_visited
	fs = []
	marked = [False for x in oc.data]	
	for x in range(oc.info.width):
		for y in range(oc.info.height):
			if (not marked[x + y*oc.info.width] and isFrontier(x, y)):
				#print fs
				fs = addTo(fs, x, y)
	print 'Number of Frontier Regions', len(fs)
	#print fs

	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 0.3
	grid.cell_height = 0.3
	for f in fs:
		print f
		for p in f:
			point = Point()
			point.x = p[0]*0.3 + 0.15 + oc.info.origin.position.x
			point.y = p[1]*0.3 + 0.15 + oc.info.origin.position.y
			grid.cells.append(point)
	pub_visited.publish(grid)
	return fs

def addTo(fs, x, y):
	for f in fs:
		for p in f:
			if (adjacent(p[0], p[1], x, y)):
				f.append((x,y))
				#print 'Added to region'
				return fs
	fs.append([(x,y)])
	#print fs
	return fs

def adjacent(x1, y1, x2, y2):
	return abs(x1-x2) <= 1 and abs(y1-y2) <=1

#Returns if x,y is a frontier
def isFrontier(x, y):
	global oc
	global marked
	#getValue(x, y)
	n = [(x+1, y), (x, y+1), (x-1, y), (x, y-1)]
	#print 'Looking around', x,y
	for p in n:
		if (getValue(x,y) == 0 and getValue(p[0], p[1]) == -1):
			print 'Found Frontier', x,y
			marked[x + y*oc.info.width] = True
			return True
	return False


def getValue(x, y):
	global oc
	if (x < 0 or x >= oc.info.width  or y < 0 or y >= oc.info.height):
		return 100
	return oc.data[x + y*oc.info.width]
	"""
	if (x < 0 or (scale + x*(0.3/oc.info.resolution) >= oc.info.width) or y < 0 or (scale + y*(0.3/oc.info.resolution) >= oc.info.height)):
		return float('inf')
	for i in range(scale):
		x1 = round(i + (x)*(0.3/oc.info.resolution))
		for j in range(scale):
			y1 = round(j + (y)*(0.3/oc.info.resolution))
			print 'trying', x1, y1, x1+y1*oc.info.width, len(oc.data), oc.info.width, oc.info.height
			if oc.data[int(x1 + y1*oc.info.width)] == 100:
				return 100
			elif oc.data[int(x1 + y1*oc.info.width)] == -1:
				return -1			
	return 0
	"""

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('vchowdhary_jwilder_lab4')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global pub
	global pose
	global odom_tf
	global odom_list
	global pub_end
	global pub_path
	global pub_visited
	global pub_frontier
	global pub_pose
	global grid
	global oc
	global updatedMap
	global updatedPose
	global pad_pts
	global p1
	global p2
	global running

	running = False
	pad_pts = []
	updatedMap = False
   	updatedPose = False
	pose = PoseStamped()
	p1 = Point()
	p2 = Point()
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) 
# Publisher for commanding robot motion
	

	pub_end = rospy.Publisher('/EndPoints', GridCells, queue_size=10)
	pub_path = rospy.Publisher('/PathPoints', GridCells, queue_size=10)
	pub_visited = rospy.Publisher('/VisitedPoints', GridCells, queue_size=10)
	pub_frontier = rospy.Publisher('/FrontierPoints', GridCells, queue_size=10)
	pub_waypoints = rospy.Publisher('/Waypoints', GridCells, queue_size=10)
	pub_pose = rospy.Publisher('/lab4_pose', PoseStamped, queue_size=10)
	sub_way = rospy.Subscriber('/WayReached', Point, replan)

	sub_goal = rospy.Subscriber('/GoalPoint', PoseStamped, goToGoal)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, updateMap, queue_size=1)
	odom_list = tf.TransformListener()
    # Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(2, 0))
	
	while (not updatedMap and not rospy.is_shutdown()):
		print 'stuck'
		rospy.sleep(1)

	# while ros_not_shutdown
	while (not rospy.is_shutdown()):
		rospy.sleep(1)
	#oc.data = padding()

	#r = Astar(p1, p2, oc, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)


	#print 'occupied?', r.isOccupied(p2)
	#print 'origin', oc.info.origin.position.x, oc.info.origin.position.y
	#print 'robot at', p1.x, p1.y, pose.pose.position.x, pose.pose.position.y
	#r.calculate()


	
