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
from Queue import PriorityQueue

# Adds padding to the map.
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
				for rx in range(2):
					for ry in range(2):
						noc = addAround(noc, i + rx, j + ry)
						noc = addAround(noc, i - rx, j + ry)
						noc = addAround(noc, i + rx, j - ry)
						noc = addAround(noc, i - rx, j - ry)

	return noc

# Checks to see if i,j is a valid point and if it is, then marks i,j as 100 (occupied).
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

# Update the map when a new message is received
def updateMap(oc_grid):
	global oc
	global updatedMap
	global running

	print 'Retreived map'
	oc = oc_grid
	updatedMap = True
	blocked = []
	findFrontiers()
	#if running:
		#replan()

#Previous lab function. Unnecessary now.
def goToGoal(g):
	#navToGoal(g.pose.position)
	print 'Not implemented anymore'

# Takes a frontier and drives to that frontier.
def navToGoal(f):
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
	global noc
	global blocked
	global path

	running = True
	print 'HUH?'

	updatePosition()
	
	print 'Currently at', p1.x, p1.y, pose.pose.position.x, pose.pose.position.y
	p2 = Point()
	
	g = centroid(f)
	print 'goal at', g[0], g[1]
	p2.x = g[0]#int((g.x - oc.info.origin.position.x)/0.3) # 30
	p2.y = g[1]#int((g.y - oc.info.origin.position.y)/0.3) # 35
	
	r = Astar(p1, p2, noc, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)

	#print 'occupied?', r.isOccupied(p2)
	print 'origin', oc.info.origin.position.x, oc.info.origin.position.y
	print 'robot at', p1.x, p1.y, p2.x, p2.y
	r.calculate()
	print 'Path length', len(r.path)
	path = r.path
	if (len(r.path) > 0):
		'PUBLISHING PATH'
		r.publish()
	else:
		#marked[g.x + g.y*oc.info.width] = True
		print 'Blocked path'
		for p in f:
			blocked.append(p)
		driveToFrontier()

# Replans the current path.
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

	updatePosition()

	if (p1.x == p2.x and p1.y == p2.y):
		return

	print 'Sleeping'
	rospy.sleep(5)
	print 'Driving Frontiers'
	driveToFrontier()
	#noc = OccupancyGrid()
	#noc.info = oc.info
	#noc.data = oc.data
	#noc.data = padding()
	#visit(pad_pts, pub_frontier)
	#r = Astar(p1, p2, noc, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)

	#print 'occupied?', r.isOccupied(p2)
	#print 'origin', oc.info.origin.position.x, oc.info.origin.position.y
	#print 'robot at', p1.x, p1.y, pose.pose.position.x, pose.pose.position.y
	#r.calculate()
	

# Determines which frontier to drive to, and sends instructions to drive to that frontier.
def driveToFrontier():
	global fs
	global p1
	global pub_centroid
	global pq

	findFrontiers()
	pq = PriorityQueue()
	l = []
	updatePosition()
	for f in fs:
		pq.put((distance(centroid(f), p1), f))
		p = Point()
		p.x = centroid(f)[0]
		p.y = centroid(f)[1]
		l.append(p)
		#print 'Centroid', p.x, p.y

	visit(l, pub_centroid)
	g = Point()

	if (len(fs) == 0):
		print 'Finished'
		return

	temp = pq.get()
	g.x = centroid(temp[1])[0]
	g.y = centroid(temp[1])[1]
	p2 = g
	print 'Going to', g
	#print 'Frontier', f
	navToGoal(f)

# Update the position of the robot.
def updatePosition():
	global pose
	global p1
	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

	#print 'ORIGIN IS', oc.info.origin.position.x
	pose.pose.position.x = position[0] - oc.info.origin.position.x
	pose.pose.position.y = position[1] - oc.info.origin.position.y
	p1.x = int((pose.pose.position.x / 0.15))
	p1.y = int((pose.pose.position.y/ 0.15))


# Marks the GridCells in RVIZ. This function is purely for visual testing/debugging.
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

# Finds the frontier regions on the map.
def findFrontiers():
	global oc
	global marked
	global pub_visited
	global p1
	global p2
	global fs
	global noc
	global blocked

	noc = OccupancyGrid()
	noc.info = oc.info
	noc.data = oc.data
	noc.data = padding()

	visit(pad_pts, pub_frontier)

	fs = []
	marked = [False for x in oc.data]	
	for x in range(oc.info.width):
		for y in range(oc.info.height):
			if (not ((x,y) in blocked) and not marked[x + y*oc.info.width] and isFrontier(x, y)):
				#print fs
				fs = addTo(fs, x, y)
	print 'Number of Frontier Regions', len(fs)
	#print fs

	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 0.15
	grid.cell_height = 0.15
	for f in fs:
		#print f
		for p in f:
			point = Point()
			point.x = p[0]*0.15 + 0.075 + oc.info.origin.position.x
			point.y = p[1]*0.15 + 0.075 + oc.info.origin.position.y
			grid.cells.append(point)
	pub_visited.publish(grid)
	return fs

# Helper function for findFrontiers that adds a point to one of the frontier regions.
def addTo(fs, x, y):
	cfr = [(x,y)] #holds the frontier region containing x,y
	nf = [] # new copy of the frontier regions
	#found = False
	for f in fs:
		found = False
		for p in f:
			if (adjacent(p[0], p[1], x, y)):
				cfr.extend(f)
				found = True
				break
		if (not found):
			nf.append(f)			
				#f.append((x,y))
				#print 'Added to region'
				#return fs
	#fs.append([(x,y)])
	nf.append(cfr)	
	#print fs
	return nf

# Calculates the distance between two points.
def distance(a1, a2):
	return abs(a1[0] - a2.x) + abs(a1[1] - a2.y)

#returns centroid of a frontier region
def centroid(fs):
	sumx = 0
	sumy = 0
	for i in fs:
		sumx += i[0]
		sumy += i[1]
	return (sumx/len(fs), sumy/len(fs))

# Determines if two points are adjacent.
def adjacent(x1, y1, x2, y2):
	return abs(x1-x2) <= 1 and abs(y1-y2) <=1

#Returns if x,y is a frontier
def isFrontier(x, y):
	global noc
	global marked
	#getValue(x, y)
	n = [(x+1, y), (x, y+1), (x-1, y), (x, y-1)]
	#print 'Looking around', x,y
	for p in n:
		if (getValue(x,y) == 0 and getValue(p[0], p[1]) == -1):
			#print 'Found Frontier', x,y
			marked[x + y*noc.info.width] = True
			return True
	return False


# Returns the map value at x,y.
def getValue(x, y):
	global noc
	if (x < 0 or x >= noc.info.width  or y < 0 or y >= noc.info.height):
		return 100
	return oc.data[x + y*noc.info.width]

# Creates an astar object to calculate the path. Takes a start, goal, and a map.
def makeAstar(start, goal, data):
	global pub_end
	global pub_path
	global pub_visited
	global pub_frontier
	global pub_waypoints
	return Astar(start, goal, data, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('vchowdhary_jwilder_final')

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
	global pub_centroid
	global grid
	global oc
	global updatedMap
	global updatedPose
	global pad_pts
	global p1
	global p2
	global running
	global blocked

	blocked = []
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
	pub_centroid = rospy.Publisher('/Centroid', GridCells, queue_size=10)
	sub_way = rospy.Subscriber('/WayReached', Point, replan)

	sub_goal = rospy.Subscriber('/GoalPoint', PoseStamped, goToGoal)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, updateMap, queue_size=1)
	odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(2, 0))
	
	while (not updatedMap and not rospy.is_shutdown()):
		print 'stuck'
		rospy.sleep(1)
	
	driveToFrontier()

	while (not rospy.is_shutdown()):
		rospy.sleep(1)


	
