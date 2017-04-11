from Queue import PriorityQueue
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from node import *
import rospy
import math

class Astar:
	def __init__(self, start, goal, oc, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints):
		print 'Initializing'
		self.start = Node(start.x, start.y)
		self.start.cost = 0
		self.goal = Node(goal.x, goal.y)
		self.frontier = PriorityQueue()
		self.path = []
		self.oc = oc
		self.pub_end = pub_end
		self.pub_path = pub_path
		self.pub_visited = pub_visited
		self.pub_frontier = pub_frontier
		self.pub_waypoints = pub_waypoints


		self.nodes = [[0 for x in range(37)] for y in range(37)] 
		for i in range(37):
			for j in range(37):
				self.nodes[i][j] = Node(i, j)

		visitCells([self.start, self.goal], self.pub_end, self.oc)
		#print 'Resolution', oc.info.resolution*oc.info.width
		

	def calculate(self):
		print "Starting A*"
		self.frontier.put((0, (self.start, [self.start])))
		frontierList = []
		visited = set()
		frontierList.append(self.start)		

		while (not self.frontier.empty()):
			(p, (curr, path)) = self.frontier.get()
			frontierList.remove(curr)
			visited.add(curr)
			curr.expand()
			visitCells(visited, self.pub_visited, self.oc)
			#print 'Current', curr.p.x, curr.p.y, curr.cost, p
			if (curr == self.goal):
				self.path = path
				break
			for n in self.getAdjacent(curr):
				prio = self.heuristic(n) + curr.cost + self.costTo(curr, n)
				if ((not self.isOccupied(n.p)) and curr.cost + self.costTo(curr, n) < n.cost):
					a = [i for i in path]
					a.append(n)
					n.cost = curr.cost + self.costTo(curr, n)
					self.frontier.put((prio, (n, a)))
					frontierList.append(n)
			
			#visitCells(frontierList, self.pub_frontier)
			#rospy.sleep(0.1)
		print '-----------------------------'
		#print visited
		#visitCells(visited, self.pub_visited)
		visitCells(self.path, self.pub_path, self.oc)	
		wp = self.findWaypoints(self.path);
		visitCells(wp, self.pub_waypoints, self.oc)	
		visitCells([], self.pub_visited, self.oc)
		#visitCells([], self.pub_frontier)
		print '-----------------------------'

	def within(self, n):
		return n.x <= 37 and n.x >= 0 and n.y >= 0 and n.y <= 37

	def heuristic(self, point):
		return math.sqrt((self.goal.p.x - point.p.x)**2 + (self.goal.p.y - point.p.y)**2)

	def costTo(self, s1, s2):
		#if (self.isOccupied(s2) or self.isOccupied(s1)):
		#	return float('inf')
		return 1

	def getAdjacent(self, s):
		p1 = self.makePoint(s.p.x + 1, s.p.y)
		p2 = self.makePoint(s.p.x - 1, s.p.y)
		p3 = self.makePoint(s.p.x, s.p.y + 1)
		p4 = self.makePoint(s.p.x, s.p.y - 1)
		
 		l = []
		if (p1 != -1 and not p1.visited):
			l.append(p1)
		if (p2 != -1 and not p2.visited):
			l.append(p2)
		if (p3 != -1 and not p3.visited):
			l.append(p3)
		if (p4 != -1 and not p4.visited):
			l.append(p4)

		return l

	def makePoint(self, x, y):
		#p = Point()
		#p.x = x;
		#p.y = y;
		#if (x < 37 and x >= 0 and y >= 0 and y < 37):
		if (abs(x) < self.oc.info.width and abs(y) < self.oc.info.height):
			return self.nodes[x][y]
		return -1

	def isOccupied(self, p):
		a = []
		for i in range(round(0.3/self.oc.info.resolution)):
			x = round(i + (p.x)*(0.3/self.oc.info.resolution))
			x2 = 289
			for j in range(round(0.3/self.oc.info.resolution)):
				y = round(j + (p.y)*(0.3/self.oc.info.resolution))
				y2 = 343
				pp = Point()
				pp.x = x
				pp.y = y				
				a.append(pp)
				print 'Checking:',x,y, self.oc.data[int(x+y*self.oc.info.width)]
				if self.oc.data[int(x + y*self.oc.info.width)] == 100:
					return True		
		visit(a, self.pub_frontier)		
		return False
		#return self.oc.data[int(p.p.x + p.p.y*self.oc.info.height)] == 100

	def findWaypoints(self, path):
		wplist = []
		i = 1
		while i + 1 < len(path):
			prev = path[i-1]
			curr = path[i]
			next = path[i+1]
			print prev.p.x, curr.p.x, next.p.x, prev.p.y, curr.p.y, next.p.y
			if prev.p.x != next.p.x and prev.p.y != next.p.y:
				print "waypoint"
				wplist.append(curr)
			i +=1
		wplist.append(self.goal)
		return wplist

def visitCells(lofp, pub, oc):
	
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 0.3
	grid.cell_height = 0.3

	for p in lofp:
		point = Point()
		point.x = p.p.x*0.3 + 0.15 + oc.info.origin.position.x
		point.y = p.p.y*0.3 + 0.15 + oc.info.origin.position.y
		grid.cells.append(point)


	pub.publish(grid)

def visit(lofp, pub):
	
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 0.075
	grid.cell_height = 0.075
	
	for p in lofp:
		point = Point()
		point.x = (p.x)*0.075 - 0.05 - 15
		point.y = (p.y)*0.075 + 0.05 - 15
		grid.cells.append(point)

	pub.publish(grid)



