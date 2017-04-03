from Queue import PriorityQueue
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from node import *
import rospy

class Astar:
	def __init__(self, start, goal, oc, pub_end, pub_path, pub_visited, pub_frontier):
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

		self.nodes = [[0 for x in range(37)] for y in range(37)] 
		for i in range(37):
			for j in range(37):
				self.nodes[i][j] = Node(i, j)

		visitCells([self.start, self.goal], self.pub_end)
		#print 'Resolution', oc.info.resolution*oc.info.width
		

	def calculate(self):
		self.frontier.put((0, (self.start, [])))
		frontierList = []
		visited = set()
		frontierList.append(self.start)		

		while (not self.frontier.empty()):
			(p, (curr, path)) = self.frontier.get()
			frontierList.remove(curr)
			visited.add(curr)
			curr.expand()
			visitCells(visited, self.pub_visited)
			print 'Current', curr.p.x, curr.p.y, curr.cost, p
			if (curr == self.goal):
				self.path = path
				break
			for n in self.getAdjacent(curr):
				prio = self.heuristic(n) + curr.cost + self.costTo(curr, n)
				if ((not self.isOccupied(n)) and curr.cost + self.costTo(curr, n) < n.cost):
					a = [i for i in path]
					a.append(n)
					n.cost = curr.cost + self.costTo(curr, n)
					self.frontier.put((prio, (n, a)))
					frontierList.append(n)
			
			visitCells(frontierList, self.pub_frontier)
			rospy.sleep(0.1)
		print '-----------------------------'
		#print visited
		#visitCells(visited, self.pub_visited)
		visitCells(self.path, self.pub_path)		
		print '-----------------------------'

	def within(self, n):
		return n.x <= 37 and n.x >= 0 and n.y >= 0 and n.y <= 37

	def heuristic(self, point):
		return (self.goal.p.x - point.p.x)**2 + (self.goal.p.y - point.p.y)**2

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
		if (x < 37 and x >= 0 and y >= 0 and y < 37):
			return self.nodes[x][y]
		return -1

	def isOccupied(self, p):
		return self.oc.data[int(p.p.x + p.p.y*self.oc.info.height)] == 100

def visitCells(lofp, pub):
	
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 0.3
	grid.cell_height = 0.3

	for p in lofp:
		point = Point()
		point.x = p.p.x*0.3 + 0.65
		point.y = p.p.y*0.3 + 0.15
		grid.cells.append(point)

	pub.publish(grid)
