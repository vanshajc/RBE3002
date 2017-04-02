from Queue import PriorityQueue
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells


class Astar:
	def __init__(self, start, goal, oc, pub_end, pub_path, pub_visited, pub_frontier):
		self.start = start
		self.goal = goal
		self.frontier = PriorityQueue()
		self.visited = []
		self.path = []
		self.oc = oc
		self.pub_end = pub_end
		self.pub_path = pub_path
		self.pub_visited = pub_visited
		self.pub_frontier = pub_frontier

		visitCells([start, goal], self.pub_end)
		

	def calculate(self):
		self.frontier.put((0, (self.start, [], 0)))
		frontierList = []
		frontierList.append(self.start)		

		while (not self.frontier.empty()):
			(p, (curr, path, cost)) = self.frontier.get()
			frontierList.remove(self.start)

			self.visited.append(curr)
			print 'Current', curr, self.heuristic(curr)
			if (curr == self.goal):
				print 'WE DID IT'
				self.path = path
				break

			for n in self.getAdjacent(curr):
				prio = self.heuristic(n) + cost + self.costTo(curr, n)
				if (not n in self.visited):
					a = [i for i in path]
					a.append(n)
					self.frontier.put((prio, (n, a, cost + self.costTo(curr, n))))
					frontierList.append(n)

			visitCells(frontierList, self.pub_frontier)
			visitCells(self.visited, self.pub_visited)

		print '-----------------------------'
		print self.visited	


	def heuristic(self, point):
		return abs(self.goal.x - point.x) + abs(self.goal.y - point.y)

	def costTo(self, s1, s2):
		if (self.isOccupied(s2)):
			return float('inf')
		return 1

	def getAdjacent(self, s):
		return [self.makePoint(s.x + 1, s.y), self.makePoint(s.x - 1, s.y), self.makePoint(s.x, s.y + 1), self.makePoint(s.x, s.y - 1)] 

	def makePoint(self, x, y):
		p = Point()
		p.x = x;
		p.y = y;
		return p

	def isOccupied(self, p):
		return self.oc.data[int(p.x + p.y*self.oc.info.height)]

def visitCells(lofp, pub):
	
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 0.5
	grid.cell_height = 0.5

	for p in lofp:
		point = Point()
		point.x = p.x*0.5
		point.y = p.y*0.5
		grid.cells.append(point)

	pub.publish(grid)
