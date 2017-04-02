from geometry_msgs.msg import Point
class Node:
	def __init__(self, i, j):
		self.p = Point()
		self.p.x = i
		self.p.y = j
		self.visited = False

	def expand(self):
		self.visited = True

	def __eq__(self, other):
		return (self.p.x == other.p.x and self.p.y == other.p.y)
	
	def __hash__(self):
        	return hash(self.p)
