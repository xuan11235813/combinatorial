import time
import math

class Graph:
	def __init__( self, path ):
		self.nodes = []
		self.edges = {}
		try:
			self.graphFile = open(path, 'r')
			status = 0
			for line in self.graphFile:
				if 'label' in line:
					status = 1
				elif 'weight' in line:
					status = 2
				if status == 1:
					for word in line.split(' '):
						word = word.rstrip()
						if word.isdigit():
							self.nodes.append(str(word))
						else:
							break;
				if status == 2:
					s = line.split(' ');
					if len(s) == 3:
						item1 = s[0].rstrip();
						item2 = s[1].rstrip();
						item3 = float(s[2].rstrip());
						dic1 = {item1: item3};
						dic2 = {item2: item3};
						if item1 in self.edges:
							self.edges[item1][item2] = item3
						else:
							self.edges[item1] = dic2
						if item2 in self.edges:
							self.edges[item2][item1] = item3
						else:
							self.edges[item2] = dic1
		except IOError as err:
			print("graph files do not exist")
	
	def getNodesSize(self):
		return len(self.nodes)
	
	def getWeight(self, node1, node2):
		return self.edges[str(node1)][str(node2)]
	
	def findMinimalNeighbor(self, node, processedNode = [], base = 0):
		dic = self.edges[str(node)]
		weight = 1000000
		vertex = -1
		for v in dic:
			if dic[v] < weight and str(v) not in processedNode:
				weight = dic[v]
				vertex =  v
		weight += base

		return vertex, weight

	def processPrim(self):
		nodeToProcess = self.nodes[:]
		nodeProcessed = []
		nodeProcessed.append(nodeToProcess[0])
		del nodeToProcess[0]
		tree = []
		cost = 0
		while len(nodeToProcess) != 0:
			weight = 1000000
			vertex = -1
			dic = {}
			
			for i in nodeProcessed:
				vertex_, weight_ = self.findMinimalNeighbor(i, nodeProcessed, 0)
				if weight_ < weight:
					weight = weight_
					vertex = vertex_
					dic = {str(i):vertex}
			cost += weight
			nodeProcessed.append(str(vertex))
			nodeToProcess.remove(str(vertex))
			tree.append(dic)

		print('minimale spannebaum cost: '+ str(cost))
		print('tree: ')
		print(tree)

	def processDijkstra(self, node1, node2):
		nodeToProcess = self.nodes[:]
		nodeProcessed = {}

		nodeToProcess.remove(str(node1))
		nodeProcessed[str(node1)] = 0

		globalCost = 0
		status = 0
		while status == 0:
			cost = 1000000
			vertex = -1

			for i in nodeProcessed:
				vertex_, cost_ = self.findMinimalNeighbor(i, nodeProcessed, int(nodeProcessed[i]))
				if cost_ < cost:
					cost = cost_
					vertex = vertex_

			nodeToProcess.remove(str(vertex))
			nodeProcessed[str(vertex)] = cost
			globalCost = cost

			if int(vertex) == int(node2):
				status = 1
		return globalCost

	def ProcessHeuristic(self, nodeSet):



graph = Graph('Graph3.lgf')


print('------------aufgabe 1 -----------------')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

print('------------aufgabe 2 -----------------')
cost = graph.processDijkstra(2,256)
print('cost of Dijkstra from 2 to 50 is ' + str(cost))