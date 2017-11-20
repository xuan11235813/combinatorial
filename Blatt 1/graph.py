import time
import math

class Graph:
	def __init__( self, path ):
		#read the data from files
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
	def readNodeSet(self, path):
		#read node set from file
		vertexSet = []
		try:
			vertexFile = open(path, 'r')
			for line in vertexFile:
				vertexSet.append(line.rstrip())
			
		except IOError as err:
			print("file name do not exist")
		return vertexSet
	
	def getWeight(self, node1, node2):
		return self.edges[str(node1)][str(node2)]
	
	def findMinimalInSet(self, node, nodeToProcess, nodeProcessed):
		# use dijkstra algorithms to calculate the minimal distance between node and any node
		# in node to Process. avoid recalculate the items in nodeProcessed.
		cost = 1000000
		vertex = -1
		path = []
		for v in nodeToProcess:
			if v not in nodeProcessed:
				cost_, path_ = self.processDijkstra(v, node)
				vertex_ = v
				if cost_ < cost:
					cost = cost_
					vertex = vertex_
					path = path_

		return vertex, cost, path

	def findMinimalNeighbor(self, node, processedNode = [], base = 0):
		# find the minimal distance and corresponding point from neighbor points

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
		# implement prim algorithms

		nodeToProcess = self.nodes[:]
		N = len(nodeToProcess)
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
		#print('tree: ')
		#print(tree)

	def processDijkstra(self, node1, node2):
		# implement dijkstra algorithms
		nodeToProcess = self.nodes[:]
		nodeProcessed = {}
		nodePre = {}
		pathNode = []
		nodeToProcess.remove(str(node1))
		nodeProcessed[str(node1)] = 0

		globalCost = 0
		status = 0
		while status == 0:
			cost = 1000000
			vertex = -1
			pre = -1
			for i in nodeProcessed:
				vertex_, cost_ = self.findMinimalNeighbor(i, nodeProcessed, int(nodeProcessed[i]))
				if cost_ <= cost:
					cost = cost_
					vertex = vertex_
					pre = i
			nodeToProcess.remove(str(vertex))
			nodeProcessed[str(vertex)] = cost
			nodePre[str(vertex)] = str(pre)
			globalCost = cost


			if int(vertex) == int(node2):
				status = 1
				pre = '-1'
				thisNode = str(node2)
				while( int(thisNode) != int(node1) ):
					pathNode.append(nodePre[str(thisNode)])
					thisNode = nodePre[thisNode]
		return globalCost, pathNode


	def ProcessHeuristic(self, nodeSet):
		# process with dijkstra heuristic 
		nodeToProcess = nodeSet[:]
		nodeProcessed = []
		nodeProcessed.append(nodeToProcess[0])
		del nodeToProcess[0]
		cost = 0
		while len(nodeToProcess) != 0:
			weight = 1000000
			vertex = -1
			path = []
			
			for i in nodeProcessed:
				
				vertex_, weight_, path_ = self.findMinimalInSet(i, nodeToProcess ,nodeProcessed)
				if weight_ < weight:
					weight = weight_
					vertex = vertex_
					path = path_
			cost += weight
			nodeProcessed.append(str(vertex))
			nodeToProcess.remove(str(vertex))
			for i in path:
				if str(i) not in nodeProcessed:
					nodeProcessed.append(str(i))




		print('cost: '+ str(cost))




graph = Graph('Graph1.lgf')


print('------------aufgabe 1 -----------------')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

print('------------aufgabe 2 -----------------')
startTime = time.time()
cost, path = graph.processDijkstra(2,50)
print('cost of Dijkstra from 2 to 50 is ' + str(cost))
print(path)
print('time: '+str(time.time() - startTime) + 's')


print('------------aufgabe 3 ------------------')
print('#############for algorithm as prim############')
nodeSet = graph.nodes
graph.ProcessHeuristic(nodeSet)


print('#############for algorithm as Dijkstra#######')
nodeSet = ['2','50']
graph.ProcessHeuristic(nodeSet)




print('------------aufgabe 4 a-----------------')
graph = Graph('Graph1.lgf')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

graph = Graph('Graph2.lgf')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

graph = Graph('Graph3.lgf')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

graph = Graph('Graph4.lgf')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

graph = Graph('Graph5.lgf')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

'''
#this is a little bit time consuming
graph = Graph('Graph6.lgf')
startTime = time.time()
graph.processPrim()
print('time: '+str(time.time() - startTime) + 's')

'''

print('------------aufgabe 4 b-----------------')
graph = Graph('Graph1.lgf')
nodeSet = graph.readNodeSet('Graph1_Terminals.txt')
graph.ProcessHeuristic(nodeSet)

print('------------aufgabe 4 c-----------------')
startTime = time.time()
graph = Graph('Graph2.lgf')
nodeSet = graph.readNodeSet('Graph2_Terminals.txt')
graph.ProcessHeuristic(nodeSet)
print('time: '+str(time.time() - startTime) + 's')

startTime = time.time()
graph = Graph('Graph3.lgf')
nodeSet = graph.readNodeSet('Graph3_Terminals.txt')
graph.ProcessHeuristic(nodeSet)
print('time: '+str(time.time() - startTime) + 's')


startTime = time.time()
graph = Graph('Graph4.lgf')
nodeSet = graph.readNodeSet('Graph4_Terminals1.txt')
graph.ProcessHeuristic(nodeSet)
print('time: '+str(time.time() - startTime) + 's')

startTime = time.time()
graph = Graph('Graph4.lgf')
nodeSet = graph.readNodeSet('Graph3_Terminals2.txt')
graph.ProcessHeuristic(nodeSet)
print('time: '+str(time.time() - startTime) + 's')

startTime = time.time()
graph = Graph('Graph4.lgf')
nodeSet = graph.readNodeSet('Graph3_Terminals3.txt')
graph.ProcessHeuristic(nodeSet)
print('time: '+str(time.time() - startTime) + 's')

startTime = time.time()
graph = Graph('Graph4.lgf')
nodeSet = graph.readNodeSet('Graph3_Terminals4.txt')
graph.ProcessHeuristic(nodeSet)
print('time: '+str(time.time() - startTime) + 's')